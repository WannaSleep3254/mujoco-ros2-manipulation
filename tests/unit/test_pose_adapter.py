"""Exercise strict IK/FK transport and candidate validation without ROS."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.workcells.pose_adapter import MoveItPoseAdapter, PoseAdapterError


def stamped(position=(1., 2., 3.), quaternion=(0., 0., 0., 1.), frame="world"):
    return NS(header=NS(frame_id=frame), pose=NS(position=NS(**dict(zip("xyz", position))),
            orientation=NS(**dict(zip("xyzw", quaternion)))))


def robot_state():
    return NS(is_diff=None, joint_state=NS(name=[], position=[]))


def ik_request():
    return NS(ik_request=NS(robot_state=robot_state(), pose_stamped=stamped(), timeout=NS()))


def fk_request():
    return NS(header=NS(), robot_state=robot_state())


class PoseAdapterTests(unittest.TestCase):
    def setUp(self):
        self.names = ["left_b", "left_a", "right_b", "right_a"]
        self.start = dict(right_a=.4, left_a=.2, right_b=.3, left_b=.1)
        self.limits = {name: dict(lower=-2., upper=2.) for name in self.names}
        self.pose = dict(frame_id="world", position_m=[1., 2., 3.], quaternion_xyzw=[0., 0., 0., 1.])
        self.ik_answer = NS(error_code=NS(val=1),
                            solution=NS(joint_state=NS(name=["left_a", "left_b"], position=[.6, .5])))
        self.fk_answer = NS(error_code=NS(val=1), fk_link_names=["left_tip"], pose_stamped=[stamped()])
        self.ik_future, self.fk_future = object(), object()
        self.ik = NS(call_async=Mock(return_value=self.ik_future))
        self.fk = NS(call_async=Mock(return_value=self.fk_future))
        self.wait = Mock(side_effect=lambda future, seconds, description:
                         self.ik_answer if future is self.ik_future else self.fk_answer)
        self.clock = Mock(side_effect=[10., 10.25, 20., 20.5])
        self.adapter = MoveItPoseAdapter(self.ik, self.fk, self.wait, ik_request, fk_request,
                                         self.names, self.limits, monotonic=self.clock)

    def solve(self, **changes):
        args = dict(group="left_arm", tip_frame="left_tip", tip_pose=self.pose,
                    start_rad=self.start, selected_joint_names=["left_b", "left_a"], timeout_sec=1.125)
        args.update(changes)
        return self.adapter.solve_ik(**args)

    def test_ik_full_seed_frame_group_tip_collision_and_bounded_timeout(self):
        target, timing = self.solve()
        request = self.ik.call_async.call_args.args[0].ik_request
        self.assertEqual(request.group_name, "left_arm")
        self.assertEqual(request.ik_link_name, "left_tip")
        self.assertIs(request.avoid_collisions, True)
        self.assertIs(request.robot_state.is_diff, False)
        self.assertEqual(request.robot_state.joint_state.name, self.names)
        self.assertEqual(request.robot_state.joint_state.position, [.1, .2, .3, .4])
        self.assertEqual(request.pose_stamped.header.frame_id, "world")
        self.assertEqual(vars(request.pose_stamped.pose.position), dict(x=1., y=2., z=3.))
        self.assertEqual(vars(request.pose_stamped.pose.orientation), dict(x=0., y=0., z=0., w=1.))
        self.assertEqual((request.timeout.sec, request.timeout.nanosec), (1, 125000000))
        self.wait.assert_called_once_with(self.ik_future, 1.125, "pose IK")
        self.assertEqual(target, dict(left_b=.5, left_a=.6, right_b=.3, right_a=.4))
        self.assertEqual(timing, dict(group="left_arm", tip_frame="left_tip", frame_id="world", error_code=1,
            selected_joint_names=["left_b", "left_a"], avoid_collisions=True, timeout_sec=1.125,
            rpc_timeout_sec=1.125, wall_sec=.25))

    def test_ik_computation_budget_and_rpc_wall_wait_are_independent(self):
        _, timing = self.solve(timeout_sec=.25, rpc_timeout_sec=2.)
        request = self.ik.call_async.call_args.args[0].ik_request
        self.assertEqual((request.timeout.sec, request.timeout.nanosec), (0, 250000000))
        self.wait.assert_called_once_with(self.ik_future, 2., "pose IK")
        self.assertEqual((timing["timeout_sec"], timing["rpc_timeout_sec"]), (.25, 2.))

    def test_full_response_other_arm_values_are_checked_but_never_applied(self):
        self.ik_answer.solution.joint_state = NS(name=["right_a", "left_a", "right_b", "left_b"],
                                                 position=[1.4, .6, 1.3, .5])
        target, _ = self.solve()
        self.assertEqual((target["right_a"], target["right_b"]), (.4, .3))
        self.assertEqual((target["left_a"], target["left_b"]), (.6, .5))
        self.assertEqual(self.start, dict(right_a=.4, left_a=.2, right_b=.3, left_b=.1))

    def test_ik_response_duplicates_coverage_unknown_names_and_all_values_are_validated(self):
        rows = [("duplicate", ["left_a", "left_a", "left_b"], [.6, .6, .5]),
                ("missing", ["left_a"], [.6]), ("unknown", ["left_a", "left_b", "extra"], [.6, .5, 0.]),
                ("length", ["left_a", "left_b"], [.6]),
                ("nonfinite", ["left_a", "left_b"], [float("nan"), .5]),
                ("boolean", ["left_a", "left_b"], [True, .5]),
                ("limit", ["left_a", "left_b"], [2.01, .5]),
                ("other_arm_limit", self.names, [.5, .6, .3, 2.01]),
                ("other_arm_nonfinite", self.names, [.5, .6, .3, float("inf")])]
        for label, names, values in rows:
            with self.subTest(label=label):
                self.setUp()
                self.ik_answer.solution.joint_state = NS(name=names, position=values)
                with self.assertRaises(PoseAdapterError) as caught:
                    self.solve()
                self.assertEqual(caught.exception.kind, "ik_response_invalid")
                self.ik.call_async.assert_called_once()
                self.fk.call_async.assert_not_called()

    def test_ik_no_solution_timeout_and_other_error_codes_are_distinct(self):
        for code, kind in ((-31, "ik_no_solution"), (-6, "ik_timeout"), (-1, "ik_failed"),
                           ("1", "ik_response_invalid")):
            with self.subTest(code=code):
                self.setUp()
                self.ik_answer.error_code.val = code
                with self.assertRaises(PoseAdapterError) as caught:
                    self.solve()
                self.assertEqual(caught.exception.kind, kind)
                self.assertEqual(caught.exception.error_code, code if type(code) is int else None)
                self.ik.call_async.assert_called_once()

    def test_timeout_and_transport_errors_preserve_cause_and_do_not_retry(self):
        for where, error, kind in (("send", ConnectionError("disconnected"), "ik_transport"),
                                    ("wait", TimeoutError("deadline"), "ik_timeout"),
                                    ("wait", RuntimeError("Timeout: pose IK"), "ik_timeout"),
                                    ("wait", RuntimeError("No result: pose IK"), "ik_transport")):
            with self.subTest(where=where, kind=kind):
                self.setUp()
                if where == "send":
                    self.ik.call_async.side_effect = error
                else:
                    self.wait.side_effect = error
                with self.assertRaises(PoseAdapterError) as caught:
                    self.solve()
                self.assertEqual(caught.exception.kind, kind)
                self.assertIs(caught.exception.__cause__, error)
                self.ik.call_async.assert_called_once()
                self.assertLessEqual(self.wait.call_count, 1)

    def test_caller_cancellation_and_budget_escape_send_or_wait_unchanged(self):
        class Cancelled(RuntimeError):
            pass
        class BudgetExceeded(RuntimeError):
            pass
        for operation in ("ik", "fk"):
            for where in ("send", "wait"):
                for kind in (Cancelled, BudgetExceeded):
                    with self.subTest(operation=operation, where=where, kind=kind.__name__):
                        self.setUp()
                        self.adapter = MoveItPoseAdapter(self.ik, self.fk, self.wait,
                            ik_request, fk_request, self.names, self.limits,
                            passthrough_exceptions=(Cancelled, BudgetExceeded))
                        error = kind("owner control event")
                        client = self.ik if operation == "ik" else self.fk
                        if where == "send":
                            client.call_async.side_effect = error
                        else:
                            self.wait.side_effect = error
                        with self.assertRaises(kind) as caught:
                            if operation == "ik":
                                self.solve()
                            else:
                                self.adapter.forward_kinematics("left_tip", self.start)
                        self.assertIs(caught.exception, error)
                        self.assertIsNone(caught.exception.__cause__)
                        client.call_async.assert_called_once()
                        self.assertLessEqual(self.wait.call_count, 1)

    def test_control_exception_configuration_requires_exception_type_tuple(self):
        for invalid in (None, [], (RuntimeError("instance"),), (object,), (BaseException,)):
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ValueError, "tuple"):
                MoveItPoseAdapter(self.ik, self.fk, self.wait, ik_request, fk_request,
                    self.names, self.limits, passthrough_exceptions=invalid)

    def test_unconfigured_runtime_error_remains_transport_and_keyboard_interrupt_escapes(self):
        self.wait.side_effect = RuntimeError("caller forgot to register this type")
        with self.assertRaises(PoseAdapterError) as caught:
            self.solve()
        self.assertEqual(caught.exception.kind, "ik_transport")
        self.setUp()
        error = KeyboardInterrupt()
        self.wait.side_effect = error
        with self.assertRaises(KeyboardInterrupt) as caught:
            self.solve()
        self.assertIs(caught.exception, error)

    def test_failed_wait_removes_pending_request_and_cancels_future_without_masking_primary(self):
        for primary in (TimeoutError("deadline"), ConnectionError("disconnected")):
            with self.subTest(primary=type(primary).__name__):
                self.setUp()
                future = NS(cancel=Mock(side_effect=RuntimeError("cleanup failed")))
                self.ik.call_async.return_value = future
                self.ik.remove_pending_request = Mock(side_effect=RuntimeError("already removed"))
                self.wait.side_effect = primary
                with self.assertRaises(PoseAdapterError) as caught:
                    self.solve()
                self.assertIs(caught.exception.__cause__, primary)
                self.ik.remove_pending_request.assert_called_once_with(future)
                future.cancel.assert_called_once_with()

    def test_owner_cancellation_cleans_pending_rpc_without_turning_it_into_service_error(self):
        class Cancelled(RuntimeError):
            pass
        error = Cancelled("cancelled by owner")
        future = NS(cancel=Mock())
        self.ik.call_async.return_value = future
        self.ik.remove_pending_request = Mock()
        self.wait.side_effect = error
        self.adapter = MoveItPoseAdapter(self.ik, self.fk, self.wait,
            ik_request, fk_request, self.names, self.limits,
            passthrough_exceptions=(Cancelled,))
        with self.assertRaises(Cancelled) as caught:
            self.solve()
        self.assertIs(caught.exception, error)
        self.ik.remove_pending_request.assert_called_once_with(future)
        future.cancel.assert_called_once_with()

    def test_successful_rpc_does_not_cancel_or_remove_completed_response(self):
        future = NS(cancel=Mock())
        self.ik.call_async.return_value = future
        self.ik.remove_pending_request = Mock()
        self.wait.side_effect = None
        self.wait.return_value = self.ik_answer
        self.solve()
        future.cancel.assert_not_called()
        self.ik.remove_pending_request.assert_not_called()

    def test_ik_validation_rejects_wrong_state_group_frame_joint_set_and_time_before_send(self):
        cases = [dict(group=""), dict(tip_frame="/world"), dict(tip_pose=dict(self.pose, frame_id="base")),
                 dict(start_rad={}), dict(start_rad=dict(self.start, left_a=True)),
                 dict(start_rad=dict(self.start, left_a=3.)), dict(start_rad=dict(self.start, unknown=0.)),
                 dict(selected_joint_names=[]), dict(selected_joint_names=["left_a", "left_a"]),
                 dict(selected_joint_names=["missing"]), dict(timeout_sec=0), dict(timeout_sec=5.01),
                 dict(timeout_sec=float("inf")), dict(timeout_sec=True), dict(timeout_sec=1e-12),
                 dict(rpc_timeout_sec=0), dict(rpc_timeout_sec=5.01), dict(rpc_timeout_sec=True),
                 dict(rpc_timeout_sec=float("nan"))]
        for changes in cases:
            with self.subTest(changes=changes):
                self.setUp()
                with self.assertRaises(ValueError):
                    self.solve(**changes)
                self.ik.call_async.assert_not_called()
                self.wait.assert_not_called()

    def test_constructor_owns_joint_order_and_limits_and_seed_snapshot_is_detached(self):
        self.names.reverse()
        self.limits["left_a"]["upper"] = -1.
        def send(request):
            self.start["right_a"] = 1.8
            self.pose["position_m"][0] = 99
            return self.ik_future
        self.ik.call_async.side_effect = send
        target, _ = self.solve()
        request = self.ik.call_async.call_args.args[0].ik_request
        self.assertEqual(request.robot_state.joint_state.name, ["left_b", "left_a", "right_b", "right_a"])
        self.assertEqual(request.robot_state.joint_state.position, [.1, .2, .3, .4])
        self.assertEqual(request.pose_stamped.pose.position.x, 1.)
        self.assertEqual(target["right_a"], .4)

    def test_fk_full_state_request_exact_tip_and_world_response(self):
        pose, timing = self.adapter.forward_kinematics("left_tip", self.start, timeout_sec=.75)
        request = self.fk.call_async.call_args.args[0]
        self.assertEqual(request.header.frame_id, "world")
        self.assertEqual(request.fk_link_names, ["left_tip"])
        self.assertIs(request.robot_state.is_diff, False)
        self.assertEqual(request.robot_state.joint_state.name, self.names)
        self.assertEqual(request.robot_state.joint_state.position, [.1, .2, .3, .4])
        self.wait.assert_called_once_with(self.fk_future, .75, "pose FK")
        self.assertEqual(pose, self.pose)
        self.assertEqual(timing, dict(tip_frame="left_tip", frame_id="world", error_code=1,
                                     timeout_sec=.75, wall_sec=.25))
        self.fk_answer.pose_stamped[0].pose.position.x = 99
        self.assertEqual(pose["position_m"][0], 1.)

    def test_fk_rejects_missing_wrong_tip_frame_and_invalid_pose(self):
        answers = [NS(error_code=NS(val=1), fk_link_names=["right_tip"], pose_stamped=[stamped()]),
                   NS(error_code=NS(val=1), fk_link_names=["left_tip"], pose_stamped=[]),
                   NS(error_code=NS(val=1), fk_link_names=["left_tip"], pose_stamped=[stamped(frame="base")]),
                   NS(error_code=NS(val=1), fk_link_names=["left_tip"], pose_stamped=[stamped(quaternion=(0, 0, 0, 0))]),
                   NS(error_code=NS(val=1), fk_link_names=["left_tip"], pose_stamped=[stamped(position=(True, 0, 0))]),
                   NS(error_code=NS(val=1)), NS()]
        for answer in answers:
            with self.subTest(answer=answer):
                self.setUp()
                self.fk_answer = answer
                with self.assertRaises(PoseAdapterError) as caught:
                    self.adapter.forward_kinematics("left_tip", self.start)
                self.assertEqual(caught.exception.kind, "fk_response_invalid")

    def test_fk_service_failure_timeout_transport_and_invalid_seed_are_distinct(self):
        for code, kind in ((-6, "fk_timeout"), (-21, "fk_failed")):
            with self.subTest(code=code):
                self.setUp()
                self.fk_answer.error_code.val = code
                with self.assertRaises(PoseAdapterError) as caught:
                    self.adapter.forward_kinematics("left_tip", self.start)
                self.assertEqual(caught.exception.kind, kind)
        self.setUp()
        self.fk.call_async.side_effect = OSError("closed")
        with self.assertRaises(PoseAdapterError) as caught:
            self.adapter.forward_kinematics("left_tip", self.start)
        self.assertEqual(caught.exception.kind, "fk_transport")
        self.setUp()
        with self.assertRaises(ValueError):
            self.adapter.forward_kinematics("left_tip", {})
        self.fk.call_async.assert_not_called()

    def test_module_import_needs_no_ros_engine_numpy_or_yaml(self):
        source_root = Path(__file__).resolve().parents[2] / "src"
        code = '''
import importlib.abc, json, sys
class BlockBackend(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy','moveit_msgs','sensor_msgs','mujoco','numpy','yaml'}:
            raise RuntimeError('Unexpected backend import: ' + fullname)
sys.meta_path.insert(0, BlockBackend())
from mujoco_ros2_manipulation.workcells.pose_adapter import MoveItPoseAdapter
from mujoco_ros2_manipulation.workcells.pose_goal import ToolDefinition
print(json.dumps({'scope': ToolDefinition('reference', 'left_tip').record()['scope']}))
'''
        result = subprocess.run([sys.executable, "-c", code], cwd="/tmp", capture_output=True,
                                text=True, env=dict(os.environ, PYTHONPATH=str(source_root)))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["scope"], "origin_reference_only")
