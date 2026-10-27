"""Exercise the real planning adapter with ROS-message-like factories, without ROS."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.workcells.planning_adapter import MoveItPlanningAdapter


def request():
    return NS(motion_plan_request=NS(start_state=NS(joint_state=NS())))


def constraints():
    return NS(joint_constraints=[])


class PlanningAdapterTests(unittest.TestCase):
    def setUp(self):
        self.names = ["left_b", "left_a", "right_b", "right_a"]
        self.start = dict(right_a=.4, left_a=.2, right_b=.3, left_b=.1)
        self.target = dict(left_a=.6, left_b=.5)
        self.answer = NS(planning_time=.08, error_code=NS(val=1),
                         trajectory=NS(joint_trajectory=NS(points=["raw"])))
        self.future = object()
        self.client = NS(call_async=Mock(return_value=self.future))
        self.wait = Mock(return_value=NS(motion_plan_response=self.answer))
        self.clock = Mock(side_effect=[10., 10.25])
        self.adapter = MoveItPlanningAdapter(self.client, self.wait, request, constraints, NS,
                                              self.names, monotonic=self.clock)

    def test_exact_default_request_partial_goal_and_full_start_order(self):
        answer, timing = self.adapter.plan_joint_target("left_arm", self.target, self.start)
        self.assertIs(answer, self.answer)
        self.client.call_async.assert_called_once()
        self.wait.assert_called_once_with(self.future, 25, "shared motion plan")
        motion = self.client.call_async.call_args.args[0].motion_plan_request
        self.assertEqual(motion.group_name, "left_arm")
        self.assertEqual(motion.pipeline_id, "ompl")
        self.assertEqual(motion.planner_id, "")
        self.assertEqual(motion.num_planning_attempts, 3)
        self.assertEqual(motion.allowed_planning_time, 5.)
        self.assertEqual(motion.max_velocity_scaling_factor, .1)
        self.assertEqual(motion.max_acceleration_scaling_factor, .1)
        self.assertTrue(motion.start_state.is_diff)
        self.assertEqual(motion.start_state.joint_state.name, self.names)
        self.assertEqual(motion.start_state.joint_state.position, [.1, .2, .3, .4])
        self.assertEqual(len(motion.goal_constraints), 1)
        joints = motion.goal_constraints[0].joint_constraints
        self.assertEqual([joint.joint_name for joint in joints], ["left_a", "left_b"])
        self.assertEqual([joint.position for joint in joints], [.6, .5])
        self.assertTrue(all(joint.tolerance_above == joint.tolerance_below == .003
                            and joint.weight == 1. for joint in joints))
        self.assertEqual(timing, dict(group="left_arm", wall_sec=.25,
            server_planning_sec=.08, requested_pipeline_id="ompl", requested_planner_id="",
            selected_planner_id=None, attempts=3, allowed_planning_sec=5.,
            goal_tolerance_rad=.003, error_code=1))

    def test_explicit_planner_budget_tolerance_and_raw_failure_code_are_preserved(self):
        self.answer.error_code.val = -31
        answer, timing = self.adapter.plan_joint_target("left_arm", self.target, self.start,
            "RRTConnectkConfigDefault", 1.375, .0001)
        self.assertIs(answer, self.answer)
        self.assertEqual(timing["error_code"], -31)
        self.assertEqual(timing["requested_planner_id"], "RRTConnectkConfigDefault")
        self.assertEqual(timing["allowed_planning_sec"], 1.375)
        self.assertEqual(timing["goal_tolerance_rad"], .0001)
        motion = self.client.call_async.call_args.args[0].motion_plan_request
        self.assertEqual(motion.allowed_planning_time, 1.375)
        self.assertTrue(all(joint.tolerance_above == .0001 for joint in motion.goal_constraints[0].joint_constraints))

    def test_request_is_detached_and_never_replaces_a_virtual_start_with_live_state(self):
        self.names.reverse()  # Constructor owns its configured ordering.
        expected_start, expected_target = deepcopy(self.start), deepcopy(self.target)
        def send(motion_request):
            self.start.update({name: 999. for name in self.start})
            self.target.clear()
            return self.future
        self.client.call_async.side_effect = send
        self.adapter.plan_joint_target("left_arm", self.target, self.start)
        motion = self.client.call_async.call_args.args[0].motion_plan_request
        self.assertEqual(motion.start_state.joint_state.name, ["left_b", "left_a", "right_b", "right_a"])
        self.assertEqual(motion.start_state.joint_state.position,
                         [expected_start[name] for name in motion.start_state.joint_state.name])
        self.assertEqual({joint.joint_name: joint.position for joint in motion.goal_constraints[0].joint_constraints}, expected_target)

    def test_transport_wait_and_factory_errors_propagate_without_retry_or_reclassification(self):
        for where in ("send", "wait", "factory"):
            with self.subTest(where=where):
                self.setUp()
                error = RuntimeError("Original " + where + " failure")
                if where == "send":
                    self.client.call_async.side_effect = error
                elif where == "wait":
                    self.wait.side_effect = error
                else:
                    self.adapter.request_factory = Mock(side_effect=error)
                with self.assertRaises(RuntimeError) as caught:
                    self.adapter.plan_joint_target("left_arm", self.target, self.start)
                self.assertIs(caught.exception, error)
                self.assertLessEqual(self.client.call_async.call_count, 1)
                self.assertLessEqual(self.wait.call_count, 1)

    def test_new_request_and_constraints_are_created_for_each_call(self):
        self.clock.side_effect = [1., 1.1, 2., 2.2]
        self.adapter.plan_joint_target("left_arm", self.target, self.start)
        self.adapter.plan_joint_target("right_arm", dict(right_a=-.5), self.start)
        first, second = [call.args[0].motion_plan_request for call in self.client.call_async.call_args_list]
        self.assertIsNot(first, second)
        self.assertIsNot(first.goal_constraints[0], second.goal_constraints[0])
        self.assertEqual(len(first.goal_constraints[0].joint_constraints), 2)
        self.assertEqual(len(second.goal_constraints[0].joint_constraints), 1)

    def test_invalid_joint_coverage_units_and_parameters_fail_before_transport(self):
        cases = [dict(start_rad=None), dict(start_rad=dict(self.start, unknown=1.)),
                 dict(start_rad={name: value for name, value in self.start.items() if name != "left_a"}),
                 dict(start_rad=dict(self.start, left_a=float("nan"))),
                 dict(target_rad={}), dict(target_rad=dict(unknown=.1)),
                 dict(target_rad=dict(left_a=True)), dict(target_rad=dict(left_a=float("inf"))),
                 dict(allowed_planning_sec=0), dict(allowed_planning_sec=float("nan")),
                 dict(goal_tolerance_rad=-.001), dict(goal_tolerance_rad=True),
                 dict(group=""), dict(planner_id=None)]
        for change in cases:
            with self.subTest(change=change):
                self.setUp()
                parameters = dict(group="left_arm", target_rad=self.target, start_rad=self.start,
                                  planner_id="", allowed_planning_sec=5., goal_tolerance_rad=.003)
                parameters.update(change)
                with self.assertRaises(ValueError):
                    self.adapter.plan_joint_target(**parameters)
                self.client.call_async.assert_not_called()
                self.wait.assert_not_called()
        for names in (None, "left", b"left", [], ["left", "left"], ["left", ""], ["left", 1], [["left"]]):
            with self.subTest(names=names), self.assertRaises(ValueError):
                MoveItPlanningAdapter(self.client, self.wait, request, constraints, NS, names)

    def test_module_import_requires_no_ros_engine_yaml_or_backend_packages(self):
        source_root = Path(__file__).resolve().parents[2] / "src"
        code = '''
import importlib.abc, json, sys
class BlockBackend(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'rclpy','moveit_msgs','sensor_msgs','mujoco','yaml','numpy'}:
            raise RuntimeError('Unexpected backend import: ' + fullname)
sys.meta_path.insert(0, BlockBackend())
from mujoco_ros2_manipulation.workcells.planning_adapter import MoveItPlanningAdapter
print(json.dumps({'imported': True}))
'''
        result = subprocess.run([sys.executable, "-c", code], cwd="/tmp", capture_output=True,
                                text=True, env=dict(os.environ, PYTHONPATH=str(source_root)))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(json.loads(result.stdout)["imported"])
