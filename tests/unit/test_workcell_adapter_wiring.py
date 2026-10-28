"""Production ownership wrappers delegate transport without changing virtual starts."""
import ast
from itertools import islice
from types import MethodType
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.workcells.planning_adapter import MoveItPlanningAdapter
from tests.unit.test_planning_adapter import request, constraints
from tests.unit.test_execution_adapter import Harness, Handle, answer
from mujoco_ros2_manipulation.workcells.execution_adapter import (
    ExecutionCleanupError, ExecutionLedger, TrajectoryExecutionAdapter, MAX_EXECUTION_GOALS)


def method(name, namespace):
    source = Path(__file__).resolve().parents[2] / "src/mujoco_ros2_manipulation/workcells/verify.py"
    tree = ast.parse(source.read_text(), filename=str(source))
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "WorkcellVerification")
    function = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == name)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])), str(source), "exec"), namespace)
    return namespace[name]


class PlanningWrapperTests(unittest.TestCase):
    plan = staticmethod(method("plan", dict(MoveItPlanningAdapter=MoveItPlanningAdapter,
        GetMotionPlan=NS(Request=request), Constraints=constraints, JointConstraint=NS)))

    def setUp(self):
        self.answer = NS(planning_time=.01, error_code=NS(val=1))
        self.node = NS(joint_names=["left", "right"], planning_timings=[],
            planner=NS(call_async=Mock(return_value="owned planning future")),
            result=Mock(return_value=NS(motion_plan_response=self.answer)),
            state=Mock(return_value=dict(left=.1, right=.2)))

    def test_virtual_waypoint_is_forwarded_without_observing_or_reanchoring_live_state(self):
        self.node.state.side_effect = RuntimeError("virtual planning must not read live state")
        virtual = dict(right=.7, left=.6)
        result = self.plan(self.node, "left_arm", dict(left=.8), virtual,
            planner_id="RRTConnectkConfigDefault", allowed_planning_sec=1.25, goal_tolerance_rad=.00001)
        self.assertIs(result, self.answer)
        self.node.state.assert_not_called()
        motion = self.node.planner.call_async.call_args.args[0].motion_plan_request
        self.assertEqual(motion.start_state.joint_state.name, ["left", "right"])
        self.assertEqual(motion.start_state.joint_state.position, [.6, .7])
        self.assertEqual(motion.goal_constraints[0].joint_constraints[0].joint_name, "left")
        self.assertEqual(self.node.planning_timings[0]["allowed_planning_sec"], 1.25)
        self.assertEqual(self.node.planning_timings[0]["goal_tolerance_rad"], .00001)

    def test_default_start_is_observed_once_and_rpc_failure_is_not_a_planner_response(self):
        self.plan(self.node, "both_arms", dict(left=.3, right=.4))
        self.node.state.assert_called_once_with()
        self.node.result.assert_called_once_with("owned planning future", 25, "shared motion plan")
        self.node.planning_timings.clear()
        problem = TimeoutError("planning service timeout")
        self.node.result.side_effect = problem
        with self.assertRaises(TimeoutError) as caught:
            self.plan(self.node, "both_arms", dict(left=.3, right=.4))
        self.assertIs(caught.exception, problem)
        self.assertEqual(self.node.planning_timings, [])


class ExecutionWrapperTests(unittest.TestCase):
    def node(self, harness):
        namespace = dict(ExecutionLedger=ExecutionLedger,
            TrajectoryExecutionAdapter=TrajectoryExecutionAdapter,
            MAX_EXECUTION_GOALS=MAX_EXECUTION_GOALS, islice=islice,
            FollowJointTrajectory=NS(Goal=NS))
        node = NS(arm_clients=harness.clients, result=harness.result,
            until=harness.until, clock_last=10., dispatched=7, handles=[],
            acceptance_margins=[], commanded=dict(left_j1=.0, right_j1=.0))
        for name in ('_trajectory_executor', 'cancel_peers', 'dispatch_event', 'dispatch'):
            setattr(node, name, MethodType(method(name, namespace), node))
        return node

    def test_generator_unchanged_endpoint_and_counter_visible_during_callback(self):
        harness = Harness()
        node = self.node(harness)
        observed = []
        harness.wait_hook = lambda *_: observed.append(node.dispatched)
        node.dispatch(iter(harness.trajectories), scheduled_start=10.5)
        self.assertTrue(all(count == 9 for count in observed))
        self.assertEqual(node.dispatched, 9)
        self.assertEqual(node.commanded, dict(left_j1=.2, right_j1=.2))
        self.assertIs(node.handles, node.execution_ledger.handles)
        self.assertEqual(node.acceptance_margins, [.5])
        self.assertEqual(node.execution_transport['request_sent_count'], 2)
        for arm, trajectory in harness.trajectories:
            self.assertIs(harness.clients[arm.id].goals[0].trajectory, trajectory)

    def test_unknown_send_preserves_diagnostics_and_never_changes_commanded_target(self):
        harness = Harness()
        node = self.node(harness)
        primary = RuntimeError('transport failed after possible submission')
        harness.clients['left'].send_goal_async = Mock(side_effect=primary)
        with self.assertRaises(ExecutionCleanupError) as caught:
            node.dispatch(harness.trajectories)
        self.assertIs(caught.exception.__cause__, primary)
        self.assertEqual(node.dispatched, 7)
        self.assertEqual(node.commanded, dict(left_j1=0., right_j1=0.))
        self.assertTrue(node.execution_transport['cleanup_uncertain'])
        self.assertEqual(node.execution_transport['request_sent_count'], 0)
        self.assertTrue(node.execution_transport['attempts'][0]['submission_started'])
        self.assertEqual(harness.clients['right'].goals, [])

    def test_abort_hook_resolved_dynamically_and_only_owned_handles_cancelled(self):
        harness = Harness(left=Handle(), right=Handle())
        node = self.node(harness)
        def abort():
            raise RuntimeError('cancel requested while awaiting execution')
        harness.until_hook = lambda: setattr(node, 'dispatch_abort', abort)
        with self.assertRaisesRegex(RuntimeError, 'cancel requested'):
            node.dispatch(harness.trajectories)
        self.assertEqual(harness.left.cancel_calls, 1)
        self.assertEqual(harness.right.cancel_calls, 1)
        self.assertEqual(harness.clients['unrelated'].goals, [])
        self.assertFalse(node.execution_transport['cleanup_uncertain'])

    def test_legacy_owned_handles_use_same_list_and_bounded_cleanup(self):
        harness = Harness()
        node = self.node(harness)
        handle = Handle()
        node.handles.append((handle, handle.result_future))
        owned = node.handles
        node.cancel_peers()
        self.assertIs(node.execution_ledger.handles, owned)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertTrue(node.execution_transport['attempts'][0]['terminal_seen'])
        self.assertEqual(harness.clients['unrelated'].goals, [])
