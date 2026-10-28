"""Production review/dispatch branches with ROS transport and physics replaced.

The real class methods, splines, sampling, clearance gates and traces run. This
does not validate a live ROS scene, middleware, hardware identity or controllers.
"""
import ast
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import uuid

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.motion_request import (
    command, decode_message, request_id, validate_command, validate_cancel, validate_approval)
from mujoco_ros2_manipulation.workcells.motion_review import PlanReviewStore, ReviewRejected
from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path, evaluate_states, sample_schedule
from mujoco_ros2_manipulation.workcells.motion_diagnostics import measure_endpoints, path_clearance_diagnostic
from mujoco_ros2_manipulation.workcells.planning_budget import PlanningBudget, PlanningBudgetExceeded
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.environment_clearance import EnvironmentClearanceRejected
from mujoco_ros2_manipulation.workcells.operation_contract import describe_operation
from mujoco_ros2_manipulation.workcells.execution_adapter import ExecutionLedger, ExecutionAttempt
from mujoco_ros2_manipulation.workcells.pose_planning import PosePlanningMixin
from mujoco_ros2_manipulation.workcells.pose_adapter import PoseAdapterError
from mujoco_ros2_manipulation.workcells.pose_goal import validate_pose_goal
from mujoco_ros2_manipulation.workcells.pose_tools import validate_goal_context, describe_pose_tools
from mujoco_ros2_manipulation.workcells.planning_config import (
    PLANNER_TYPES, planning_configuration, canonical_sha256, trajectory_cache_identity)
from mujoco_ros2_manipulation.workcells.trajectory_cache import TrajectoryCache
from mujoco_ros2_manipulation.workcells.cache_dynamics import validate_reanchored_dynamics, CacheDynamicsRejected


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def load_server():
    source = Path(__file__).resolve().parents[2]/"src/mujoco_ros2_manipulation/workcells/motion_server.py"
    tree = ast.parse(source.read_text(),filename=str(source))
    body = [v for v in tree.body if isinstance(v,ast.ClassDef)]
    namespace = dict(WorkcellVerification=object,InterferenceVerification=SimpleNamespace(split=lambda *a:None),
        PosePlanningMixin=PosePlanningMixin,PoseAdapterError=PoseAdapterError,
        validate_pose_goal=validate_pose_goal,
        PLANNER_TYPES=PLANNER_TYPES, planning_configuration=planning_configuration,
        canonical_sha256=canonical_sha256,trajectory_cache_identity=trajectory_cache_identity,
        TrajectoryCache=TrajectoryCache,validate_reanchored_dynamics=validate_reanchored_dynamics,
        CacheDynamicsRejected=CacheDynamicsRejected,
        validate_goal_context=validate_goal_context,describe_pose_tools=describe_pose_tools,
        deepcopy=deepcopy,Path=Path,json=json,time=time,uuid=uuid,RequestTrace=RequestTrace,
        PlanReviewStore=PlanReviewStore,ReviewRejected=ReviewRejected,request_id=request_id,
        decode_message=decode_message,validate_command=validate_command,validate_cancel=validate_cancel,
        validate_approval=validate_approval,require=require,Spline=Spline,PlanningBudget=PlanningBudget,
        PlanningBudgetExceeded=PlanningBudgetExceeded,PlanCandidatesExhausted=type("Candidates",(RuntimeError,),{}),
        EnvironmentClearanceRejected=EnvironmentClearanceRejected,file_sha256=file_sha256,
        describe_operation=describe_operation,
        TimingRejected=type("Timing",(RuntimeError,),{}),measure_endpoints=measure_endpoints,
        path_clearance_diagnostic=path_clearance_diagnostic,evaluate_path=evaluate_path,
        evaluate_states=evaluate_states,sample_schedule=sample_schedule,MAX_PLAN_CANDIDATES=12,
        rclpy=SimpleNamespace(spin_once=Mock()),to_ros=lambda curve,*a:curve.record())
    exec(compile(ast.fix_missing_locations(ast.Module(body=body,type_ignores=[])),str(source),"exec"),namespace)
    return namespace["MotionCommandServer"]


Server = load_server()


class ReviewServerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        n = self.node = Server.__new__(Server)
        n.session_id = "a"*32
        n.directory = Path(self.temp.name)
        n.joint_names = ["left","right"]
        n.cell = SimpleNamespace(joints=n.joint_names,instances=[
            SimpleNamespace(id="left",joints=["left"]),SimpleNamespace(id="right",joints=["right"])])
        n.metadata = dict(joint_limits={name:dict(lower=-1,upper=1) for name in n.joint_names})
        n.policy = dict(min_inter_arm_distance_m=.02,tracking_reserve_m=.01,sample_dt_sec=.01)
        n.environment_policy = dict(schema_version=1,min_environment_distance_m=.005,tracking_reserve_m=.01)
        n.environment_threshold_m = .015
        n.environment_policy_path = n.directory / "environment-policy.yaml"
        n.environment_policy_path.write_text("test policy snapshot")
        n.environment_policy_sha256 = file_sha256(n.environment_policy_path)
        n.review_store = PlanReviewStore(n.session_id)
        n.review_identity = None
        self.identity = dict(build=dict(model="base",policy="timing"),planning_scene_sha256="b"*64)
        n._review_build_identity = Mock(return_value=deepcopy(self.identity["build"]))
        n._review_runtime_identity = Mock(return_value=deepcopy(self.identity))
        n.actual = dict(left=0.0,right=0.0)
        n.state = lambda:dict(n.actual)
        n.commanded = dict(n.actual)
        n.pending = n.active = None
        n.blocked = n.cancel_requested = n.stop_requested = False
        n.initialized = True
        n.contact_guard = dict(latched=False)
        n.clock_last,n.state_stamp = 10.0,10.0
        n.clock_backwards = False
        n.state_received_ns = n.clock_received_ns = time.monotonic_ns()
        n.replies,n.observations,n.handles = {},[],[]
        n.dispatched = 0
        n.compute_budget,n.compute_budget_sec = None,30.0
        n._last_compute_spin_ns = 0
        n._defer_trace_persist = False
        n.publish = Mock()
        n.quiet = Mock()
        n.get_logger = lambda:SimpleNamespace(warning=Mock())
        n.validity_of = Mock(return_value=SimpleNamespace(valid=True))
        n.evaluator = SimpleNamespace(at=Mock(return_value=dict(distance_m=.1,tips_m=[[0,0,0],[1,0,0]],
            environment=dict(distance_m=.1,bodies=["left","fixture"],
                nearest_points_m=[[0,0,0],[.1,0,0]],pair_count=2,
                scope="moving_robot_vs_fixed_environment"))))
        self.curve = Spline(["left","right"],[
            dict(t=0.0,p=[0.0,0.0],v=[],a=[]),dict(t=.1,p=[.1,.1],v=[],a=[])])
        n.coordinated_path = Mock(return_value=self.curve)
        n.split = Mock(return_value=[("left","left curve"),("right","right curve")])
        def dispatch(commands,**kwargs):
            n.dispatched += len(commands)
            n.observations = [dict(simulation_sec=10.,state_rad=dict(n.actual)),
                              dict(simulation_sec=10.1,state_rad=dict(left=.1,right=.1))]
            n.commanded = dict(left=.1,right=.1)
        n.dispatch = Mock(side_effect=dispatch)
        n.settle = Mock()

    def send(self,payload,kind="plan"):
        getattr(self.node,{"plan":"receive_plan","approve":"receive_approval","command":"receive","cancel":"cancel"}[kind])(
            SimpleNamespace(data=json.dumps(payload)))

    def make_review(self, *, target=None):
        payload = command(target or dict(left=.1,right=.1))
        self.send(payload)
        self.node.process_next()
        record = json.loads((self.node.directory/(payload["request_id"]+".json")).read_text())
        self.assertEqual(record["status"],"succeeded",record.get("error"))
        self.assertEqual(record["dispatched_goals"],0)
        self.assertEqual(self.node.replies[payload["request_id"]]["stage"],"plan_ready")
        review = self.node.review_store.summary()
        approval = dict(schema_version=1,request_id=uuid.uuid4().hex,plan_request_id=payload["request_id"],
            plan_sha256=review["plan_sha256"],session_id=self.node.session_id)
        return payload,record,approval

    def test_plan_runs_both_clearance_and_moveit_checks_without_dispatching(self):
        payload,record,_ = self.make_review()
        self.node.dispatch.assert_not_called()
        self.assertEqual(record["mode"],"plan_only")
        self.assertEqual(record["trajectories"],[self.curve.record()])
        self.assertGreater(record["moveit_safety_samples"],2)
        self.assertTrue(record["planned_clearance"]["complete"])
        self.assertEqual(record["review"]["state"],"awaiting_approval")
        self.assertEqual(record["events"][-1]["name"],"terminal")
        self.assertEqual(record["review_payload"]["trajectories"],record["trajectories"])
        operation = self.node.replies[payload["request_id"]]["operation"]
        self.assertEqual(operation["source_stage"], "plan_ready")
        self.assertEqual(operation["phase"], "review")
        self.assertEqual(operation["mode"], "plan_only")
        self.assertEqual(operation["plan_request_id"], payload["request_id"])
        self.assertIsNone(operation["execution_request_id"])
        self.assertEqual(operation["events"]["retained_count"], 1)
        self.assertGreater(operation["events"]["omitted_prefix_count"], 0)

    def test_approved_curve_is_rechecked_against_fixed_environment_before_dispatch(self):
        _, _, approval = self.make_review()
        self.send(approval, "approve")
        baseline = deepcopy(self.node.evaluator.at.return_value)
        def assess(state):
            value = deepcopy(baseline)
            if .03 < state["left"] < .07:
                value["environment"]["distance_m"] = .002
            return value
        self.node.evaluator.at.side_effect = assess
        self.node.process_next()
        value = json.loads((self.node.directory / (approval["request_id"] + ".json")).read_text())
        self.assertEqual(value["status"], "failed")
        self.assertEqual(value["failure_kind"], "environment_clearance")
        self.assertEqual(value["planned_clearance"]["environment_minimum"]["distance_m"], .002)
        self.node.dispatch.assert_not_called()
        self.assertEqual(self.node.review_store.summary()["state"], "consumed")

    def test_environment_policy_change_after_approval_cannot_dispatch_old_plan(self):
        _, _, approval = self.make_review()
        self.send(approval, "approve")
        self.node.environment_policy_path.write_text("changed policy; restart required")
        self.node.process_next()
        value = json.loads((self.node.directory / (approval["request_id"] + ".json")).read_text())
        self.assertEqual(value["status"], "failed")
        self.assertIn("Environment clearance policy changed", value["error"]["message"])
        self.node.dispatch.assert_not_called()

    def test_environment_policy_change_at_final_state_gate_is_still_refused(self):
        _, _, approval = self.make_review()
        self.send(approval, "approve")
        original = self.node.confirm_start_unchanged
        count = [0]
        def change_at_last_gate(initial):
            original(initial)
            count[0] += 1
            if count[0] == 2:
                self.node.environment_policy_path.write_text("changed just before dispatch")
        self.node.confirm_start_unchanged = change_at_last_gate
        self.node.process_next()
        value = json.loads((self.node.directory / (approval["request_id"] + ".json")).read_text())
        self.assertEqual(count[0], 2)
        self.assertEqual(value["status"], "failed")
        self.assertIn("Environment clearance policy changed", value["error"]["message"])
        self.node.dispatch.assert_not_called()

    def test_action_completion_without_observed_joint_states_is_not_success(self):
        _, _, approval = self.make_review()
        self.send(approval, "approve")
        def incomplete_dispatch(commands, **kwargs):
            self.node.dispatched += len(commands)
            self.node.observations = []
        self.node.dispatch.side_effect = incomplete_dispatch
        self.node.process_next()
        value = json.loads((self.node.directory / (approval["request_id"] + ".json")).read_text())
        self.assertEqual(value["status"], "failed")
        self.assertEqual(value["failure_kind"], "execution_failed")
        self.assertIn("observed joint-state coverage", value["error"]["message"])
        self.assertTrue(self.node.blocked)

    def test_approval_rechecks_same_curves_without_replanning_and_consumes_once(self):
        _,_,approval = self.make_review()
        self.node.coordinated_path.reset_mock()
        self.node.validity_of.reset_mock()
        self.send(approval,"approve")
        self.assertIsNotNone(self.node.pending)
        self.node.process_next()
        result = self.node.replies[approval["request_id"]]
        self.assertEqual(result["status"],"succeeded",result.get("error"))
        self.assertEqual(result["mode"],"approved_plan")
        self.assertEqual(result["approval"],approval)
        self.assertEqual(result["plan_request_id"],approval["plan_request_id"])
        self.assertEqual(result["execution_request_id"],approval["request_id"])
        self.node.coordinated_path.assert_not_called()
        self.assertGreater(self.node.validity_of.call_count,2)
        self.node.dispatch.assert_called_once()
        self.send(approval,"approve")  # The same execution ID only replays its result.
        self.node.process_next()
        self.send(dict(approval,request_id=uuid.uuid4().hex),"approve")
        self.node.process_next()
        self.node.dispatch.assert_called_once()

    def test_approval_after_live_scene_change_fails_before_dispatch(self):
        _,_,approval = self.make_review()
        self.send(approval,"approve")
        self.node._review_runtime_identity.return_value = dict(self.identity,planning_scene_sha256="c"*64)
        self.node.process_next()
        self.assertEqual(self.node.replies[approval["request_id"]]["status"],"failed")
        self.node.dispatch.assert_not_called()
        self.assertEqual(self.node.review_store.summary()["state"],"consumed")

    def test_rejected_malformed_or_foreign_approval_does_not_expose_execution_link(self):
        _,_,approval=self.make_review()
        for mutation in (dict(extra=True),dict(plan_sha256='wrong'),dict(session_id='f'*32)):
            payload=dict(approval,request_id=uuid.uuid4().hex,**mutation)
            self.send(payload,'approve')
            result=self.node.replies[payload['request_id']]
            self.assertEqual(result['status'],'rejected')
            self.assertNotIn('approval',result)
            self.assertNotIn('plan_request_id',result)
            self.assertNotIn('execution_request_id',result)
        self.node.dispatch.assert_not_called()

    def test_approved_path_still_fails_current_moveit_safety_check(self):
        _,_,approval = self.make_review()
        self.send(approval,"approve")
        # Both endpoints are valid; a middle sample is now invalid.
        self.node.validity_of.side_effect = [SimpleNamespace(valid=True),SimpleNamespace(valid=True),
                                             SimpleNamespace(valid=False)]
        self.node.process_next()
        self.assertEqual(self.node.replies[approval["request_id"]]["status"],"failed")
        self.node.dispatch.assert_not_called()

    def test_state_change_between_approval_and_execution_refuses_dispatch(self):
        _,_,approval = self.make_review()
        self.send(approval,"approve")
        self.node.actual["left"] = .002
        self.node.process_next()
        self.node.dispatch.assert_not_called()
        self.assertEqual(self.node.replies[approval["request_id"]]["failure_kind"],"review_invalidated")

    def test_busy_approval_cannot_steal_pending_owner(self):
        _,_,approval = self.make_review()
        other = RequestTrace(uuid.uuid4().hex)
        self.node.pending = other
        self.send(approval,"approve")
        self.assertIs(self.node.pending,other)
        self.assertEqual(self.node.replies[approval["request_id"]]["status"],"rejected")
        self.assertEqual(self.node.review_store.summary()["state"],"awaiting_approval")

    def test_cancel_revokes_an_unexecuted_review_and_cannot_approve_it(self):
        payload,_,approval = self.make_review()
        self.send(dict(request_id=payload["request_id"]),"cancel")
        self.assertEqual(self.node.review_store.summary()["state"],"invalidated")
        revoked = self.node.publish.call_args.args[0]
        self.assertEqual(revoked["stage"], "plan_revoked")
        self.assertEqual(revoked["operation"]["source_stage"], "plan_revoked")
        self.assertEqual(revoked["operation"]["phase"], "review")
        self.assertIsNone(revoked["operation"]["physical_guard"])
        self.send(approval,"approve")
        self.assertIsNone(self.node.pending)
        self.node.dispatch.assert_not_called()

    def test_repeated_plan_reply_reports_expiry_without_replanning(self):
        self.node.review_store.clock = lambda:100.0
        payload,_,_ = self.make_review()
        self.node.review_store.clock = lambda:160.0
        self.send(payload)
        self.assertIsNone(self.node.pending)
        self.assertEqual(self.node.publish.call_args.args[0]["review"]["state"],"expired")
        self.node.coordinated_path.assert_called_once()
        self.node.dispatch.assert_not_called()

    def test_regular_v1_command_revokes_review_and_keeps_automatic_execution(self):
        _,_,approval = self.make_review()
        payload = command(dict(left=.1,right=.1))
        self.send(payload,"command")
        self.assertEqual(self.node.review_store.summary()["state"],"invalidated")
        self.node.process_next()
        self.assertEqual(self.node.replies[payload["request_id"]]["status"],"succeeded")
        self.node.dispatch.assert_called_once()

    def test_no_motion_plan_keeps_zero_dispatch_without_an_approval_token(self):
        payload = command(dict(left=0.0,right=0.0))
        self.send(payload)
        self.node.process_next()
        result = self.node.replies[payload["request_id"]]
        self.assertEqual(result["status"],"succeeded")
        self.assertEqual(result["stage"],"plan_not_required")
        self.assertIsNone(self.node.review_store.summary())
        self.assertNotIn("review",result)
        record = json.loads((self.node.directory/(payload["request_id"]+".json")).read_text())
        self.assertEqual(record["events"][-1]["name"],"terminal")
        self.node.dispatch.assert_not_called()

    def test_unknown_submission_blocks_even_when_no_send_future_was_returned(self):
        payload = command(dict(left=.1, right=.1))
        self.send(payload, "command")
        def unknown(*args, **kwargs):
            self.node.execution_ledger = ExecutionLedger(attempts=[ExecutionAttempt(
                "left", submission_started=True, submission_uncertain=True)],
                cleanup_requested=True, cleanup_uncertain=True)
            # A future with an exception must not be read again by finally.
            future = Mock()
            future.done.return_value = True
            future.result.side_effect = RuntimeError("must not erase primary")
            self.node.handles = [(Mock(), future)]
            raise RuntimeError("original send failure")
        self.node.dispatch.side_effect = unknown
        self.node.process_next()
        reply = self.node.replies[payload["request_id"]]
        self.assertEqual(reply["status"], "failed")
        self.assertEqual(reply["failure_kind"], "execution_failed")
        self.assertTrue(self.node.blocked)
        record = json.loads((self.node.directory/(payload["request_id"]+".json")).read_text())
        self.assertEqual(record["error"]["message"], "original send failure")
        self.assertEqual(record["dispatched_goals"], 0)
        self.assertEqual(record["action_results"], [])
        self.assertTrue(record["execution_transport"]["cleanup_uncertain"])
        self.node.handles[0][1].result.assert_not_called()
        later = command(dict(left=.2, right=.2))
        self.send(later, "command")
        self.assertIsNone(self.node.pending)
        self.assertEqual(self.node.replies[later["request_id"]]["status"], "rejected")
        self.node.dispatch.assert_called_once()

    def test_plan_only_clears_old_execution_ledger_without_reusing_old_diagnostics(self):
        self.node.execution_ledger = ExecutionLedger(cleanup_uncertain=True)
        self.node.execution_transport = {"previous_request": True}
        payload, _, _ = self.make_review()
        self.assertIsNone(self.node.execution_ledger)
        self.assertIsNone(self.node.execution_transport)
        record = json.loads((self.node.directory/(payload["request_id"]+".json")).read_text())
        self.assertNotIn("execution_transport", record)
        self.assertEqual(record["action_results"], [])
        self.assertFalse(self.node.blocked)


if __name__ == "__main__":
    unittest.main()
