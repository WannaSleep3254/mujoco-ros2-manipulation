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


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def load_server():
    source = Path(__file__).resolve().parents[2]/"src/mujoco_ros2_manipulation/workcells/motion_server.py"
    tree = ast.parse(source.read_text(),filename=str(source))
    body = [v for v in tree.body if isinstance(v,ast.ClassDef)]
    namespace = dict(WorkcellVerification=object,InterferenceVerification=SimpleNamespace(split=lambda *a:None),
        deepcopy=deepcopy,Path=Path,json=json,time=time,uuid=uuid,RequestTrace=RequestTrace,
        PlanReviewStore=PlanReviewStore,ReviewRejected=ReviewRejected,request_id=request_id,
        decode_message=decode_message,validate_command=validate_command,validate_cancel=validate_cancel,
        validate_approval=validate_approval,require=require,Spline=Spline,PlanningBudget=PlanningBudget,
        PlanningBudgetExceeded=PlanningBudgetExceeded,PlanCandidatesExhausted=type("Candidates",(RuntimeError,),{}),
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
        n.evaluator = SimpleNamespace(at=Mock(return_value=dict(distance_m=.1,tips_m=[[0,0,0],[1,0,0]])))
        self.curve = Spline(["left","right"],[
            dict(t=0.0,p=[0.0,0.0],v=[],a=[]),dict(t=.1,p=[.1,.1],v=[],a=[])])
        n.coordinated_path = Mock(return_value=self.curve)
        n.split = Mock(return_value=[("left","left curve"),("right","right curve")])
        def dispatch(commands,**kwargs):
            n.dispatched += len(commands)
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


if __name__ == "__main__":
    unittest.main()
