"""Legacy setup/saved schedules get the production native self admission gate."""
import ast
from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path
from mujoco_ros2_manipulation.workcells.self_clearance import require_self_clearance,SelfClearanceRejected
from tests.unit.test_self_path_gate import evaluator,POLICY,own

def require(ok,message):
    if not ok:raise RuntimeError(message)

class Base:
    def dispatch(self,commands,**kwargs):
        self.execution_ledger = NS(attempts=[NS(submission_started=False)])
        self.goal_observer()
        self.dispatch_abort()
        self.execution_ledger.attempts[0].submission_started = True
        self.sent.append((commands,kwargs));self.dispatched+=len(commands)
        self.execution_transport={'attempts':[{'terminal_status':5}]}
        if self.action_failure:raise RuntimeError('native contact stop; arm failed')
        return 'actual transport return'

def load_dispatch():
    path=Path(__file__).resolve().parents[2]/'src/mujoco_ros2_manipulation/workcells/verify_interference.py'
    tree=ast.parse(path.read_text());cls=next(v for v in tree.body if isinstance(v,ast.ClassDef) and v.name=='InterferenceVerification')
    cls.body=[v for v in cls.body if isinstance(v,ast.FunctionDef) and v.name=='dispatch'];cls.bases=[ast.Name(id='Base',ctx=ast.Load())]
    namespace=dict(Base=Base,require=require,Spline=Spline,file_sha256=file_sha256,evaluate_path=evaluate_path,
        require_self_clearance=require_self_clearance,self_evaluator_identity=Mock(return_value={"pinned":"native"}),SelfClearanceRejected=SelfClearanceRejected,deepcopy=deepcopy)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),str(path),'exec'),namespace)
    return namespace['InterferenceVerification'],namespace

class LegacySelfDispatchTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        cls,self.namespace=load_dispatch();self.node=n=cls()
        n.self_policy=deepcopy(POLICY);n.self_policy_path=Path(self.folder.name)/'policy.yaml'
        n.self_policy_path.write_text('checked policy');n.self_policy_sha256=file_sha256(n.self_policy_path)
        n.commanded=dict(left=0.,right=0.);n.cell=NS(root=Path(self.folder.name),joints=list(n.commanded),instances=[NS(id=x,joints=[x]) for x in n.commanded])
        n.dispatched=0;n.evidence={};n.sent=[];n.action_failure=False;n.goal_observer=Mock();n.clock_last=49.;n.contact_guard=dict(latched=False)
        n.ready_guard=Mock();n.state=lambda:dict(n.commanded);n.save_evidence=Mock()
        self.ev=evaluator(.02);self.ev.metadata=lambda:{'self_collision':{'pair_count':24}}
        self.ev.__enter__=lambda:self.ev
        # Python special methods live on the type, not on a SimpleNamespace.
        class Context:
            def __enter__(_):return self.ev
            def __exit__(_, *args):pass
        self.namespace['DistanceEvaluator']=Mock(return_value=Context())

    def commands(self,partial=False):
        names=['left'] if partial else ['left','right']
        tr=NS(joint_names=names,points=[NS(positions=[t*.1]*len(names),velocities=[],accelerations=[],
            time_from_start=NS(sec=t,nanosec=0)) for t in (0,1)])
        return [('original arm',tr)]

    def test_old_intermediate_self_overlap_sends_no_goals_and_preserves_rejection(self):
        original=self.ev.at.side_effect
        def at(q):
            r=original(q)
            if .03<q['left']<.07:r['self_collision']['distance_m']=-.002
            return r
        self.ev.at.side_effect=at
        with self.assertRaises(SelfClearanceRejected):self.node.dispatch(self.commands(),scheduled_start=50.)
        self.assertEqual(self.node.sent,[])
        record=self.node.evidence['dispatch_admission'][0]
        self.assertEqual(record['status'],'rejected');self.assertFalse(record['native_self_clearance']['complete'])
        self.assertEqual(record['scheduled_start_sec'],50.)

    def test_saved_single_arm_path_keeps_peer_held_without_sending_a_peer_goal(self):
        commands=self.commands(partial=True)
        self.assertEqual(self.node.dispatch(commands,scheduled_start=50.),'actual transport return')
        self.assertEqual(self.node.sent[0][0],commands);self.assertEqual(len(self.node.sent[0][0]),1)
        self.assertTrue(all(call.args[0]['right']==0. for call in self.ev.at.call_args_list))

    def test_cancellation_terminal_failure_keeps_original_reservation_and_transport(self):
        self.node.action_failure=True
        with self.assertRaisesRegex(RuntimeError,'native contact stop'):self.node.dispatch(self.commands(),scheduled_start=50.)
        record=self.node.evidence['dispatch_admission'][0]
        self.assertEqual(record['scheduled_start_sec'],50.)
        self.assertEqual(record['execution_transport']['attempts'][0]['terminal_status'],5)
        self.assertEqual(record['dispatched_goals_after'],1)

    def test_slow_admission_cannot_send_goals_after_the_reserved_start(self):
        self.node.save_evidence.side_effect=lambda *_:setattr(self.node,'clock_last',49.95)
        with self.assertRaisesRegex(RuntimeError,'reservation'):self.node.dispatch(self.commands(),scheduled_start=50.)
        self.assertEqual(self.node.sent,[])

    def test_late_goal_observer_cannot_change_policy_clock_or_geometry_then_send(self):
        def changed():
            self.node.self_policy_path.write_text('changed by goal observer')
        self.node.goal_observer.side_effect=changed
        with self.assertRaisesRegex(RuntimeError,'policy changed'):self.node.dispatch(self.commands(),scheduled_start=50.)
        self.assertEqual(self.node.sent,[])
        self.node.self_policy_path.write_text('checked policy')
        self.node.goal_observer.side_effect=lambda:setattr(self.node,'clock_last',49.95)
        with self.assertRaisesRegex(RuntimeError,'reservation'):self.node.dispatch(self.commands(),scheduled_start=50.)
        self.assertEqual(self.node.sent,[])

    def test_terminal_save_error_preserves_primary_but_successful_transport_still_fails_missing_record(self):
        def disk_error(stage):
            if stage == 'dispatch_terminal':raise OSError('terminal disk unavailable')
        self.node.save_evidence.side_effect=disk_error
        self.node.action_failure=True
        with self.assertRaisesRegex(RuntimeError,'native contact stop'):
            self.node.dispatch(self.commands(),scheduled_start=50.)
        record=self.node.evidence['dispatch_admission'][-1]
        self.assertEqual(record['error']['type'],'RuntimeError')
        self.assertEqual(record['persistence_error']['type'],'OSError')
        self.assertEqual(record['execution_transport']['attempts'][0]['terminal_status'],5)
        self.node.action_failure=False
        with self.assertRaisesRegex(OSError,'terminal disk unavailable'):
            self.node.dispatch(self.commands(),scheduled_start=50.)

    def test_policy_or_observed_geometry_change_during_checkpoint_sends_zero_goals(self):
        self.node.save_evidence.side_effect=lambda *_:self.node.self_policy_path.write_text('changed')
        with self.assertRaisesRegex(RuntimeError,'policy changed'):self.node.dispatch(self.commands(),scheduled_start=50.)
        self.assertEqual(self.node.sent,[])
        self.node.self_policy_path.write_text('checked policy')
        self.node.save_evidence.side_effect=lambda *_:setattr(self.ev.at,'side_effect',lambda q:dict(
            distance_m=.2,tips_m=[[0,0,0],[1,0,0]],self_collision=own(.0005)))
        with self.assertRaises(SelfClearanceRejected):self.node.dispatch(self.commands(),scheduled_start=50.)
        self.assertEqual(self.node.sent,[])
