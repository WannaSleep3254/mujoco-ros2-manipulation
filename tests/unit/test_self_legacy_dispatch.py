"""Legacy setup/saved schedules get the production native self admission gate."""
import ast
from copy import deepcopy
from contextlib import contextmanager
import json
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
    cls.body=[v for v in cls.body if isinstance(v,ast.FunctionDef) and v.name in ('dispatch','_self_dispatch_fresh','_self_command_key','_self_file_stats','_self_model_binding','_self_boundary','_self_proof_key','_confirm_prepared','prepare_dispatch','reserve_prepared','stamp_prepared','dispatch_prepared')];cls.bases=[ast.Name(id='Base',ctx=ast.Load())]
    namespace=dict(Base=Base,require=require,Spline=Spline,contextmanager=contextmanager,json=json,Path=Path,Time=NS,file_sha256=file_sha256,evaluate_path=evaluate_path,
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
        self.wall_ns = 9_000_000_000
        self.namespace['time'] = NS(monotonic_ns=lambda:self.wall_ns)
        n._self_clock_received_ns = n._self_state_received_ns = self.wall_ns
        n.clock_backwards = False
        n.ready_guard=Mock();n.state=lambda:dict(n.commanded);n.save_evidence=Mock();n.settings={'tracking_tolerance_rad':.01}
        n._self_model_binding=Mock(return_value=dict(xml_sha256='model',manifest_sha256='manifest',stats={'bound_file':'stable'}))
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

    def relative_commands(self,partial=False):
        commands=self.commands(partial)
        for _,trajectory in commands:trajectory.header=NS(stamp=NS(sec=0,nanosec=0),frame_id='world')
        return commands

    def fresh(self,commands):
        with self.node.prepare_dispatch(commands,fresh_relative=True) as prepared:
            start=self.node.reserve_prepared(prepared)
            stamped=self.node.stamp_prepared(commands,prepared)
            self.node.dispatch_prepared(stamped,prepared,scheduled_start=start)
            return start,deepcopy(prepared['record'])

    def test_slow_geometry_and_checkpoint_complete_before_original_one_second_reservation(self):
        evaluate=self.namespace['evaluate_path']
        def slow(*args,**kwargs):
            result=evaluate(*args,**kwargs)
            self.wall_ns+=1_500_000_000
            return result
        self.namespace['evaluate_path']=slow
        def checkpoint(stage):
            if stage=='native_self_dispatch_gate':self.wall_ns+=750_000_000
        self.node.save_evidence.side_effect=checkpoint
        def fresh_receipts():
            self.node.clock_last=51.25
            self.node._self_clock_received_ns=self.node._self_state_received_ns=self.wall_ns
        self.node.ready_guard.side_effect=fresh_receipts
        commands=self.relative_commands();original=deepcopy(commands)
        start,record=self.fresh(commands)
        self.assertEqual(start,52.25);self.assertEqual(record['reserved_from_clock_sec'],51.25)
        self.assertEqual(record['reservation_lead_sec'],1.0);self.assertEqual(record['reservation_mint_count'],1)
        self.assertEqual(record['stage_elapsed_sec']['native_full_scan'],1.5)
        self.assertEqual(record['stage_elapsed_sec']['prepare_checkpoint'],.75)
        self.assertEqual(record['dispatched_goals_before'],0);self.assertEqual(len(self.node.sent),1)
        self.assertEqual(self.node.sent[0][0][0][1].points,original[0][1].points)
        self.assertEqual(commands,original)

    def test_fractional_start_uses_one_canonical_header_and_keeps_partial_delayed_knots(self):
        self.node.clock_last=15.29
        commands=self.relative_commands(partial=True)
        commands[0][1].points.insert(1,NS(positions=[0.],velocities=[],accelerations=[],time_from_start=NS(sec=0,nanosec=250_000_000)))
        original=deepcopy(commands[0][1].points)
        start,record=self.fresh(commands)
        self.assertEqual(start,16.29)
        trajectory=self.node.sent[0][0][0][1]
        self.assertEqual((trajectory.header.stamp.sec,trajectory.header.stamp.nanosec),(16,290_000_000))
        self.assertEqual(trajectory.points,original);self.assertEqual(len(self.node.sent[0][0]),1)
        self.assertEqual(record['reservation_mint_count'],1)

    def test_two_arm_relative_delay_derivatives_and_order_stay_exact_across_stamping(self):
        left=self.relative_commands(partial=True)[0][1]
        right=deepcopy(left);right.joint_names=['right']
        for point in right.points:point.velocities=[0.];point.accelerations=[0.]
        right.points.insert(1,NS(positions=[0.],velocities=[0.],accelerations=[0.],time_from_start=NS(sec=0,nanosec=400_000_000)))
        right.points[-1].velocities=[.01];right.points[-1].accelerations=[-.02]
        commands=list(zip(self.node.cell.instances,[left,right]));original=deepcopy(commands)
        with self.node.prepare_dispatch(commands,fresh_relative=True) as prepared:
            start=self.node.reserve_prepared(prepared);stamped=self.node.stamp_prepared(commands,prepared)
            self.assertEqual([t.points for _,t in stamped],[t.points for _,t in original])
            self.assertEqual({(t.header.stamp.sec,t.header.stamp.nanosec) for _,t in stamped},{(50,0)})
            stamped.reverse()
            with self.assertRaisesRegex(RuntimeError,'arm order'):
                self.node.dispatch_prepared(stamped,prepared,scheduled_start=start)
        self.assertEqual(self.node.sent,[]);self.assertEqual(commands,original)

    def test_checkpoint_mutation_cannot_seal_a_different_relative_curve_or_initial(self):
        for field in ('positions','velocities','accelerations','relative_time'):
            with self.subTest(field=field):
                commands=self.relative_commands()
                def changed(stage):
                    if stage!='native_self_dispatch_gate':return
                    point=commands[0][1].points[-1]
                    if field=='relative_time':point.time_from_start.nanosec=1
                    else:setattr(point,field,[.2,.2])
                self.node.save_evidence.side_effect=changed
                with self.assertRaisesRegex(RuntimeError,'relative trajectory changed'):
                    self.fresh(commands)
                self.assertEqual(self.node.sent,[])
        self.node.save_evidence.side_effect=lambda stage:self.node.commanded.update(left=.1) if stage=='native_self_dispatch_gate' else None
        with self.assertRaisesRegex(RuntimeError,'Commanded start changed'):self.fresh(self.relative_commands())
        self.assertEqual(self.node.sent,[])

    def test_mutable_evidence_cannot_extend_reservation_or_allow_a_second_submission(self):
        commands=self.relative_commands()
        with self.assertRaisesRegex(RuntimeError,'reservation differs'):
            with self.node.prepare_dispatch(commands,fresh_relative=True) as prepared:
                start=self.node.reserve_prepared(prepared)
                prepared['record']['scheduled_start_sec']=start+10
                stamped=self.node.stamp_prepared(commands,prepared)
                self.node.dispatch_prepared(stamped,prepared,scheduled_start=start+10)
        self.assertEqual(self.node.sent,[])
        with self.assertRaisesRegex(RuntimeError,'already consumed'):
            with self.node.prepare_dispatch(commands,fresh_relative=True) as prepared:
                start=self.node.reserve_prepared(prepared);stamped=self.node.stamp_prepared(commands,prepared)
                self.node.dispatch_prepared(stamped,prepared,scheduled_start=start)
                self.node.dispatch_prepared(stamped,prepared,scheduled_start=start)
        self.assertEqual(len(self.node.sent),1)

    def test_fresh_cancel_guard_model_or_late_final_reservation_failure_sends_zero(self):
        for boundary in ('cancel','guard','model','reservation'):
            with self.subTest(boundary=boundary):
                self.node.contact_guard={'latched':False};self.node.dispatch_abort=None
                self.node._self_model_binding.side_effect=None
                cancelled=[False]
                if boundary=='cancel':
                    self.node.dispatch_abort=lambda:require(not cancelled[0],'cancelled')
                def final():
                    if boundary=='cancel':cancelled[0]=True
                    if boundary=='guard':self.node.contact_guard['latched']=True
                    if boundary=='model':self.node._self_model_binding.side_effect=RuntimeError('model changed')
                    if boundary=='reservation':
                        self.node.clock_last+=.95
                        self.node._self_clock_received_ns=self.node._self_state_received_ns=self.wall_ns
                self.node.goal_observer.side_effect=final
                with self.assertRaises(RuntimeError):self.fresh(self.relative_commands())
                self.assertEqual(self.node.sent,[])

    def test_stale_preparation_or_expired_saved_schedule_is_not_rebased_and_failure_is_recorded(self):
        self.node.save_evidence.side_effect=lambda stage:setattr(self,'wall_ns',self.wall_ns+2_000_000_000) if stage=='native_self_dispatch_gate' else None
        with self.assertRaisesRegex(RuntimeError,'stale'):self.fresh(self.relative_commands())
        record=self.node.evidence['dispatch_admission'][-1]
        self.assertEqual(record['reservation_mint_count'],0);self.assertEqual(record['status'],'failed')
        self.assertIn('stale',record['error']['message']);self.assertIn('observed_start',record)
        self.assertTrue(record['boundaries']);self.assertEqual(record['dispatched_goals_after'],0)

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

    def test_slow_final_identity_or_geometry_cannot_send_with_held_clock_receipts(self):
        ordinary = self.ev.at.side_effect
        for stage in ('identity','geometry'):
            with self.subTest(stage=stage):
                self.node._self_clock_received_ns = self.node._self_state_received_ns = self.wall_ns
                calls = [0]
                def identity(*_):
                    calls[0] += 1
                    if stage == 'identity' and calls[0] == 2:
                        self.wall_ns += 2_000_000_000
                    return {'pinned':'native'}
                self.namespace['self_evaluator_identity'].side_effect = identity
                def slow_geometry(q):
                    self.wall_ns += 2_000_000_000
                    return ordinary(q)
                self.node.goal_observer.side_effect = (lambda:setattr(self.ev.at,'side_effect',slow_geometry)) if stage == 'geometry' else None
                self.ev.at.side_effect = ordinary
                with self.assertRaisesRegex(RuntimeError,'stale'):
                    self.node.dispatch(self.commands(),scheduled_start=50.)
                self.assertEqual(self.node.sent,[])

    def test_fresh_aged_clock_does_not_extend_the_original_reservation(self):
        self.node.goal_observer.side_effect=lambda:setattr(self,'wall_ns',self.wall_ns+200_000_000)
        with self.assertRaisesRegex(RuntimeError,'reservation'):
            self.node.dispatch(self.commands(),scheduled_start=49.25)
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
