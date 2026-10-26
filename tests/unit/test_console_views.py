"""Owner-scaffold tests: fake app/state/host, no ROS or model compilation."""
import ast
from copy import deepcopy
import hashlib
from importlib.util import find_spec
import json
import math
import re
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console.scene_host import SceneSource
from mujoco_ros2_manipulation.operator_console.view_binding import make_binding, binding_id
from mujoco_ros2_manipulation.operator_console.view_plan import ReviewPlan, SavedPlan
from tests.unit.test_view_plan import production_review_fixture


def console_class(**injected):
    # Compile the installed owner's actual methods with pure/fake collaborators;
    # this isolates orchestration tests from runtime workers and model loading.
    source = Path(find_spec('mujoco_ros2_manipulation.operator_console.console_views').origin)
    tree = ast.parse(source.read_text())
    tree.body = [node for node in tree.body if not isinstance(node, (ast.Import, ast.ImportFrom))]
    namespace = dict(deepcopy=deepcopy,hashlib=hashlib,json=json,math=math,Path=Path,
        time=SimpleNamespace(monotonic=lambda:10.),SceneSource=SceneSource,make_binding=make_binding,
        file_sha256=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        check_workcell=lambda cell:dict(workcell_sha256='f'*64),
        check_generated_model=lambda profile:None,
        load_profile=lambda *args,**kwargs:(_ for _ in ()).throw(ValueError('unavailable')),
        matching_process=lambda launch:True,SceneHost=None,SavedPlan=None,ReviewPlan=ReviewPlan,
        request_id=lambda value:None if isinstance(value,str) and re.fullmatch('[0-9a-f]{32}',value)
            else (_ for _ in ()).throw(ValueError('bad request')))
    namespace.update(injected)
    exec(compile(tree,str(source),'exec'),namespace)
    return namespace['ConsoleViews']


class FakeHost:
    def __init__(self,*args,**kwargs):
        self.binding = None; self.source = None; self.submissions = []; self.invalidated = []; self.activated = []
    def invalidate(self,binding=None):
        self.binding=deepcopy(binding);self.source=None;self.submissions=[];self.invalidated.append(deepcopy(binding))
    def activate(self,binding,source):
        self.binding=deepcopy(binding);self.source=source;self.submissions=[];self.activated.append(deepcopy(binding))
    def submit(self,positions,**kwargs):
        self.submissions.append((deepcopy(positions),deepcopy(kwargs)))
    def snapshot(self):
        return dict(binding_id=None if self.binding is None else binding_id(self.binding),binding=deepcopy(self.binding))
    def close(self):
        pass


class ConsoleViewsTests(unittest.TestCase):
    def setUp(self):
        temporary=TemporaryDirectory();self.addCleanup(temporary.cleanup)
        root=Path(temporary.name)
        self.cell=SimpleNamespace(id='fr5_ur5e',label='pair',root=root,output=root/'base',joints=['q'],home=[0.])
        state=SimpleNamespace(lock=threading.RLock(),active=None,positions={'q':.1},joint_at=11.,clock_at=11.,
            clock_advanced_at=11.,observed_model_path=str(self.cell.output/'cell.xml'),identity_requested_at=11.,
            identity_checked_at=11.,graph={'clock':1,'joints':1},server={'session_id':'b'*32},
            joint_state_stamp_simulation_sec=3.,admission_suspended=True,history={},
            config={'joint_limits':{'q':{'lower':-3.,'upper':3.}}})
        self.demos=SimpleNamespace(busy=False,current=None,validate_launch=lambda launch:None)
        self.app=SimpleNamespace(cell=self.cell,state=state,demos=self.demos,phase='idle',generation=1,
            session_id='a'*32,directory=root/'console',lock=threading.RLock(),closing=threading.Event(),audit=lambda *a,**kw:None)
        self.app.display_mode='web3d';self.app.image_render_enabled=False
        self.owner=console_class()(self.app,host_factory=FakeHost)

    def launch(self,run='run-1',variant='contact'):
        return dict(job_id='c'*32,stage='motion',variant=variant,run_id=run,
            active_launch=dict(run_id=run,model_path=str(self.cell.output/'cell.xml'),model_sha256='d'*64))

    def test_waiting_job_and_variant_changes_blank_binding_before_new_model_or_q(self):
        self.owner.tick()
        original=self.owner.host.snapshot()['binding_id']
        self.assertEqual(len(self.owner.host.submissions),1)
        self.demos.busy=True
        self.demos.current=dict(job_id='c'*32,stage='preparing',variant='contact',run_id=None,active_launch=None)
        self.owner.tick()
        waiting=self.owner.host.snapshot()['binding_id']
        self.assertNotEqual(waiting,original)
        self.assertIsNone(self.owner.source)
        self.assertEqual(self.owner.host.submissions,[])
        self.owner.tick()
        self.assertEqual(self.owner.host.snapshot()['binding_id'],waiting)
        self.demos.current.update(stage='preparing-control',variant='control')
        self.owner.tick()
        self.assertNotEqual(self.owner.host.snapshot()['binding_id'],waiting)
        self.assertEqual(self.owner.host.submissions,[])

    def test_new_model_needs_new_joint_and_newly_started_identity_query_not_old_completion(self):
        self.demos.busy=True;self.demos.current=self.launch()
        self.app.state.identity_requested_at=9.
        self.app.state.identity_checked_at=12.
        self.owner.tick()
        self.assertEqual(len(self.owner.host.activated),1)
        self.assertEqual(self.owner.host.submissions,[])
        self.app.state.identity_requested_at=10.5
        self.owner.tick()
        self.assertEqual(len(self.owner.host.submissions),1)
        self.demos.current=self.launch(run='run-2',variant='control')
        self.app.state.joint_at=9.
        self.owner.tick()
        self.assertEqual(len(self.owner.host.activated),2)
        self.assertEqual(self.owner.host.submissions,[])

    def test_invalid_or_no_longer_owned_launch_blanks_previous_pose(self):
        self.owner.tick();self.assertTrue(self.owner.host.submissions)
        self.demos.busy=True;self.demos.current=self.launch()
        def reject(launch): raise ValueError('invalid launch')
        self.demos.validate_launch=reject
        with self.assertRaisesRegex(ValueError,'invalid launch'):
            self.owner.tick()
        self.assertEqual(self.owner.host.submissions,[])
        self.assertIsNone(self.owner.source)
        self.demos.validate_launch=lambda launch:None
        self.owner.__class__.tick.__globals__['matching_process']=lambda launch:False
        self.owner.tick()
        count=len(self.owner.host.invalidated)
        self.owner.tick()
        self.assertEqual(len(self.owner.host.invalidated),count,'A stable waiting epoch does not churn bindings')

    def test_same_known_demo_stopped_launch_keeps_last_pose_without_new_samples(self):
        clock=[10.]
        self.owner.__class__._tick.__globals__['time'].monotonic=lambda:clock[0]
        self.demos.busy=True;self.demos.current=self.launch()
        self.owner.tick()
        before=self.owner.host.snapshot();samples=deepcopy(self.owner.host.submissions)
        source=self.owner.source;key=self.owner.key;invalidations=len(self.owner.host.invalidated)
        self.assertEqual(len(samples),1)
        self.owner.__class__._tick.__globals__['matching_process']=lambda launch:False
        self.demos.current['stage']='cleanup'
        clock[0]=12.
        state=self.app.state
        state.positions={'q':.9};state.joint_state_stamp_simulation_sec=4.
        state.joint_at=state.clock_at=state.clock_advanced_at=12.
        state.identity_requested_at=state.identity_checked_at=12.
        state.clock_backwards=False
        for _ in range(3):self.owner.tick()
        self.assertEqual(self.owner.host.snapshot(),before)
        self.assertEqual(self.owner.host.submissions,samples,'Stopped launch cannot refresh the last receiver time')
        self.assertEqual(self.owner.key,key);self.assertIs(self.owner.source,source)
        self.assertEqual(len(self.owner.host.invalidated),invalidations)
        self.assertEqual(len(self.owner.host.activated),1)
        self.assertFalse(state.clock_backwards)
        self.assertTrue(state.admission_suspended)

    def test_same_known_demo_missing_launch_keeps_last_pose_through_terminal_cleanup(self):
        self.demos.busy=True;self.demos.current=self.launch()
        self.owner.tick()
        before=self.owner.host.snapshot();samples=deepcopy(self.owner.host.submissions)
        source=self.owner.source;key=self.owner.key
        self.demos.current.update(stage='cleanup',active_launch=None)
        self.owner.tick();self.owner.tick()
        self.demos.current.update(status='succeeded',stage='complete')
        self.demos.busy=False;self.app.phase='idle'
        self.owner.tick()
        self.assertEqual(self.owner.host.snapshot(),before)
        self.assertEqual(self.owner.host.submissions,samples)
        self.assertEqual(self.owner.key,key);self.assertIs(self.owner.source,source)
        self.assertEqual(len(self.owner.host.activated),1)
        self.assertTrue(self.app.state.admission_suspended)

    def test_last_demo_pose_cannot_be_claimed_by_changed_job_variant_run_or_model(self):
        changes=[
            lambda job:job.update(job_id='e'*32,active_launch=None),
            lambda job:job.update(variant='control',active_launch=None),
            lambda job:job.update(run_id=None,stage='preflight',active_launch=None),
            lambda job:job.update(run_id='run-2',active_launch=None),
            lambda job:job['active_launch'].update(model_sha256='e'*64),
            lambda job:job['active_launch'].update(model_path=str(self.cell.output/'other.xml')),
        ]
        for busy in (True,False):
            for index,change in enumerate(changes):
                with self.subTest(change=index,busy=busy):
                    self.owner=console_class()(self.app,host_factory=FakeHost)
                    self.demos.busy=True;self.demos.current=self.launch()
                    self.owner.tick();before=self.owner.host.snapshot()['binding_id']
                    self.assertTrue(self.owner.host.submissions)
                    self.owner.__class__._tick.__globals__['matching_process']=lambda launch:False
                    change(self.demos.current)
                    self.demos.busy=busy;self.app.phase='demo' if busy else 'idle'
                    self.owner.tick()
                    self.assertNotEqual(self.owner.host.snapshot()['binding_id'],before)
                    self.assertIsNone(self.owner.source)
                    self.assertEqual(self.owner.host.submissions,[])
        # Even a different terminal job without a busy worker cannot reuse it.
        self.owner=console_class()(self.app,host_factory=FakeHost)
        self.demos.busy=True;self.demos.current=self.launch();self.owner.tick()
        before=self.owner.host.snapshot()['binding_id']
        self.demos.current.update(job_id='e'*32,active_launch=None)
        self.demos.busy=False;self.app.phase='idle'
        self.owner.tick()
        self.assertNotEqual(self.owner.host.snapshot()['binding_id'],before)
        self.assertIsNone(self.owner.source)

    def test_profile_metadata_none_return_is_replaced_by_exact_frozen_source_bytes(self):
        output=self.cell.root/'profile';output.mkdir()
        path=output/'source.json';path.write_text('{"robot_profile_sha256":"original"}')
        profile=SimpleNamespace(output=output,label='FR5',scene_path=output/'scene.xml',joints=['q'],home=[0.])
        def load(identifier,**kwargs):
            if identifier!='fr5':raise ValueError('unavailable')
            return profile
        owner=console_class(load_profile=load)(self.app,host_factory=FakeHost)
        source=owner.registry['profile:fr5']
        self.assertEqual(source.validate(),dict(robot_profile_sha256='original'))
        path.write_text('{"robot_profile_sha256":"new"}')
        with self.assertRaisesRegex(ValueError,'model changed'):
            source.validate()

    def test_bad_new_motion_session_cannot_leave_old_binding_current(self):
        self.owner.tick();self.assertTrue(self.owner.host.submissions)
        self.app.state.server={'session_id':'untrusted-invalid-session'}
        with self.assertRaises(ValueError):self.owner.tick()
        self.assertIsNone(self.owner.host.snapshot()['binding_id'])
        self.assertIsNone(self.owner.source)
        self.assertEqual(self.owner.host.submissions,[])

    def test_seek_requires_current_binding_and_never_restores_admission(self):
        self.owner.mode='plan_preview'
        self.owner.plan=SimpleNamespace(duration=2.,positions=lambda stamp:{'q':stamp})
        self.owner._activate(('plan','first'),self.owner.base,request_id='d'*32,motion_session_id='b'*32)
        previous=self.owner.host.snapshot()['binding_id']
        self.owner._activate(('plan','second'),self.owner.base,request_id='e'*32,motion_session_id='b'*32)
        with self.assertRaisesRegex(ValueError,'미리보기가 바뀌었습니다'):
            self.owner.seek(1.,binding_id=previous)
        self.assertEqual(self.owner.host.submissions,[])
        current=self.owner.host.snapshot()['binding_id']
        self.assertEqual(self.owner.seek(1.,binding_id=current)['preview_time_sec'],1.)
        self.assertEqual(len(self.owner.host.submissions),1)
        self.assertTrue(self.app.state.admission_suspended)

    def test_disabled_mode_needs_no_models_full_cell_or_worker(self):
        def unavailable(*args,**kwargs):raise AssertionError('Disabled views must not load model assets or start a host')
        cls=console_class(check_workcell=unavailable,load_profile=unavailable)
        app=SimpleNamespace(cell=SimpleNamespace(root=self.cell.root),lock=threading.RLock())
        owner=cls(app,enabled=False,host_factory=unavailable)
        self.assertIsNone(owner.host);self.assertIsNone(owner.base);self.assertIsNone(owner.metadata)
        self.assertEqual(owner.registry,{})
        self.assertEqual(owner.snapshot()['catalog'],[])
        self.assertEqual(owner.snapshot()['status'],'disabled')
        owner.tick();owner.close()

    def test_same_model_clock_reset_blocks_old_launch_until_new_run_and_fresh_query(self):
        clock=[10.]
        self.owner.__class__._tick.__globals__['time'].monotonic=lambda:clock[0]
        self.demos.busy=True;self.demos.current=self.launch()
        state=self.app.state
        state.clock_backwards=False
        self.owner.tick()
        old=self.owner.host.snapshot()['binding_id']
        self.assertEqual(self.owner.last_accepted_stamp,3.)
        clock[0]=12.
        state.joint_at=state.clock_at=state.clock_advanced_at=12.
        state.joint_state_stamp_simulation_sec=0.
        self.owner.tick()
        self.assertEqual(self.owner.run_reset_blocked,('c'*32,'run-1'))
        blank=self.owner.host.snapshot()['binding_id']
        self.assertNotEqual(blank,old)
        self.assertEqual(self.owner.host.submissions,[])
        # Even newer q and a newly completed query cannot reuse the old run ID.
        state.identity_requested_at=state.identity_checked_at=12.1
        state.joint_state_stamp_simulation_sec=.1
        self.owner.tick()
        self.assertEqual(self.owner.host.snapshot()['binding_id'],blank)
        self.assertEqual(self.owner.host.submissions,[])
        clock[0]=13.
        self.demos.current=self.launch(run='run-2',variant='control')
        self.owner.tick()
        self.assertIsNone(self.owner.run_reset_blocked)
        self.assertEqual(self.owner.host.submissions,[])
        # New-run activation has its own epoch; an earlier request is insufficient.
        clock[0]=13.3
        state.joint_at=state.clock_at=state.clock_advanced_at=13.2
        state.identity_requested_at=12.5;state.identity_checked_at=13.2
        state.joint_state_stamp_simulation_sec=.2
        self.owner.tick();self.assertEqual(self.owner.host.submissions,[])
        state.identity_requested_at=13.1
        self.owner.tick()
        self.assertEqual(len(self.owner.host.submissions),1)
        self.assertEqual(self.owner.last_accepted_stamp,.2)
        self.assertFalse(state.clock_backwards)
        self.assertTrue(state.admission_suspended)

    def test_pending_identity_and_missing_stamp_wait_without_error(self):
        self.app.state.identity_requested_at=self.app.state.identity_checked_at=None
        self.owner.tick()
        self.assertEqual(self.owner.host.submissions,[])
        self.app.state.identity_requested_at=self.app.state.identity_checked_at=11.
        self.app.state.joint_state_stamp_simulation_sec=None
        self.owner.tick()
        self.assertEqual(self.owner.host.submissions,[])

    def test_manual_generation_change_is_required_to_release_manual_clock_reset(self):
        clock=[10.]
        self.owner.__class__._tick.__globals__['time'].monotonic=lambda:clock[0]
        self.owner.tick()
        self.app.state.joint_state_stamp_simulation_sec=0.
        self.owner.tick()
        self.assertEqual(self.owner.run_reset_blocked,(None,None))
        self.owner.tick();self.assertEqual(self.owner.host.submissions,[])
        self.app.generation+=1;clock[0]=12.
        self.owner.tick()
        self.assertIsNone(self.owner.run_reset_blocked)
        self.assertEqual(self.owner.host.submissions,[])
        clock[0]=12.3
        state=self.app.state
        state.joint_at=state.clock_at=state.clock_advanced_at=12.2
        state.identity_requested_at=12.1;state.identity_checked_at=12.2
        state.joint_state_stamp_simulation_sec=.1
        self.owner.tick()
        self.assertEqual(len(self.owner.host.submissions),1)
        self.assertTrue(state.admission_suspended)

    def unlocked(self):
        self.assertFalse(self.app.lock._is_owned(),'Model/trace validation cannot hold the app lock')
        self.assertFalse(self.app.state.lock._is_owned(),'Model/trace validation cannot hold the state lock')

    def test_model_validation_is_outside_locks_and_generation_race_preserves_old_selection(self):
        self.owner.tick();before=self.owner.host.snapshot();original_mode=self.owner.mode
        original=self.owner.base
        def validate():
            self.unlocked();self.app.generation+=1
        source=SceneSource(original.id,original.label,original.xml_path,original.joint_names,original.home,validate)
        self.owner.registry[source.id]=source
        with self.assertRaisesRegex(RuntimeError,'검증 중 실행'):
            self.owner.select({'mode':'model_preview','source_id':source.id})
        self.assertEqual(self.owner.mode,original_mode)
        self.assertEqual(self.owner.host.snapshot(),before)
        self.assertTrue(self.app.state.admission_suspended)

    def test_plan_read_validation_is_outside_locks_then_history_is_rechecked(self):
        identifier='e'*32
        directory=self.cell.root/'runtime'/'saved';directory.mkdir(parents=True)
        (directory/(identifier+'.json')).write_bytes(b'trace original bytes')
        (directory/'session.json').write_bytes(b'session original bytes')
        record={'status':'succeeded','trace_path':str(directory/(identifier+'.json')),'session_id':'b'*32}
        self.app.state.history[identifier]=record
        original=self.owner.base
        def validate():self.unlocked()
        self.owner.base=SceneSource(original.id,original.label,original.xml_path,original.joint_names,original.home,validate)
        self.owner.tick();before=self.owner.host.snapshot()
        original_open=Path.open
        def checked_open(path,*args,**kwargs):
            self.unlocked()
            return original_open(path,*args,**kwargs)
        def saved_plan(trace,session,**kwargs):
            self.unlocked()
            self.assertEqual(trace,b'trace original bytes');self.assertEqual(session,b'session original bytes')
            self.app.state.history[identifier]=dict(record,session_id='c'*32)
            return SimpleNamespace(duration=1.,positions=lambda stamp:{'q':stamp})
        self.owner.__class__.select_plan.__globals__['SavedPlan']=saved_plan
        with patch.object(Path,'open',checked_open),self.assertRaisesRegex(RuntimeError,'요청이나 모델 출처'):
            self.owner.select_plan(identifier)
        self.assertEqual(self.owner.mode,'live')
        self.assertEqual(self.owner.host.snapshot(),before)
        self.assertTrue(self.app.state.admission_suspended)

    def test_image_mode_rejects_previews_and_pauses_worker_once_until_new_web3d_epoch(self):
        self.owner.tick();old=self.owner.host.snapshot()['binding_id']
        self.app.display_mode='image';self.app.image_render_enabled=True
        with self.assertRaisesRegex(RuntimeError,'로컬 3D'):
            self.owner.select({'mode':'model_preview','source_id':self.owner.base.id})
        self.owner.tick()
        self.assertIsNone(self.owner.host.snapshot()['binding_id'])
        count=len(self.owner.host.invalidated)
        self.owner.tick();self.assertEqual(len(self.owner.host.invalidated),count)
        self.assertIsNone(self.owner.source)
        self.app.display_mode='web3d';self.app.image_render_enabled=False
        self.owner.tick()
        self.assertNotEqual(self.owner.host.snapshot()['binding_id'],old)
        self.assertTrue(self.app.state.admission_suspended)

    def pending_review(self):
        trace,session,options,review=production_review_fixture()
        identifier=options['request_id']
        directory=self.cell.root/'runtime'/'review';directory.mkdir(parents=True)
        path=directory/(identifier+'.json');path.write_text(json.dumps(trace))
        (directory/'session.json').write_text(json.dumps(session))
        self.cell.joints=list(options['joint_names']);self.cell.home=[0.]
        state=self.app.state
        state.positions=deepcopy(trace['initial_state_rad'])
        state.config['joint_limits']=deepcopy(options['joint_limits'])
        state.server={'session_id':review['session_id']}
        self.pending=deepcopy(review)
        state.review_snapshot=lambda:deepcopy(self.pending)
        state.history[identifier]=dict(request_id=identifier,status='succeeded',mode='plan_only',
            trace_path=str(path),session_id=review['session_id'])
        self.owner=console_class(check_workcell=lambda cell:deepcopy(options['model_manifest']),
                                 SavedPlan=SavedPlan)(self.app,host_factory=FakeHost)
        self.owner.tick()
        return identifier,path,trace

    def test_pending_review_selection_and_approval_use_separate_display_epochs(self):
        identifier,_,_=self.pending_review()
        live=self.owner.host.snapshot()['binding_id'];history=deepcopy(self.app.state.history)
        preview=self.owner.select_review_plan(identifier)
        self.assertTrue(preview['review_plan']);self.assertTrue(preview['preview_active'])
        self.assertEqual(preview['mode'],'plan_preview')
        self.assertNotEqual(preview['binding_id'],live)
        self.assertEqual(preview['binding']['request_id'],identifier)
        self.assertEqual(self.owner.host.submissions[-1],({'joint':0.},{'preview_time_sec':0.}))
        self.owner.seek(.5,binding_id=preview['binding_id'])
        self.assertEqual(self.owner.host.submissions[-1],({'joint':.5},{'preview_time_sec':.5}))
        self.owner.approve_review(identifier)
        result=self.owner.snapshot()
        self.assertEqual(result['mode'],'live');self.assertFalse(result['review_plan'])
        self.assertFalse(result['preview_active']);self.assertIsNone(result['binding']['request_id'])
        self.assertNotEqual(result['binding_id'],preview['binding_id'])
        self.assertEqual(self.app.state.history,history,'A display switch cannot reserve or execute a command')
        self.assertTrue(self.app.state.admission_suspended)

    def test_changed_or_expired_pending_review_cannot_select_or_approve_old_path(self):
        identifier,_,_=self.pending_review()
        live=self.owner.host.snapshot()
        self.pending['remaining_sec']=0
        with self.assertRaisesRegex(ValueError,'승인 대기'):self.owner.select_review_plan(identifier)
        self.assertEqual(self.owner.host.snapshot(),live)
        self.pending['remaining_sec']=60
        self.owner.select_review_plan(identifier)
        preview=self.owner.host.snapshot()
        for change in ({'state':'expired'},{'session_id':'e'*32},{'plan_request_id':'e'*32}):
            saved=deepcopy(self.pending);self.pending.update(change)
            with self.subTest(change=change),self.assertRaisesRegex(RuntimeError,'다시 검토'):
                self.owner.approve_review(identifier)
            self.assertEqual(self.owner.host.snapshot(),preview)
            self.assertEqual(self.owner.mode,'plan_preview')
            self.pending=saved

    def test_modified_pending_trace_is_rejected_without_replacing_live_binding(self):
        identifier,path,trace=self.pending_review()
        live=self.owner.host.snapshot()
        trace['trajectories'][0]['points'][1]['p']=[1.5]
        trace['planned_trajectories']=deepcopy(trace['trajectories'])
        path.write_text(json.dumps(trace))
        with self.assertRaisesRegex(ValueError,'exact pending plan digest'):
            self.owner.select_review_plan(identifier)
        self.assertEqual(self.owner.host.snapshot(),live)
        self.assertFalse(self.owner.review_plan)

    def test_only_review_selection_can_approve_and_old_seek_binding_cannot_replay_after_switch(self):
        identifier,_,_=self.pending_review()
        with self.assertRaisesRegex(RuntimeError,'다시 검토'):self.owner.approve_review(identifier)
        with self.assertRaisesRegex(ValueError,'pending-plan preview'):self.owner.select_plan(identifier)
        preview=self.owner.select_review_plan(identifier)
        self.owner.select({'mode':'live','source_id':self.owner.base.id})
        with self.assertRaises(RuntimeError):self.owner.seek(.5,binding_id=preview['binding_id'])
        with self.assertRaisesRegex(RuntimeError,'다시 검토'):self.owner.approve_review(identifier)


if __name__=='__main__':unittest.main()
