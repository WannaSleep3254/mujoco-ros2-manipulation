"""ROS-free transport and owned-worker boundaries of the real acceptance tool."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console import verify_views
from mujoco_ros2_manipulation.operator_console.view_binding import envelope, make_binding


def fixture(mode='live'):
    binding=make_binding('a'*32,1,'workcell:manual',mode,
        motion_session_id='b'*32 if mode!='model_preview' else None,
        request_id='c'*32 if mode=='plan_preview' else None)
    pose=dict(schema_version=1,model_sha256='d'*64,scene_sha256='e'*64,
        geoms=[dict(id=0,position=[1.,2.,3.],rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.])])
    value=envelope(binding,pose,1,source_state_age_sec=.05 if mode=='live' else None,
        simulation_sec=1. if mode=='live' else None,preview_time_sec=.5 if mode=='plan_preview' else None)
    manifest=dict(model_sha256='d'*64,scene_sha256='e'*64,geoms=[dict(id=0)])
    return binding,value,manifest


class VerifyViewsTests(unittest.TestCase):
    def test_exact_compiled_identity_binding_and_whole_catalog(self):
        binding,value,manifest=fixture()
        self.assertEqual(verify_views.checked_pose(value,manifest,binding,live=True),value)
        for change in (lambda v:v.update(binding_id='f'*64),lambda v:v['pose'].update(model_sha256='f'*64),
            lambda v:v['pose']['geoms'].clear(),lambda v:v.update(extra='unknown')):
            candidate=deepcopy(value);change(candidate)
            with self.assertRaises((RuntimeError,ValueError)):
                verify_views.checked_pose(candidate,manifest,binding,live=True)

    def test_live_age_and_simulation_stamp_are_required(self):
        binding,value,manifest=fixture()
        for change in (lambda v:v.update(source_state_age_sec=.5),lambda v:v.update(source_state_age_sec=None),
                       lambda v:v.update(simulation_sec=None)):
            candidate=deepcopy(value);change(candidate)
            with self.assertRaises((RuntimeError,ValueError)):
                verify_views.checked_pose(candidate,manifest,binding,live=True)

    def test_plan_preview_uses_null_live_age_and_the_requested_playback_time(self):
        binding,value,manifest=fixture('plan_preview')
        self.assertEqual(verify_views.checked_pose(value,manifest,binding,playback=.5),value)
        with self.assertRaises(RuntimeError):verify_views.checked_pose(value,manifest,binding,playback=1.)
        value['source_state_age_sec']=0.
        with self.assertRaises(ValueError):verify_views.checked_pose(value,manifest,binding,playback=.5)

    def test_bounded_runtime_file_rejects_links_and_oversized_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'record.json';source.write_bytes(b'{}')
            self.assertEqual(verify_views.bounded_file(source,2),b'{}')
            with self.assertRaises(RuntimeError):verify_views.bounded_file(source,1)
            alias=root/'alias.json';alias.symlink_to(source)
            with self.assertRaises(RuntimeError):verify_views.bounded_file(alias,2)

    def worker_fixture(self,root,identity):
        proc=root/'proc';session=root/'session';owned=session/'scene-view/worker-test';owned.mkdir(parents=True)
        children=proc/'99/task/99/children';children.parent.mkdir(parents=True);children.write_text('')
        # SceneHost launches from its supervisor thread; /task/<main>/children
        # alone does not list children created by another Linux thread.
        children=proc/'99/task/123/children';children.parent.mkdir(parents=True);children.write_text('100 200')
        for pid,argv in ((100,['python','-m',verify_views.WORKER_MODULE,'--directory',str(owned)]),
                         (200,['unrelated-user-process'])):
            path=proc/str(pid)/'cmdline';path.parent.mkdir(parents=True);path.write_bytes(b'\0'.join(item.encode() for item in argv)+b'\0')
        return proc,session,owned

    def test_only_direct_owned_worker_and_confined_directory_are_recorded(self):
        identity=dict(pid=100,start_ticks=123,ppid=99,pgid=100,sid=100)
        with tempfile.TemporaryDirectory() as directory:
            proc,session,owned=self.worker_fixture(Path(directory),identity)
            with patch.object(verify_views,'process_identity',return_value=identity) as inspect:
                values=verify_views.scene_workers(99,session,proc=proc)
            self.assertEqual(values,[dict(identity,directory=str(owned))])
            inspect.assert_called_once_with(100)

    def test_external_parent_group_or_session_is_never_accepted_as_owned(self):
        identity=dict(pid=100,start_ticks=123,ppid=99,pgid=100,sid=100)
        with tempfile.TemporaryDirectory() as directory:
            proc,session,_=self.worker_fixture(Path(directory),identity)
            for field in ('ppid','pgid','sid'):
                with patch.object(verify_views,'process_identity',return_value=dict(identity,**{field:1})),self.assertRaises(RuntimeError):
                    verify_views.scene_workers(99,session,proc=proc)

class DemoCPUViewTests(unittest.TestCase):
    def setUp(self):
        self.run='20261023T120000.123456Z-1234abcd'
        self.job='f'*32
        self.binding=make_binding('a'*32,2,'workcell:manual','live',job_id=self.job,run_id=self.run)
        self.manifest=dict(model_sha256='d'*64,scene_sha256='e'*64,geoms=[dict(id=0)])
        pose=dict(schema_version=1,model_sha256='d'*64,scene_sha256='e'*64,
            geoms=[dict(id=0,position=[1.,2.,3.],rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.])])
        self.value=envelope(self.binding,pose,3,source_state_age_sec=.05,simulation_sec=2.)
        self.view=dict(binding_id=self.value['binding_id'],binding=self.binding,status='ready',error=None,
            display_mode='web3d',pose_ready=True,manifest_descriptor=dict(model_sha256='d'*64))
        self.context=dict(operator_session_id='a'*32,source_id='workcell:manual',job_id=self.job,
            run_id=self.run,model_sha256='d'*64)

    def test_each_demo_pose_requires_the_exact_current_owned_run_binding_and_model(self):
        self.assertEqual(verify_views.checked_demo_binding(self.view,**self.context),self.binding)
        for change in (dict(job_id='b'*32),dict(run_id='older-run'),dict(operator_session_id='c'*32),
                       dict(source_id='profile:fr10'),dict(model_sha256='c'*64),dict(model_sha256=None)):
            with self.subTest(change=change),self.assertRaises((RuntimeError,ValueError)):
                verify_views.checked_demo_binding(self.view,**dict(self.context,**change))
        previous=deepcopy(self.view)
        previous['binding']=make_binding('a'*32,1,'workcell:manual','live',job_id=self.job,run_id='previous-run')
        previous['binding_id']=verify_views.binding_id(previous['binding'])
        with self.assertRaises(RuntimeError):verify_views.checked_demo_binding(previous,**self.context)

    def test_transition_may_be_blank_but_cannot_keep_old_manifest_or_pose(self):
        waiting=make_binding('a'*32,3,'workcell:manual','live')
        view=dict(self.view,binding=waiting,binding_id=verify_views.binding_id(waiting),status='waiting',
            manifest_descriptor=None,pose_ready=False)
        self.assertIsNone(verify_views.checked_demo_binding(view,**dict(self.context,run_id=None,model_sha256=None)))
        for change in (dict(pose_ready=True),dict(manifest_descriptor=dict(model_sha256='d'*64)),
                       dict(status='error'),dict(error=dict(code='owner_failed')),dict(display_mode='image')):
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                verify_views.checked_demo_binding(dict(view,**change),**self.context)
        null=dict(view,binding=None,binding_id=None)
        self.assertIsNone(verify_views.checked_demo_binding(null,**self.context))
        with self.assertRaises(RuntimeError):
            verify_views.checked_demo_binding(dict(null,pose_ready=True),**self.context)

    def test_only_explicit_terminal_reads_allow_stale_live_poses(self):
        value=deepcopy(self.value);value['source_state_age_sec']=2.
        with self.assertRaises(RuntimeError):verify_views.checked_pose(value,self.manifest,self.binding,live=True)
        self.assertEqual(verify_views.checked_pose(value,self.manifest,self.binding,live=True,allow_stale=True),value)
        for change in (dict(source_state_age_sec=None),dict(source_state_age_sec=float('inf')),dict(simulation_sec=None)):
            with self.subTest(change=change),self.assertRaises((RuntimeError,ValueError)):
                verify_views.checked_pose(dict(value,**change),self.manifest,self.binding,live=True,allow_stale=True)

    def test_live_sequence_can_advance_but_same_sequence_cannot_change_pose_or_time(self):
        next_value=deepcopy(self.value);next_value.update(sequence=4,simulation_sec=2.1)
        self.assertEqual(verify_views.checked_pose_progress(self.value,next_value),next_value)
        same=deepcopy(self.value);same['source_state_age_sec']=.1
        self.assertEqual(verify_views.checked_pose_progress(self.value,same),same)
        changes=[dict(sequence=2),dict(simulation_sec=1.9),dict(binding_id='c'*64),dict(source_state_age_sec=.01)]
        for change in changes:
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                verify_views.checked_pose_progress(self.value,dict(self.value,**change))
        for key in ('pose','simulation_sec'):
            changed=deepcopy(self.value)
            if key=='pose':changed['pose']['geoms'][0]['position'][0]=4.
            else:changed['simulation_sec']=2.01
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                verify_views.checked_pose_progress(self.value,changed)

    def test_terminal_retains_last_run_pose_and_original_age_instead_of_clearing_or_refreshing(self):
        current=deepcopy(self.value);current['source_state_age_sec']=.81
        self.assertEqual(verify_views.checked_retained_pose(self.value,current),current)
        for change in (dict(sequence=4),dict(binding_id='c'*64),dict(simulation_sec=3.),
                       dict(source_state_age_sec=.05),dict(source_state_age_sec=.49)):
            with self.subTest(change=change),self.assertRaises(RuntimeError):
                verify_views.checked_retained_pose(self.value,dict(current,**change))
        changed=deepcopy(current);changed['pose']['geoms'].clear()
        with self.assertRaises(RuntimeError):verify_views.checked_retained_pose(self.value,changed)

    def test_final_view_must_serve_a_current_pose_not_an_in_memory_previous_sample(self):
        self.assertEqual(verify_views.checked_final_read(self.view,self.value,self.value['binding_id']),self.value)
        for view,pose in ((self.view,None),(dict(self.view,pose_ready=False),self.value),
                          (dict(self.view,status='waiting'),self.value),
                          (dict(self.view,binding_id='c'*64),self.value)):
            with self.subTest(view=view,pose=pose),self.assertRaises(RuntimeError):
                verify_views.checked_final_read(view,pose,self.value['binding_id'])

    def launch_fixture(self,root):
        run=root/'runtime/validation/runs'/self.run;run.mkdir(parents=True)
        contact=run/'contact_test.xml';contact.write_bytes(b'<mujoco model="contact"/>')
        base=root/'runtime/generated/workcells/fr5_ur5e/cell.xml';base.parent.mkdir(parents=True);base.write_bytes(b'<mujoco model="base"/>')
        launch=dict(run_id=self.run,report_path=str(run/'result.json'),model_path=str(contact),
            model_sha256=verify_views.digest(contact.read_bytes()),pid=999,start_ticks=123)
        return launch,contact,base

    def test_run_specific_contact_model_and_base_timing_model_are_separate_exact_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);launch,contact,base=self.launch_fixture(root)
            for variant in ('contact','control'):
                self.assertEqual(verify_views.checked_demo_launch(launch,variant,root=root,base_model=base),contact)
            timing=dict(launch,model_path=str(base),model_sha256=verify_views.digest(base.read_bytes()))
            for variant in ('prepare','baseline','timing'):
                self.assertEqual(verify_views.checked_demo_launch(timing,variant,root=root,base_model=base),base)
            with self.assertRaises(RuntimeError):verify_views.checked_demo_launch(launch,'baseline',root=root,base_model=base)
            with self.assertRaises(RuntimeError):verify_views.checked_demo_launch(timing,'contact',root=root,base_model=base)

    def test_foreign_paths_changed_bytes_and_unstable_launch_identity_are_never_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);launch,contact,base=self.launch_fixture(root)
            for change in (dict(run_id='../other'),dict(report_path=str(root/'other.json')),
                           dict(model_path=str(base)),dict(model_sha256='0'*64),dict(pid=True),dict(start_ticks=0)):
                with self.subTest(change=change),self.assertRaises(RuntimeError):
                    verify_views.checked_demo_launch(dict(launch,**change),'contact',root=root,base_model=base)
            alias=contact.with_name('alias.xml');alias.symlink_to(contact)
            with self.assertRaises(RuntimeError):
                verify_views.checked_demo_launch(dict(launch,model_path=str(alias)),'contact',root=root,base_model=base)
            contact.write_bytes(b'<mujoco model="changed"/>')
            with self.assertRaises(RuntimeError):verify_views.checked_demo_launch(launch,'control',root=root,base_model=base)


if __name__=='__main__':unittest.main()
