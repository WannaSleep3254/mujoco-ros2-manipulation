"""Owned CPU-preflight protocol and pure preview registration; no CAD/ROS engine."""
from copy import deepcopy
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console import environment_sources as sources
from mujoco_ros2_manipulation.workcells import environment_check_worker as worker


def manifest(identifier='installed_fixture'):
    return dict(schema_version=1, artifact_kind='environment-overlay-v1', id=identifier,
        fingerprint='a'*64, base_workcell_id='pair', joint_names=['left_q','right_q'], home_rad=[0.,.2],
        inputs=dict(base_workcell_sha256='b'*64, base_manifest_sha256='c'*64),
        outputs={'cell.xml':'d'*64}, dependencies={'private-design':{'sha256':'e'*64,'bytes':12}},
        geometry_check=dict(engine='MuJoCo 3.14.0',physics_steps=0,home_contacts=0,
            visual_world_bbox_error_m=0.,collision_center_error_m=0.,urdf_mjcf_corner_error_m=0.))


class FakeProcess:
    def __init__(self, factory, argv, kwargs):
        self.factory,self.argv,self.kwargs=factory,argv,kwargs
        self.returncode=None;self.terminated=self.killed=0;self.waits=[]
        def argument(name):return argv[argv.index(name)+1]
        self.receipt=Path(argument('--receipt'));self.directory=self.receipt.parent
        kwargs['stdout'].write(factory.log_bytes)
        report=deepcopy(factory.reports[Path(argument('--overlay'))])
        value=sources._receipt(report, token=argument('--request-token'),
            manifest_sha256=argument('--manifest-sha256'),worker_sha256=argument('--worker-sha256'))
        if factory.mutate:factory.mutate(value)
        if factory.publish:
            self.receipt.write_bytes(factory.raw if factory.raw is not None else json.dumps(value).encode())
    def poll(self):return self.returncode
    def terminate(self):self.terminated+=1
    def kill(self):self.killed+=1
    def wait(self,timeout):
        self.waits.append(timeout)
        if self.factory.timeout and not self.terminated or self.factory.stubborn and not self.killed or self.factory.unreapable:
            raise subprocess.TimeoutExpired('owned-checker',timeout)
        self.returncode=-9 if self.killed else -15 if self.terminated else self.factory.code
        return self.returncode


class Factory:
    def __init__(self,reports):
        self.reports=reports;self.processes=[];self.code=0
        self.mutate=None;self.raw=None;self.publish=True
        self.timeout=self.stubborn=self.unreapable=False
        self.log_bytes=b'CPU checker diagnostic\n'
    def __call__(self,argv,**kwargs):
        process=FakeProcess(self,argv,kwargs);self.processes.append(process);return process


class EnvironmentSourceTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name);self.overlay=self.root/'overlay';self.overlay.mkdir()
        (self.overlay/'cell.xml').write_text('<mujoco/>')
        self.python=self.root/'.venv/bin/python';self.python.parent.mkdir(parents=True);self.python.touch()
        self.base=NS(root=self.root,id='pair',joints=['left_q','right_q'],home=[0.,.2])
        self.path=self.root/'catalog.json';self.digest='f'*64
        self.write([dict(directory='overlay',manifest_sha256=self.digest)])
        self.reports={self.overlay:manifest()};self.factory=Factory(self.reports)
        self.identity=patch.object(sources,'_identity',side_effect=lambda directory,*args:deepcopy(self.reports[directory])).start()
        self.addCleanup(patch.stopall)
        patch.object(sources,'subprocess',NS(Popen=self.factory,DEVNULL=subprocess.DEVNULL,
            STDOUT=subprocess.STDOUT,TimeoutExpired=subprocess.TimeoutExpired)).start()
        patch.object(sources,'_worker_source_sha256',return_value='1'*64).start()
    def write(self,entries,**extra):
        self.path.write_text(json.dumps(dict(schema_version=1,overlays=entries,**extra)))
    def load(self,**kwargs):return sources.load_scene_catalog(self.path,self.base,**kwargs)

    def test_sources_are_complete_detached_private_and_use_only_owned_preflight(self):
        values=self.load();self.assertIsInstance(values,tuple);self.assertEqual(len(values),1)
        source=values[0]
        self.assertEqual(source.id,'scene:installed_fixture');self.assertEqual(source.xml_path,self.overlay/'cell.xml')
        self.assertEqual(source.joint_names,tuple(self.base.joints));self.assertEqual(source.home,dict(left_q=0.,right_q=.2))
        self.assertEqual(source.label,'설계 환경 · installed_fixture')
        token=source.validate();token['fingerprint']='mutated'
        self.assertEqual(source.validate()['fingerprint'],'a'*64)
        self.assertNotIn(str(self.root),json.dumps(source.validate()))
        self.assertNotIn('private-design',json.dumps(source.validate()))
        process=self.factory.processes[0]
        self.assertTrue(process.kwargs['start_new_session']);self.assertEqual(process.kwargs['cwd'],self.root)
        self.assertEqual(process.kwargs['stdin'],subprocess.DEVNULL)
        self.assertEqual(process.argv[:3],[str(self.python),'-m',sources.WORKER_MODULE])
        self.assertEqual(process.argv[process.argv.index('--parent-pid')+1],str(os.getpid()))
        self.assertEqual(process.terminated,0);self.assertEqual(process.killed,0)
        self.assertTrue(process.directory.exists(),'Owned receipt/log evidence is retained')

    def test_success_keeps_separate_private_receipts_and_logs_inside_workspace(self):
        self.load();self.load()
        first,second=self.factory.processes
        expected=self.root/'runtime/validation/environment-checks'
        self.assertEqual(first.directory.parent,expected)
        self.assertEqual(second.directory.parent,expected)
        self.assertNotEqual(first.directory,second.directory)
        for process in (first,second):
            token=process.argv[process.argv.index('--request-token')+1]
            self.assertEqual(process.directory.name,token)
            self.assertEqual(process.directory.stat().st_mode & 0o777,0o700)
            log=process.directory/'checker.log'
            self.assertEqual(log.stat().st_mode & 0o777,0o600)
            self.assertEqual(log.read_bytes(),self.factory.log_bytes)
            value=json.loads(process.receipt.read_bytes())
            self.assertEqual(value['request_token'],token)
            self.assertEqual(value['report_sha256'],sources._sha(sources._canonical(self.reports[self.overlay])))

    def test_failed_checker_keeps_original_diagnostics_and_rejected_receipt(self):
        self.factory.code=1
        self.factory.log_bytes=b'CPU checker failed: diagnostic retained\n'
        self.factory.mutate=lambda value:value.update(request_token='0'*32)
        with self.assertRaisesRegex(ValueError,'CPU validation failed'):self.load()
        process=self.factory.processes[0]
        self.assertEqual(process.directory.parent,self.root/'runtime/validation/environment-checks')
        self.assertEqual((process.directory/'checker.log').read_bytes(),self.factory.log_bytes)
        self.assertEqual(json.loads(process.receipt.read_bytes())['request_token'],'0'*32)
        self.assertEqual(process.returncode,1)
        self.assertEqual(process.terminated,0);self.assertEqual(process.killed,0)

    def test_output_directory_links_never_escape_workspace_or_spawn_checker(self):
        outside=self.root/'outside';outside.mkdir()
        for index,linked_name in enumerate(('runtime','validation','environment-checks')):
            with self.subTest(component=linked_name):
                project=self.root/('project'+str(index));project.mkdir()
                current=project
                for name in ('runtime','validation','environment-checks'):
                    current=current/name
                    if name==linked_name:
                        current.symlink_to(outside,target_is_directory=True);break
                    current.mkdir()
                base=NS(root=project,id='pair')
                with self.assertRaisesRegex(ValueError,'unlinked workspace'):
                    sources._full_check(self.overlay,base,self.digest,manifest(),self.python,1.)
        root_alias=self.root/'workspace-link';root_alias.symlink_to(self.root,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'unlinked workspace'):
            sources._full_check(self.overlay,NS(root=root_alias,id='pair'),self.digest,manifest(),self.python,1.)
        self.assertEqual(self.factory.processes,[])
        self.assertEqual(list(outside.iterdir()),[])

    def test_existing_check_directory_is_never_overwritten_even_on_token_collision(self):
        with patch.object(sources.secrets,'token_hex',return_value='a'*32):
            self.load();process=self.factory.processes[0]
            before={p.name:p.read_bytes() for p in process.directory.iterdir()}
            with self.assertRaises(FileExistsError):self.load()
            self.assertEqual({p.name:p.read_bytes() for p in process.directory.iterdir()},before)
        self.assertEqual(len(self.factory.processes),1)

    def test_catalog_schema_is_exact_bounded_and_all_entries_checked_before_spawn(self):
        valid=dict(directory='overlay',manifest_sha256=self.digest)
        malformed=[dict(schema_version=True,overlays=[valid]),dict(schema_version=1,overlays=[]),
            dict(schema_version=1,overlays=[valid]*9),dict(schema_version=1,overlays=[dict(valid,source_id='scene:override')]),
            dict(schema_version=1,overlays=[dict(valid,manifest_sha256='bad')]),
            dict(schema_version=1,overlays=[valid,valid]),dict(schema_version=1,overlays=[valid],unexpected=True)]
        for value in malformed:
            with self.subTest(value=value):
                self.path.write_text(json.dumps(value))
                with self.assertRaises(ValueError):self.load()
        for raw in (b'{"schema_version":1,"schema_version":1,"overlays":[]}',b'{"x":NaN}',
                    b' '* (sources.MAX_CATALOG_BYTES+1)):
            self.path.write_bytes(raw)
            with self.assertRaises(ValueError):self.load()
        self.assertEqual(self.factory.processes,[])
        self.identity.assert_not_called()

    def test_catalog_and_directory_links_and_fifo_never_become_registered_inputs(self):
        alias=self.root/'catalog-link.json';alias.symlink_to(self.path)
        with self.assertRaises((OSError,ValueError)):sources.load_scene_catalog(alias,self.base)
        linked=self.root/'linked';linked.symlink_to(self.overlay,target_is_directory=True)
        self.write([dict(directory='linked',manifest_sha256=self.digest)])
        with self.assertRaises(ValueError):self.load()
        fifo=self.root/'fifo';os.mkfifo(fifo)
        with self.assertRaises(ValueError):sources.load_scene_catalog(fifo,self.base)
        self.assertEqual(self.factory.processes,[])

    def test_duplicate_declared_ids_return_no_partial_catalog(self):
        second=self.root/'second';second.mkdir();self.reports[second]=manifest()
        self.write([dict(directory='overlay',manifest_sha256=self.digest),dict(directory='second',manifest_sha256='e'*64)])
        with self.assertRaisesRegex(ValueError,'Duplicate declared'):self.load()
        self.assertEqual(len(self.factory.processes),1)
        self.assertTrue(self.factory.processes[0].directory.exists())

    def test_receipt_cannot_substitute_nonce_model_geometry_hash_or_private_extra_fields(self):
        mutations=[lambda value:value.update(request_token='0'*32),lambda value:value.update(overlay_manifest_sha256='0'*64),
            lambda value:value.update(report_sha256='0'*64),lambda value:value['geometry_check'].update(home_contacts=1),
            lambda value:value.update(worker_sha256='0'*64),lambda value:value.update(schema_version=True),
            lambda value:value.update(private_path=str(self.root))]
        for mutate in mutations:
            with self.subTest(mutation=mutations.index(mutate)):
                self.factory.mutate=mutate
                with self.assertRaisesRegex(ValueError,'receipt'):self.load()
                self.assertTrue(self.factory.processes[-1].directory.exists())

    def test_missing_oversized_or_failed_worker_receipt_is_not_a_success(self):
        self.factory.publish=False
        with self.assertRaises(OSError):self.load()
        self.factory.publish=True;self.factory.raw=b' '* (sources.MAX_RECEIPT_BYTES+1)
        with self.assertRaises(ValueError):self.load()
        self.factory.raw=None;self.factory.code=1
        with self.assertRaisesRegex(ValueError,'CPU validation'):self.load()
        for process in self.factory.processes:self.assertTrue(process.directory.exists())

    def test_timeout_reaps_only_its_owned_child_with_term_then_kill(self):
        self.factory.timeout=self.factory.stubborn=True
        with self.assertRaises(subprocess.TimeoutExpired):self.load(timeout_sec=.5)
        process=self.factory.processes[0]
        self.assertEqual(process.waits,[.5,2,2]);self.assertEqual(process.terminated,1);self.assertEqual(process.killed,1)
        self.assertEqual(process.returncode,-9);self.assertTrue(process.directory.exists())
        self.assertEqual(len(self.factory.processes),1)

    def test_unreapable_checker_failure_is_explicit_and_keeps_original_cause(self):
        self.factory.timeout=self.factory.stubborn=self.factory.unreapable=True
        with self.assertRaisesRegex(RuntimeError,'cleanup failed') as caught:self.load(timeout_sec=.5)
        self.assertIsInstance(caught.exception.__cause__,subprocess.TimeoutExpired)
        self.assertEqual(self.factory.processes[0].killed,1)

    def test_before_after_artifact_change_and_later_registered_change_fail_closed(self):
        before=deepcopy(self.reports[self.overlay]);after=deepcopy(before);after['outputs']['cell.xml']='0'*64
        self.identity.side_effect=[before,after]
        with self.assertRaisesRegex(ValueError,'changed while'):self.load()
        self.identity.side_effect=lambda directory,*args:deepcopy(self.reports[directory])
        source=self.load()[0]
        self.reports[self.overlay]['fingerprint']='0'*64
        with self.assertRaisesRegex(ValueError,'Registered overlay changed'):source.validate()
        self.assertEqual(source.home,dict(left_q=0.,right_q=.2))

    def test_wrong_joint_or_base_contract_and_invalid_timeouts_do_not_spawn(self):
        for change in (lambda value:value.update(base_workcell_id='other'),lambda value:value.update(id='pair'),
            lambda value:value.update(joint_names=['wrong']),lambda value:value.update(home_rad=[0.]),
            lambda value:value.update(id='file:private')):
            self.reports[self.overlay]=manifest();change(self.reports[self.overlay])
            with self.assertRaises(ValueError):self.load()
        for value in (True,0.,61.,float('inf'),float('nan')):
            with self.assertRaises(ValueError):self.load(timeout_sec=value)
        self.assertEqual(self.factory.processes,[])


class EnvironmentCheckWorkerTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        self.root=Path(temp.name);self.overlay=self.root/'overlay';self.overlay.mkdir()
        self.receipt=self.root/'private/receipt.json';self.receipt.parent.mkdir(mode=0o700)
        self.report=manifest();self.base=object()
        self.args=NS(parent_pid=42,manifest_sha256='f'*64,worker_sha256=hashlib.sha256(Path(worker.__file__).read_bytes()).hexdigest(),
            request_token='a'*32,overlay=str(self.overlay),receipt=str(self.receipt),workspace=str(self.root),base_workcell='pair',cpu_limit_sec=32)
        self.parent=patch.object(worker.os,'getppid',return_value=42).start()
        self.load=patch.object(worker,'_load_base',return_value=self.base).start()
        self.check=patch.object(worker,'_check_full',return_value=self.report).start()
        self.addCleanup(patch.stopall)

    def test_full_cpu_report_receipt_is_bounded_detached_and_exclusive(self):
        value=worker.check_and_receipt(self.args)
        expected=sources._receipt(self.report,token=self.args.request_token,
            manifest_sha256=self.args.manifest_sha256,worker_sha256=self.args.worker_sha256)
        self.assertEqual(value,expected);self.assertEqual(json.loads(self.receipt.read_bytes()),expected)
        self.check.assert_called_once_with(self.overlay,self.base,self.args.manifest_sha256)
        self.assertEqual(self.receipt.stat().st_mode & 0o777,0o600)
        self.assertNotIn('private-design',self.receipt.read_text())
        self.report['geometry_check']['home_contacts']=1
        self.assertEqual(value['geometry_check']['home_contacts'],0)
        saved=self.receipt.read_bytes()
        with self.assertRaises(ValueError):worker.check_and_receipt(self.args)
        self.assertEqual(self.receipt.read_bytes(),saved)

    def test_parent_or_worker_identity_mismatch_never_runs_full_checker(self):
        self.parent.return_value=1
        with self.assertRaisesRegex(ValueError,'parent'):worker.check_and_receipt(self.args)
        self.parent.return_value=42;self.args.worker_sha256='0'*64
        with self.assertRaisesRegex(ValueError,'implementation'):worker.check_and_receipt(self.args)
        self.check.assert_not_called();self.assertFalse(self.receipt.exists())

    def test_lost_parent_after_cpu_check_never_publishes_receipt(self):
        self.parent.side_effect=[42,1]
        with self.assertRaisesRegex(ValueError,'parent'):worker.check_and_receipt(self.args)
        self.check.assert_called_once();self.assertFalse(self.receipt.exists())

    def test_main_failures_do_not_print_private_engine_paths_or_mutate_test_resource_limits(self):
        private_path=str(self.root/'private-design.step')
        with patch.object(worker.argparse.ArgumentParser,'parse_args',return_value=self.args), \
             patch.object(worker.resource,'setrlimit') as limits, \
             patch.object(worker,'check_and_receipt',side_effect=ValueError(private_path)), \
             patch.object(worker.sys,'stderr',new_callable=io.StringIO) as error:
            self.assertEqual(worker.main(),1)
            self.assertNotIn(private_path,error.getvalue());self.assertIn('ValueError',error.getvalue())
            self.assertEqual(limits.call_count,3)

    def test_modules_import_without_ros_mujoco_or_gl(self):
        code="""import sys
for name in ('mujoco','numpy','rclpy','OpenGL','glfw'):sys.modules[name]=None
from mujoco_ros2_manipulation.operator_console import environment_sources
from mujoco_ros2_manipulation.workcells import environment_check_worker
assert environment_sources.WORKER_MODULE.endswith('environment_check_worker')
"""
        subprocess.run([sys.executable,'-c',code],check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)


if __name__=='__main__':unittest.main()
