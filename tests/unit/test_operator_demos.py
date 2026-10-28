"""Owned comparison admission, cancellation, cleanup and frame identity without ROS."""
from copy import deepcopy
import json
import hashlib
from pathlib import Path
import queue
import signal
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.catalog import describe_scenarios
from mujoco_ros2_manipulation.operator_console.demo_jobs import matching_process
from mujoco_ros2_manipulation.operator_console.demo_worker import DemoCancelled, DemoJob, frame_matches
from mujoco_ros2_manipulation.operator_console.http_server import ConsoleHTTPServer, Handler
from mujoco_ros2_manipulation.operator_console.state import ConsoleState
from mujoco_ros2_manipulation.operator_console.verify_demos import completed_job, checked_reports
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.workcells.motion_request import command

JOBS = 'mujoco_ros2_manipulation.operator_console.demo_jobs'


class DemoApplicationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.now = [3.0]
        config = dict(domain=91, home_rad=dict(l=0., r=0.),
                      joint_limits={n:dict(lower=-1., upper=1.) for n in ('l', 'r')})
        self.state = ConsoleState(config, clock=lambda:self.now[0])
        self.state.started = 0
        self.cell = NS(id='fr5_ur5e', root=self.root,
                       output=self.root/'runtime/generated/workcells/fr5_ur5e')
        self.cell.output.mkdir(parents=True)
        (self.cell.output/'cell.xml').write_text('<mujoco/>')
        self.bridge = NS(outbox=queue.Queue())
        with patch.object(ConsoleApplication, 'watch', return_value=None):
            self.app = ConsoleApplication(self.cell, self.state, self.bridge,
                                          self.root/'runtime/operator/session', render=False)
            self.app.monitor.join(timeout=2)
        self.addCleanup(self.close_app)
        self.process = Mock(pid=987654, returncode=None)
        self.process.poll.return_value = None
        self.process.wait.return_value = 0

    def close_app(self):
        # Tests never spawn a native child; leave no mock ownership for close().
        self.app.demos.process = None
        self.app.demos.stopping = False
        self.app.children.clear()
        self.app.worker = None
        self.app.close()

    def spawn(self, kind, args, folder, **environment):
        self.app.children[kind] = self.process
        return self.process

    def start_job(self, scenario='contact_comparison'):
        with patch.object(self.app, 'spawn', side_effect=self.spawn):
            return self.app.demos.start(dict(scenario=scenario, native_gui=False))

    def launch(self, *, run_id='owned-run', pid=876543):
        directory = self.root/'runtime/validation/runs'/run_id
        directory.mkdir(parents=True, exist_ok=True)
        model = self.cell.output/'cell.xml'
        return dict(pid=pid, start_ticks=1234, run_id=run_id,
                    report_path=str(directory/'result.json'), model_path=str(model),
                    model_sha256=file_sha256(model))

    def status(self, **changes):
        data = dict(self.app.demos.current, **changes)
        atomic_json(self.app.demos.folder/'status.json', data)
        return data

    def test_only_fixed_scenarios_and_boolean_native_gui_are_admitted(self):
        for options in (dict(scenario='../../arbitrary', native_gui=False),
                        dict(scenario='contact_comparison', native_gui=1),
                        dict(scenario='contact_comparison', native_gui=False, command='shell')):
            with self.subTest(options=options), patch.object(self.app, 'spawn') as spawn:
                with self.assertRaises(ValueError):
                    self.app.demos.start(options)
                spawn.assert_not_called()
        self.cell.id = 'other_workcell'
        with self.assertRaises(ValueError):
            self.start_job()

    def test_existing_ros_owner_and_owned_manual_run_block_demo_start(self):
        for mode in ('graph', 'phase', 'child'):
            with self.subTest(mode=mode):
                self.state.graph.update(clock=1 if mode == 'graph' else 0)
                self.app.phase = 'running' if mode == 'phase' else 'idle'
                self.app.children = {'simulation':self.process} if mode == 'child' else {}
                with patch.object(self.app, 'spawn') as spawn, self.assertRaises(RuntimeError):
                    self.start_job()
                spawn.assert_not_called()
        self.app.children.clear()
        self.app.phase = 'idle'

    def test_demo_excludes_duplicate_start_manual_start_and_joint_dispatch(self):
        payload = command(dict(l=.2, r=.2))
        reply = self.start_job()
        self.assertEqual(reply['status'], 'starting')
        self.assertTrue(self.app.demos.busy)
        with self.assertRaises(RuntimeError):
            self.start_job('timing_comparison')
        with self.assertRaises(RuntimeError):
            self.app.start(dict(native_gui=False, planning_case='none'))
        with self.assertRaises(RuntimeError):
            self.app.submit(payload)
        self.assertTrue(self.bridge.outbox.empty())
        self.assertFalse(self.app.snapshot()['can_submit'])

    def test_new_job_clears_previous_frames_and_source_observation(self):
        self.state.joints(['l','r'], [0,0], [0,0], source_stamp=80)
        self.state.observed_model_path = 'old-model'
        (self.app.directory/'frame.json').write_text('{"job_id":"old"}')
        (self.app.directory/'frame.jpg').write_bytes(b'old')
        self.start_job()
        self.assertIsNone(self.state.positions)
        self.assertIsNone(self.state.joint_state_stamp_simulation_sec)
        self.assertIsNone(self.state.observed_model_path)
        self.assertFalse((self.app.directory/'frame.json').exists())
        self.assertFalse((self.app.directory/'frame.jpg').exists())

    def test_wrong_job_cancel_never_signals_and_requested_cancel_waits_for_cleanup(self):
        reply = self.start_job()
        with self.assertRaises(RuntimeError):
            self.app.demos.request_stop('f'*32)
        self.process.send_signal.assert_not_called()
        with patch(JOBS+'.threading.Thread') as thread:
            result = self.app.demos.request_stop(reply['job_id'])
        thread.return_value.start.assert_called_once()
        self.assertEqual(result['status'], 'cancel_requested')
        self.assertEqual(self.app.demos.snapshot()['current']['status'], 'cancel_requested')
        self.assertTrue(self.app.demos.busy)
        self.process.send_signal.assert_called_once_with(signal.SIGINT)
        with self.assertRaises(RuntimeError):
            self.start_job()
        self.process.poll.return_value = 0
        self.process.returncode = 0
        self.app.demos.current.update(status='succeeded',
            comparison={'must_not_be_presented_as_complete':True})
        with patch.object(self.app.demos, 'read_status'), patch(JOBS+'.group_gone', return_value=True):
            self.app.demos._stop_sync()
        self.assertFalse(self.app.demos.busy)
        self.assertEqual(self.app.demos.current['status'], 'cancelled')
        self.assertIsNone(self.app.demos.current['comparison'])

    def test_unproven_external_groups_are_never_signalled_and_keep_admission_blocked(self):
        self.start_job()
        record = self.launch()
        self.app.demos.launches[(record['pid'],record['start_ticks'])] = record
        self.process.poll.return_value = 1
        with patch(JOBS+'.group_gone', side_effect=lambda pid:pid == self.process.pid), \
             patch(JOBS+'.matching_process', return_value=False), \
             patch(JOBS+'.process_identity', return_value=None), patch(JOBS+'.os.killpg') as kill:
            self.app.demos.cleanup_launches()
            self.app.demos.finish(forced=True)
        kill.assert_not_called()
        self.assertTrue(self.app.demos.busy)
        self.assertTrue(self.app.demos.cleanup_blocked)
        self.assertEqual(self.app.demos.current['status'], 'failed')
        self.assertFalse(self.app.demos.current['cleanup_clean'])

    def test_external_session_leader_cannot_be_registered_as_a_worker_child(self):
        self.start_job()
        record = self.launch()
        self.status(status='running', active_launch=record)
        external = dict(pid=record['pid'], start_ticks=record['start_ticks'],
                        pgid=record['pid'], sid=record['pid'], ppid=112233)
        with patch(JOBS+'.process_identity', return_value=external), \
             patch.object(self.app.demos, 'group_members', return_value=[external]), \
             patch(JOBS+'.os.killpg') as kill:
            with self.assertRaises(ValueError):
                self.app.demos.read_status()
        kill.assert_not_called()
        self.assertFalse(self.app.demos.launches)

    def test_corrupt_sidecar_cannot_replace_trusted_state_or_skip_owned_cleanup(self):
        for corruption, cleanup_path in (('invalid_json','crashed_worker'),('foreign_model','cancel_cleanup')):
            with self.subTest(corruption=corruption):
                self.process.poll.return_value=None
                reply=self.start_job()
                manager=self.app.demos
                record=self.launch()
                manager.launches[(record['pid'],record['start_ticks'])]=record
                manager.current['active_launch']=record
                trusted=deepcopy(manager.current)
                if corruption=='invalid_json':
                    (manager.folder/'status.json').write_text('{interrupted sidecar')
                else:
                    self.status(active_launch=dict(record,model_path=str(self.root/'unowned.xml')))
                with self.assertRaises(ValueError):
                    manager.read_status()
                self.assertEqual(manager.current,trusted)
                self.process.poll.return_value=1
                alive=[True]
                def gone(pid):
                    return not alive[0] if pid==record['pid'] else True
                def kill(pid,sig):
                    self.assertEqual(pid,record['pid'])
                    alive[0]=False
                with patch(JOBS+'.group_gone',side_effect=gone), \
                     patch.object(manager,'group_owned',return_value=True), \
                     patch(JOBS+'.os.killpg',side_effect=kill) as signalled:
                    if cleanup_path=='crashed_worker':
                        manager.finish_exited()
                    else:
                        manager._stop_sync()
                signalled.assert_called_once_with(record['pid'],signal.SIGTERM)
                self.assertFalse(alive[0])
                self.assertTrue(manager.monitor_error)
                self.assertEqual(manager.current['job_id'],reply['job_id'])
                self.assertEqual(manager.current['status'],'failed')
                self.assertFalse(manager.busy)
                self.assertFalse(manager.stopping)
                self.assertNotIn('demo',self.app.children)

    def test_worker_crash_schedules_owned_launch_cleanup_before_release(self):
        self.start_job()
        record = self.launch()
        self.app.demos.launches[(record['pid'],record['start_ticks'])] = record
        self.process.poll.return_value = 1
        alive = [True]
        def gone(pid):
            return not alive[0] if pid == record['pid'] else True
        def kill(pid, sig):
            self.assertEqual(pid, record['pid'])
            alive[0] = False
        with patch.object(self.app.demos, 'read_status'), \
             patch(JOBS+'.threading.Thread') as thread:
            self.app.demos.tick()
        self.assertTrue(self.app.demos.stopping)
        self.assertTrue(self.app.demos.busy)
        self.assertEqual(thread.call_args.kwargs['target'], self.app.demos.finish_exited)
        with patch.object(self.app.demos, 'read_status'), patch(JOBS+'.group_gone', side_effect=gone), \
             patch.object(self.app.demos, 'group_owned', return_value=True), \
             patch(JOBS+'.os.killpg', side_effect=kill) as signalled:
            self.app.demos.finish_exited()
        signalled.assert_called_once_with(record['pid'], signal.SIGTERM)
        self.assertFalse(alive[0])
        self.assertFalse(self.app.demos.busy)
        self.assertEqual(self.app.demos.current['status'], 'failed')

    def test_owned_launch_ignoring_term_escalates_to_kill_without_touching_other_groups(self):
        self.start_job()
        record = self.launch()
        self.app.demos.launches[(record['pid'],record['start_ticks'])] = record
        alive = [True]
        now = [0]
        def clock():
            now[0] += 6
            return now[0]
        def kill(pid, sig):
            self.assertEqual(pid, record['pid'])
            if sig == signal.SIGKILL:
                alive[0] = False
        with patch(JOBS+'.group_gone', side_effect=lambda _pid:not alive[0]), \
             patch.object(self.app.demos, 'group_owned', return_value=True), \
             patch(JOBS+'.os.killpg', side_effect=kill) as signalled, \
             patch(JOBS+'.time.monotonic', side_effect=clock), patch(JOBS+'.time.sleep'):
            forced = self.app.demos.cleanup_launches()
        self.assertTrue(forced)
        self.assertEqual([call.args for call in signalled.call_args_list],
                         [(record['pid'],signal.SIGTERM),(record['pid'],signal.SIGKILL)])
        self.assertFalse(alive[0])

    def test_reused_pid_does_not_recover_group_ownership_from_an_old_record(self):
        record = self.launch()
        reused = dict(pid=record['pid'], start_ticks=record['start_ticks']+1,
                      pgid=record['pid'], sid=record['pid'])
        with patch(JOBS+'.process_identity', return_value=reused):
            self.assertFalse(matching_process(record))
        self.app.demos.members[(record['pid'],record['start_ticks'])] = [dict(reused,start_ticks=10)]
        with patch(JOBS+'.matching_process', return_value=False), patch(JOBS+'.process_identity', return_value=reused):
            self.assertFalse(self.app.demos.group_owned(record))

    def test_renderer_waits_for_exact_actual_model_and_rejects_changed_model_hash(self):
        self.app.render_enabled = True
        self.app.image_render_enabled = True
        self.start_job()
        record = self.launch()
        self.status(status='running', active_launch=record)
        self.state.graph.update(clock=1,joints=1)
        self.state.joints(['l','r'], [0,0], [0,0], source_stamp=1)
        self.state.observed_model_path = 'different-model'
        owned_identity = dict(pid=record['pid'],start_ticks=record['start_ticks'],ppid=self.process.pid,pgid=record['pid'],sid=record['pid'])
        with patch(JOBS+'.matching_process', return_value=True), \
             patch(JOBS+'.process_identity', return_value=owned_identity), \
             patch.object(self.app.demos, 'group_members', return_value=[]), \
             patch.object(self.app, 'spawn') as spawn:
            self.app.demos.tick()
        spawn.assert_not_called()
        self.state.observed_model_path = record['model_path']
        Path(record['model_path']).write_text('<mujoco changed="true"/>')
        with patch(JOBS+'.matching_process', return_value=True), \
             patch(JOBS+'.process_identity', return_value=owned_identity), \
             patch.object(self.app.demos, 'group_members', return_value=[]), \
             patch.object(self.app.demos, 'request_stop') as cancel, patch.object(self.app, 'spawn') as spawn:
            self.app.demos.tick()
        spawn.assert_not_called()
        cancel.assert_called_once_with(self.app.demos.current['job_id'])


    def test_monitor_failure_stays_failed_after_clean_cancellation_cleanup(self):
        self.start_job()
        self.app.demos.monitor_error = 'Owned model changed'
        self.app.demos.cancel_requested = True
        self.app.demos.current.update(status='cancelled', comparison={'invalid':True})
        self.process.poll.return_value = 0
        with patch(JOBS+'.group_gone',return_value=True):
            self.app.demos.finish()
        self.assertEqual(self.app.demos.current['status'],'failed')
        self.assertEqual(self.app.demos.current['error'],'Owned model changed')
        self.assertIsNone(self.app.demos.current['comparison'])
        self.assertTrue(self.app.demos.current['cleanup_clean'])
        self.assertFalse(self.app.demos.busy)

    def test_cancel_request_does_not_hide_worker_failure_after_clean_cleanup(self):
        for code, worker_status in ((1, 'failed'), (130, 'failed'), (130, 'cancelled')):
            with self.subTest(code=code, worker_status=worker_status):
                self.process.poll.return_value = None
                self.start_job()
                manager = self.app.demos
                manager.cancel_requested = True
                error = dict(type='RuntimeError', message='genuine decoder failure')
                self.status(status=worker_status, error=error, comparison={'invalid':True})
                manager.read_status()
                self.assertEqual(manager.current['status'], worker_status)
                self.assertEqual(manager.snapshot()['current']['status'], 'cancel_requested')
                self.assertTrue(manager.busy)
                self.process.poll.return_value = code
                with patch(JOBS+'.group_gone', return_value=True):
                    manager.finish()
                self.assertEqual(manager.current['status'], 'failed')
                self.assertEqual(manager.current['error'], error)
                self.assertEqual(manager.history[0]['error'], error)
                self.assertIsNone(manager.current['comparison'])
                self.assertTrue(manager.current['cleanup_clean'])
                self.assertFalse(manager.busy)

    def test_confirmed_worker_cancellation_waits_for_cleanup_before_completion(self):
        self.start_job()
        manager = self.app.demos
        manager.cancel_requested = True
        self.status(status='cancelled', error=dict(type='Cancelled', message='user request'))
        manager.read_status()
        self.assertEqual(manager.snapshot()['current']['status'], 'cancel_requested')
        self.assertEqual(manager.snapshot()['current']['stage'], 'cleanup')
        self.assertTrue(manager.busy)
        self.process.poll.return_value = 130
        with patch(JOBS+'.group_gone', return_value=True):
            manager.finish()
        self.assertEqual(manager.snapshot()['current']['status'], 'cancelled')
        self.assertIsNone(manager.current['error'])
        self.assertTrue(manager.current['cleanup_clean'])
        self.assertFalse(manager.busy)

    def test_cancellation_without_worker_confirmation_stays_failed(self):
        self.start_job()
        manager = self.app.demos
        manager.cancel_requested = True
        self.status(status='cancelled', error=None)
        manager.read_status()
        self.process.poll.return_value = 130
        with patch(JOBS+'.group_gone', return_value=True):
            manager.finish()
        self.assertEqual(manager.current['status'], 'failed')
        self.assertIn('exit=130', manager.current['error'])
        self.assertFalse(manager.busy)

    def test_worker_terminal_status_is_hidden_until_owned_cleanup_finishes(self):
        self.start_job()
        self.status(status='succeeded', stage='complete', comparison={'complete_pair':True})
        self.app.demos.read_status()
        current = self.app.snapshot()['demo']['current']
        self.assertEqual(current['status'], 'running')
        self.assertEqual(current['stage'], 'cleanup')
        self.assertTrue(self.app.demos.busy)
        self.app.demos.cancel_requested = True
        self.assertEqual(self.app.snapshot()['demo']['current']['status'], 'cancel_requested')

    def test_known_descendant_remains_owned_after_leader_exit_and_reparenting(self):
        record = self.launch()
        previous = dict(pid=765432, start_ticks=4321, pgid=record['pid'],
                        sid=record['pid'], ppid=record['pid'])
        self.app.demos.members[(record['pid'],record['start_ticks'])] = [previous]
        reparented = dict(previous, ppid=1)
        with patch(JOBS+'.matching_process', return_value=False), \
             patch(JOBS+'.process_identity', return_value=reparented):
            self.assertTrue(self.app.demos.group_owned(record))

    def test_http_demo_mutations_require_origin_token_and_keep_manual_dispatch_exclusive(self):
        server = ConsoleHTTPServer(('127.0.0.1',0), Handler)
        server.application, server.token = self.app, 'demo-test-token'
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        url = 'http://127.0.0.1:'+str(server.server_address[1])
        def post(route, payload, **overrides):
            headers = {'Content-Type':'application/json','Origin':url,'X-Operator-Token':server.token,**overrides}
            with urlopen(Request(url+route,data=json.dumps(payload).encode(),headers=headers),timeout=3) as response:
                return response.status,json.load(response)
        try:
            options = dict(scenario='contact_comparison',native_gui=False)
            for headers in ({'Origin':'https://foreign.invalid'},{'X-Operator-Token':'wrong'}):
                with self.subTest(headers=headers), self.assertRaises(HTTPError) as error:
                    post('/api/demos/start',options,**headers)
                self.assertEqual(error.exception.code,403)
                self.assertIsNone(self.app.demos.process)
            with patch.object(self.app,'spawn',side_effect=self.spawn):
                code, reply = post('/api/demos/start',options)
            self.assertEqual(code,202)
            for route,payload in (('/api/demos/start',options),
                                  ('/api/start',dict(native_gui=False,planning_case='none')),
                                  ('/api/command',command(dict(l=.2,r=.2))),
                                  ('/api/demos/stop',dict(job_id='f'*32))):
                with self.subTest(route=route),self.assertRaises(HTTPError) as error:
                    post(route,payload)
                self.assertEqual(error.exception.code,409)
            with patch.object(self.app.demos,'stop_sync'):
                code, stopped = post('/api/demos/stop',dict(job_id=reply['job_id']))
            self.assertEqual(code,202)
            self.assertEqual(stopped['status'],'cancel_requested')
            self.assertTrue(self.bridge.outbox.empty())
        finally:
            server.shutdown();server.server_close();thread.join(timeout=2)


class DemoWorkerTests(unittest.TestCase):
    def make_job(self, *, render=False):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        console = root/'runtime/operator/session'
        return DemoJob(console/'demos'/('a'*32), 'contact_comparison', 91, console, render=render, root=root)

    def test_catalog_fixed_variants_and_detached_descriptions(self):
        first = describe_scenarios()
        self.assertEqual({s['id'] for s in first}, {'contact_comparison','timing_comparison'})
        self.assertEqual([v['id'] for v in first[0]['variants']], ['contact','control'])
        first[0]['variants'].clear()
        self.assertEqual(len(describe_scenarios()[0]['variants']), 2)

    def test_only_fresh_exact_job_run_model_frames_unlock_motion(self):
        frame = dict(job_id='a',run_id='r',model_sha256='f'*64,sequence=1,
                     rendered_monotonic=9.5,source_state_age_sec=.1)
        self.assertTrue(frame_matches(frame,'a','r','f'*64,now=10))
        for changes in (dict(job_id='old'),dict(run_id='old'),dict(model_sha256='e'*64),
                        dict(rendered_monotonic=9),dict(rendered_monotonic=11),
                        dict(source_state_age_sec=.5),dict(source_state_age_sec=float('nan')),
                        dict(sequence=0),dict(sequence=True),dict(error='renderer failed')):
            with self.subTest(changes=changes):
                self.assertFalse(frame_matches(dict(frame,**changes),'a','r','f'*64,now=10))

    def test_model_hash_is_published_before_current_frame_unlocks_motion(self):
        job = self.make_job(render=True)
        directory = job.root/'runtime/validation/runs/owned-run'
        directory.mkdir(parents=True)
        model = directory/'contact_test.xml'
        model.write_text('<mujoco/>')
        report = NS(directory=directory,path=directory/'result.json',run_id='owned-run')
        job.data['variant'] = 'contact'
        with patch('mujoco_ros2_manipulation.operator_console.demo_worker.read_process_start_ticks',return_value=123):
            job._on_launch(report,NS(),NS(),NS(pid=4242))
        self.assertIsNone(job.data['active_launch']['model_sha256'])
        atomic_json(job.console_directory/'frame.json',dict(job_id=job.data['job_id'],run_id=report.run_id,
            model_sha256=file_sha256(model),sequence=1,rendered_monotonic=time.monotonic(),source_state_age_sec=.01))
        node = NS(until=lambda predicate,*args:self.assertTrue(predicate()))
        job._frame_barrier(node)
        self.assertEqual(job.data['active_launch']['model_sha256'],file_sha256(model))
        self.assertEqual(job.data['stage'],'frame_ready')

    def test_cancelled_first_variant_never_starts_second_variant_or_comparison(self):
        job = self.make_job()
        ros = ModuleType('rclpy');ros.init=Mock();ros.ok=Mock(return_value=False);ros.shutdown=Mock()
        signals = ModuleType('rclpy.signals');signals.SignalHandlerOptions=NS(NO=0)
        service = ModuleType('std_srvs.srv');service.Trigger=object
        contact = ModuleType('mujoco_ros2_manipulation.workcells.verify_contact_demo')
        contact.VisibleContactDemo=object;contact.finalize_demo=Mock()
        modules={'rclpy':ros,'rclpy.signals':signals,'std_srvs':ModuleType('std_srvs'),
                 'std_srvs.srv':service,'mujoco_ros2_manipulation.workcells.verify_contact_demo':contact}
        def interrupted(*args, **kwargs):
            directory = job.root/'runtime/validation/runs/cancelled-run'
            directory.mkdir(parents=True)
            path = directory/'result.json'
            atomic_json(path, dict(run_id='cancelled-run', status='interrupted',
                error=dict(type='DemoCancelled', message='user request'),
                shutdown_clean=True, process_group_gone=True))
            job.active_report = NS(path=path)
            return 130
        with patch.dict('sys.modules',modules), \
             patch('mujoco_ros2_manipulation.verification.runner.run_verification',side_effect=interrupted) as run, \
             patch('mujoco_ros2_manipulation.workcells.contact_comparison.compare_runs') as compare:
            with self.assertRaises(DemoCancelled):
                job._contact_pair(NS(id='fr5_ur5e'))
        self.assertEqual(run.call_count,1)
        compare.assert_not_called()

    def test_cancellation_between_variants_preserves_cancelled_without_comparison(self):
        job = self.make_job()
        def cancelled(_cell):
            job.cancel_requested=True
            job.check_cancel()
        with patch('mujoco_ros2_manipulation.workcells.config.load_workcell',return_value=NS()), \
             patch.object(job,'_contact_pair',side_effect=cancelled), \
             patch.object(job,'_timing_pair') as other,patch.dict('os.environ'):
            self.assertEqual(job.run(),130)
        other.assert_not_called()
        data=json.loads(job.path.read_text())
        self.assertEqual(data['status'],'cancelled')
        self.assertIsNone(data['comparison'])
        self.assertFalse((job.directory/'comparison.json').exists())


class LaunchHookTests(unittest.TestCase):
    def test_launch_hook_precedes_ros_init_and_failure_still_cleans_owned_launch(self):
        for hook_fails in (False,True):
            with self.subTest(hook_fails=hook_fails),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);output=root/"models/fr5";output.mkdir(parents=True)
                (output/"source.json").write_text("{}")
                profile=NS(id="fr5",label="FR5",fingerprint="hash",output=output,ros={"planning_group":"arm"})
                args=NS(robot="fr5",gui=False,domain=91,stop_mode="interrupt",screenshot_dir=None)
                ros=Mock();ros.ok.return_value=False
                node=Mock();node.run.return_value={}
                process=Mock(pid=42,returncode=0);process.poll.return_value=None
                def hook(report,actual,options,owned):
                    ros.init.assert_not_called()
                    self.assertIs(actual,profile);self.assertIs(owned,process)
                    self.assertEqual(report.data["stage"],"launch")
                    self.assertTrue(report.path.is_file())
                    if hook_fails:raise RuntimeError("identity capture failed")
                with patch("mujoco_ros2_manipulation.verification.runner.load_profile",return_value=profile), \
                     patch("mujoco_ros2_manipulation.verification.runner.check_generated_model"), \
                     patch("mujoco_ros2_manipulation.verification.runner.subprocess.Popen",return_value=process), \
                     patch("mujoco_ros2_manipulation.verification.runner.stop_process",return_value=False) as stop, \
                     patch("mujoco_ros2_manipulation.verification.runner.group_gone",return_value=True), \
                     patch.dict("os.environ"),patch("builtins.print"):
                    code=run_verification(args,root,ros,lambda *_args,**_kw:node,None,on_launch=hook)
                stop.assert_called_once_with(process)
                self.assertEqual(code,1 if hook_fails else 0)
                report=json.loads((root/"runtime/validation/fr5_headless.json").read_text())
                self.assertEqual(report["shutdown_clean"],True)
                if hook_fails:
                    self.assertEqual(report["error"]["message"],"identity capture failed")
                    ros.init.assert_not_called()


class DemoVerificationTests(unittest.TestCase):
    def test_terminal_worker_report_is_not_completion_before_owned_cleanup(self):
        identifier='a'*32
        current=dict(job_id=identifier,status='succeeded',cleanup_clean=True,active_launch=None)
        good=dict(demo=dict(current=current,can_start=True),lifecycle=dict(owned={}))
        self.assertEqual(completed_job(good,identifier),current)
        for change in ('wrong_job','running','unclean','active_launch','owned_worker','domain_busy'):
            state=deepcopy(good)
            if change=='wrong_job':state['demo']['current']['job_id']='b'*32
            if change=='running':state['demo']['current']['status']='running'
            if change=='unclean':state['demo']['current']['cleanup_clean']=False
            if change=='active_launch':state['demo']['current']['active_launch']={'pid':42}
            if change=='owned_worker':state['lifecycle']['owned']['demo']=True
            if change=='domain_busy':state['demo']['can_start']=False
            with self.subTest(change=change):
                self.assertIsNone(completed_job(state,identifier))

    def test_fresh_report_sha_domain_time_and_physical_success_are_all_required(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            folder=root/'runtime/validation/runs/new-run';folder.mkdir(parents=True)
            path=folder/'result.json'
            report=dict(run_id='new-run',ros_domain_id=91,started_utc='2026-10-23T01:00:01+00:00',
                        status='passed',shutdown_clean=True,process_group_gone=True,
                        motion_checks_passed=True,physics_checks_passed=True)
            atomic_json(path,report)
            summary=dict(variant='contact',run_id='new-run',report_path=str(path),report_sha256=file_sha256(path))
            job=dict(results=[summary])
            self.assertEqual(checked_reports(job,('contact',),root,91,'2026-10-23T01:00:00+00:00')['contact'][1],report)
            for change in ('sha','domain','archived','physics','shutdown','outside'):
                data=deepcopy(report);candidate=deepcopy(job)
                if change=='domain':data['ros_domain_id']=92
                if change=='archived':data['started_utc']='2026-10-22T01:00:00+00:00'
                if change=='physics':data['physics_checks_passed']=False
                if change=='shutdown':data['shutdown_clean']=False
                atomic_json(path,data)
                candidate['results'][0]['report_sha256']=file_sha256(path)
                if change=='sha':candidate['results'][0]['report_sha256']='0'*64
                if change=='outside':candidate['results'][0]['report_path']=str(root/'foreign.json')
                with self.subTest(change=change),self.assertRaises(RuntimeError):
                    checked_reports(candidate,('contact',),root,91,'2026-10-23T01:00:00+00:00')


class DemoFrameHTTPTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory=Path(self.temporary.name)
        self.image=self.directory/'frame.jpg'
        self.metadata=self.directory/'frame.json'
        self.pixels=b'complete-frame-bytes'
        self.image.write_bytes(self.pixels)
        self.frame=dict(sequence=7,job_id='a'*32,run_id='owned-run',model_sha256='f'*64,
                        jpeg_sha256=hashlib.sha256(self.pixels).hexdigest())
        atomic_json(self.metadata,self.frame)
        self.server=ConsoleHTTPServer(('127.0.0.1',0),Handler)
        self.server.application=NS(directory=self.directory)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.addCleanup(self.close_server)
        self.url='http://127.0.0.1:'+str(self.server.server_address[1])

    def close_server(self):
        self.server.shutdown();self.server.server_close();self.thread.join(timeout=2)

    def get(self, query=''):
        with urlopen(self.url+'/api/frame'+query,timeout=3) as response:
            return response.status,response.read()

    def test_frame_pixels_require_current_query_identity_and_matching_digest(self):
        query='?job_id='+self.frame['job_id']+'&run_id=owned-run&model_sha256='+self.frame['model_sha256']+'&sequence=7'
        self.assertEqual(self.get(query),(200,self.pixels))
        for wrong in ('job_id=old','run_id=old','model_sha256=old','sequence=6','sequence=7&sequence=8'):
            with self.subTest(query=wrong):
                self.assertEqual(self.get('?'+wrong),(204,b''))
        self.image.write_bytes(b'new-scene-before-new-metadata')
        self.assertEqual(self.get(query),(204,b''))
        self.image.write_bytes(self.pixels)
        self.frame.pop('jpeg_sha256')
        atomic_json(self.metadata,self.frame)
        self.assertEqual(self.get(),(204,b''))

    def test_metadata_change_while_reading_pixels_returns_no_frame(self):
        original=Path.read_bytes
        calls=[0]
        before=self.metadata.read_bytes()
        after=json.dumps(dict(self.frame,sequence=8)).encode()
        def changed(path):
            if path==self.metadata:
                calls[0]+=1
                return before if calls[0]==1 else after
            return original(path)
        with patch.object(Path,'read_bytes',changed):
            self.assertEqual(self.get(),(204,b''))
        self.assertGreaterEqual(calls[0],2)
