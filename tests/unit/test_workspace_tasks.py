"""Task ownership and same-origin native display routing, with no ROS/physics."""
from copy import deepcopy
from http.client import HTTPConnection
import json
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.console_views import ConsoleViews
from mujoco_ros2_manipulation.operator_console.http_server import ConsoleHTTPServer, Handler
from mujoco_ros2_manipulation.operator_console.state import ConsoleState

MODEL, PREVIEW, LIVE = 'b' * 64, 'c' * 64, 'd' * 64


class Host:
    def __init__(self):
        self.identifier = 'a' * 64
        self.invalidated = 0
    def invalidate(self):
        self.identifier = None
        self.invalidated += 1
    def manifest_bytes(self, identifier):
        if identifier != self.identifier:
            raise ValueError('Normal binding changed')
        return b'normal-model'
    def pose_envelope(self, identifier):
        self.manifest_bytes(identifier)
        return dict(kind='normal', binding_id=identifier)
    def asset_bytes(self, digest):
        if digest != 'e' * 64:
            raise KeyError('Not normal mesh')
        return b'normal-mesh'
    def hierarchy_bytes(self, identifier):
        self.manifest_bytes(identifier)
        return None


class Views:
    def __init__(self, app, **_):
        self.app, self.host = app, Host()
        self.enabled, self.mode, self.preview = True, 'live', False
        self.base = NS(id='workcell:fixture')
        self.source = self.key = self.stamp_key = self.last_accepted_stamp = None
        self.revision = 0
        self.error = None
    def tick(self):
        self.revision += 1
        self.host.identifier = f'{self.revision:064x}'
    def snapshot(self):
        return dict(enabled=True, mode='live', binding_id=self.host.identifier, preview_active=self.preview)
    def close(self):
        pass


class Native:
    """Retain deliberately permissive old-preview reads to test wrapper scope."""
    def __init__(self):
        self.domain = 231
        self.source = dict(id='scene:ros_fr5_grasp', label='Native fixture', model_sha256=MODEL)
        self.job = None
        self.calls = []
        self.closed = False
        self.clean_on_close = True
        self.closing = threading.Event()
    def snapshot(self):
        identifier = PREVIEW if self.job is None else LIVE
        return dict(source=deepcopy(self.source), job=deepcopy(self.job), view=dict(
            binding_id=identifier, binding=dict(operator_session_id='f'*32, generation=0 if self.job is None else 1),
            mode='model_preview' if self.job is None else 'live', manifest_descriptor=dict(model_sha256=MODEL),
            pose_ready=True, sequence=1))
    def start(self, value):
        self.calls.append(('start', deepcopy(value)))
        if value['model_sha256'] != MODEL:
            raise ValueError('Pinned native model differs')
        self.job = dict(job_id='1'*32, generation=1, finished=False, owner_cleanup_clean=None, cancel_requested=False)
        return self.snapshot()
    def cancel(self, value):
        if (not self.job or value['job_id'] != self.job['job_id'] or type(value['generation']) is not int
                or value['generation'] != self.job['generation'] or value['model_sha256'] != MODEL):
            raise ValueError('Exact native cancellation owner differs')
        self.calls.append(('cancel', deepcopy(value)))
        self.job['cancel_requested'] = True
        return self.snapshot()
    def manifest_bytes(self, identifier):
        if identifier not in (PREVIEW, LIVE):
            raise ValueError('Native binding differs')
        return b'native-model'
    def pose_envelope(self, identifier):
        self.manifest_bytes(identifier)
        return dict(binding_id=identifier, source_state_age_sec=5.25, simulation_sec=8., native_original=True)
    def asset_bytes(self, digest):
        if digest != 'f'*64:
            raise KeyError('Not native mesh')
        return b'native-mesh'
    def close(self):
        self.closed = True
        if self.job:
            self.job.update(finished=True, owner_cleanup_clean=self.clean_on_close)


class TaskTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root/'docs/reports').mkdir(parents=True)
        (self.root/'docs/reports/catalog.json').write_text('{"reports": []}')
        config = dict(domain=230, home_rad=dict(l=0., r=0.),
            joint_limits={name: dict(lower=-1., upper=1.) for name in ('l', 'r')})
        self.state = ConsoleState(config, clock=lambda:3.)
        self.state.started = 0.
        self.native = Native()
        with patch('mujoco_ros2_manipulation.operator_console.application.ConsoleViews', Views), patch.object(ConsoleApplication, 'watch'):
            self.app = ConsoleApplication(NS(root=self.root, id='fr5_ur5e'), self.state,
                NS(outbox=queue.Queue()), self.root/'console', grasp_application=self.native)
            self.app.monitor.join(1.)
        self.tasks = self.app.tasks
    def tearDown(self):
        self.app.children.clear()
        self.state.active = None
        self.app.bridge.outbox = queue.Queue()
        self.app.close()
        self.temp.cleanup()
    def select(self, task):
        return self.tasks.select(dict(task=task, selection_id=self.tasks.selection_id))
    def start(self):
        return self.tasks.start_grasp(dict(selection_id=self.tasks.selection_id, request_id='2'*32, model_sha256=MODEL))
    def cancel(self, **changes):
        value = dict(selection_id=self.tasks.selection_id, job_id='1'*32, generation=1, model_sha256=MODEL)
        value.update(changes)
        return self.tasks.cancel_grasp(value)

    def test_selection_never_starts_and_invalidates_normal_round_trip(self):
        before = self.app.views.host.identifier
        old = self.tasks.selection_id
        state = self.select('grasp')
        self.assertEqual(state['selected'], 'grasp')
        self.assertTrue(state['can_start'])
        self.assertNotEqual(old, state['selection_id'])
        self.assertEqual(self.native.calls, [])
        self.assertIsNone(self.app.views.host.identifier)
        self.select('motion')
        self.assertNotEqual(self.app.views.host.identifier, before)
        with self.assertRaises(ValueError):
            self.app.views.host.pose_envelope(before)

    def test_stale_selection_fields_models_and_tasks_cannot_dispatch(self):
        old = self.tasks.selection_id
        self.select('grasp')
        for payload in (
            dict(selection_id=old, request_id='2'*32, model_sha256=MODEL),
            dict(selection_id=self.tasks.selection_id, request_id='2'*32, model_sha256=MODEL, domain=999),
        ):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                self.tasks.start_grasp(payload)
        self.assertEqual(self.native.calls, [])
        with self.assertRaises(ValueError):
            self.tasks.select(dict(task='arbitrary/path', selection_id=self.tasks.selection_id))
        with self.assertRaises(ValueError):
            self.tasks.start_grasp(dict(selection_id=self.tasks.selection_id, request_id='2'*32, model_sha256='0'*64))
        self.assertIsNone(self.native.job)

    def test_normal_owned_running_stopping_external_graph_pending_queue_and_review_block_selection(self):
        cases = ('failed', 'running', 'starting', 'stopping', 'active', 'queued', 'preview', 'recording', 'graph', 'review', 'child')
        for case in cases:
            self.app.phase = 'idle'; self.app.error = None; self.state.active = None; self.state.pending_plan = None
            self.app.bridge.outbox = queue.Queue(); self.app.views.preview = False
            self.app.sync_recordings.current['status'] = 'idle'
            self.state.graph = {key:0 for key in self.state.graph}; self.app.children.clear()
            if case == 'failed': self.app.phase = 'failed'; self.app.error = 'Retained shutdown error'
            if case in ('running', 'starting', 'stopping'): self.app.phase = case
            if case == 'active': self.state.active = '3'*32
            if case == 'queued': self.app.bridge.outbox.put(('command', {}))
            if case == 'preview': self.app.views.preview = True
            if case == 'recording': self.app.sync_recordings.current['status'] = 'saving'
            if case == 'graph': self.state.graph['clock'] = 1
            if case == 'child': self.app.children['simulation'] = NS(poll=lambda:None)
            context = patch.object(self.state, 'review_snapshot', return_value={'state':'awaiting_approval'} if case == 'review' else None)
            with context, self.subTest(case=case), self.assertRaises(RuntimeError):
                self.select('grasp')
            self.assertEqual(self.tasks.selected, 'motion')
            self.assertEqual(self.native.calls, [])

    def test_native_active_and_unclean_finished_owner_block_change_and_new_start(self):
        self.select('grasp'); self.start()
        for finished, clean in ((False,None),(True,False),(True,None)):
            self.native.job.update(finished=finished, owner_cleanup_clean=clean)
            state = self.tasks.snapshot()
            self.assertFalse(state['can_select']); self.assertFalse(state['can_start'])
            with self.assertRaises(RuntimeError): self.select('motion')
        self.native.job.update(finished=True, owner_cleanup_clean=True)
        self.assertTrue(self.tasks.snapshot()['can_select'])
        self.select('motion')

    def test_cancel_routes_only_exact_current_native_owner_without_motion_request(self):
        self.select('grasp'); self.start()
        for change in (dict(job_id='9'*32),dict(generation=2),dict(generation=True),dict(model_sha256='a'*64),dict(selection_id='0'*32)):
            with self.subTest(change=change), self.assertRaises(ValueError): self.cancel(**change)
        self.assertFalse(self.native.job['cancel_requested'])
        self.cancel()
        self.assertTrue(self.native.job['cancel_requested'])
        self.assertEqual(self.native.calls[-1][0], 'cancel')
        self.assertTrue(self.app.bridge.outbox.empty())
        self.assertFalse(self.tasks.snapshot()['can_select'])

    def test_all_normal_dispatch_and_preview_admission_remain_blocked(self):
        self.select('grasp')
        operations = [lambda:self.app.start(dict(native_gui=False, planning_case='none')),
            lambda:self.app.submit({}),lambda:self.app.create_plan({}),lambda:self.app.create_pose_plan({}),
            lambda:self.app.execute_plan({}),lambda:self.app.discard_plan('3'*32),
            lambda:self.app.cancel('3'*32),lambda:self.app.stop_async(),lambda:self.app.set_display('web3d'),
            lambda:ConsoleViews._idle(NS(app=self.app))]
        for operation in operations:
            with self.subTest(operation=operation), self.assertRaisesRegex(RuntimeError, '집기'):
                operation()
        self.assertIn('집기', self.app.demos.reason())
        self.assertTrue(self.app.bridge.outbox.empty())
        state = self.app.snapshot()
        self.assertFalse(any(state[key] for key in ('can_submit','can_plan','can_pose_plan','can_approve')))
        self.assertEqual(state['view']['task_selected'], 'grasp')
        self.assertEqual(state['task']['selected'], 'grasp')
        self.assertEqual(state['view']['binding_id'], PREVIEW)

    def test_shared_view_requires_selected_binding_and_preserves_native_age_and_source(self):
        with self.assertRaises(ValueError): self.tasks.manifest_bytes(PREVIEW)
        self.select('grasp')
        self.assertEqual(self.tasks.manifest_bytes(PREVIEW), b'native-model')
        self.start()
        with self.assertRaises(ValueError): self.tasks.manifest_bytes(PREVIEW)
        with self.assertRaises(ValueError): self.tasks.pose_envelope('a'*64)
        before = self.native.pose_envelope(LIVE)
        self.assertEqual(self.tasks.pose_envelope(LIVE), before)
        self.assertEqual(before['source_state_age_sec'], 5.25)
        self.assertEqual(self.app.snapshot()['view']['binding_id'], LIVE)
        self.native.job.update(finished=True,owner_cleanup_clean=True)
        self.select('motion')
        with self.assertRaises(ValueError): self.tasks.pose_envelope(LIVE)

    def test_close_waits_owned_native_cleanup_and_reports_unclean_finished_job(self):
        self.select('grasp'); self.start(); self.native.clean_on_close = False
        self.assertFalse(self.app.close())
        self.assertTrue(self.native.closed)
        self.assertFalse(self.native.job['owner_cleanup_clean'])
        self.native.clean_on_close = True

    def test_normal_transport_epoch_is_rechecked_after_slow_work_and_never_leaks(self):
        original = self.tasks.selection_id
        with self.tasks.motion_request([original]):
            self.app.require_motion_task()
            self.tasks.selection_id = '5' * 32
            with self.assertRaisesRegex(ValueError, '작업 선택'):
                self.app.submit({})
        self.app.require_motion_task()  # A different trusted call has no old HTTP context.
        for headers in ([], ['x'], [original, original], [original]):
            with self.subTest(headers=headers), self.assertRaises(ValueError):
                with self.tasks.motion_request(headers):
                    self.fail('stale or ambiguous transport was admitted')
        with self.tasks.motion_request([self.tasks.selection_id]):
            self.app.require_motion_task()
        self.assertTrue(self.app.bridge.outbox.empty())

    def test_retained_request_limit_blocks_new_job_without_trapping_task_selection(self):
        self.select('grasp')
        self.native.requests = {str(index): None for index in range(8)}
        state = self.tasks.snapshot()
        self.assertFalse(state['can_start'])
        self.assertTrue(state['can_select'])
        self.assertIn('보존 상한', state['reason'])
        self.select('motion')

    def test_unconfigured_default_is_available_only_for_motion(self):
        self.tasks.grasp = None
        value = self.tasks.snapshot()
        self.assertEqual(value['selected'], 'motion')
        self.assertFalse(value['can_select']); self.assertFalse(value['options'][1]['available'])
        with self.assertRaises(ValueError): self.select('grasp')


class TaskHTTPTests(unittest.TestCase):
    def setUp(self):
        TaskTests.setUp(self)
        self.server = ConsoleHTTPServer(('127.0.0.1',0),Handler)
        self.server.application, self.server.token = self.app, 'test-task-token'
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()
        self.port = self.server.server_address[1]
    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(2)
        TaskTests.tearDown(self)
    def request(self, path, value=None, *, token=True, origin=True, selection=None):
        connection = HTTPConnection('127.0.0.1',self.port,timeout=3)
        headers = {}
        raw = None
        if value is not None:
            raw = json.dumps(value)
            headers['Content-Type']='application/json'
            headers['X-Workspace-Selection']=self.tasks.selection_id if selection is None else selection
            if token: headers['X-Operator-Token']='test-task-token'
            if origin: headers['Origin']=f'http://127.0.0.1:{self.port}'
        connection.request('GET' if value is None else 'POST',path,body=raw,headers=headers)
        response=connection.getresponse(); body=response.read(); status=response.status
        connection.close()
        return status,body
    def test_old_normal_http_epoch_after_round_trip_cannot_start_a_new_world(self):
        original = self.tasks.selection_id
        self.tasks.select(dict(task='grasp',selection_id=original))
        self.tasks.select(dict(task='motion',selection_id=self.tasks.selection_id))
        with patch.object(self.app, 'start_worker') as spawn:
            status,_ = self.request('/api/start',dict(native_gui=False,planning_case='none'),selection=original)
            self.assertEqual(status,400)
            self.assertEqual(self.app.phase,'idle')
            spawn.assert_not_called()
        self.assertTrue(self.app.bridge.outbox.empty())

    def test_same_origin_routes_share_native_scene_and_keep_csrf_and_stale_owner_checks(self):
        status,raw=self.request('/api/bootstrap')
        self.assertEqual(status,200)
        self.assertEqual(json.loads(raw)['task']['selected'],'motion')
        payload=dict(task='grasp',selection_id=self.tasks.selection_id)
        for options in (dict(token=False),dict(origin=False)):
            self.assertEqual(self.request('/api/task/select',payload,**options)[0],403)
        self.assertEqual(self.tasks.selected,'motion')
        self.assertEqual(self.request('/api/task/select',payload)[0],202)
        self.assertEqual(self.request('/api/view/manifest?binding_id='+PREVIEW),(200,b'native-model'))
        self.assertEqual(self.request('/api/view/assets/'+'f'*64),(200,b'native-mesh'))
        self.assertEqual(self.request('/api/view/assets/'+'e'*64)[0],400)
        self.assertEqual(self.request('/api/start',dict(native_gui=False,planning_case='none'))[0],409)
        start=dict(selection_id=self.tasks.selection_id,request_id='2'*32,model_sha256=MODEL)
        self.assertEqual(self.request('/api/task/grasp/start',start)[0],202)
        self.assertEqual(self.request('/api/view/manifest?binding_id='+PREVIEW)[0],400)
        self.assertEqual(self.request('/api/view/pose?binding_id='+LIVE)[0],200)
        stale=dict(selection_id=self.tasks.selection_id,job_id='9'*32,generation=1,model_sha256=MODEL)
        self.assertEqual(self.request('/api/task/grasp/cancel',stale)[0],400)
        stale['job_id']='1'*32
        self.assertEqual(self.request('/api/task/grasp/cancel',stale)[0],202)
        self.assertTrue(self.native.job['cancel_requested'])
