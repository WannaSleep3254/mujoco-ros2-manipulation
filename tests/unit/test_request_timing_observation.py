"""Original bounded timing observations, with no ROS or real control processes."""
import ast
from copy import deepcopy
from importlib.util import find_spec
import json
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation import timing
from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.workcells.motion_request import command
from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace
from tests.unit import test_console_integration as console_fixtures
from tests.unit import test_motion_review_server as recorder_fixtures
from tests.unit.test_view_http import handler


class TimingMetadataTests(unittest.TestCase):
    def test_exact_ns_and_invalid_optional_samples(self):
        value = 2**60 + 123
        self.assertEqual(timing.decimal_ns(value), str(value))
        for invalid in (True, -1, 1.5, str(value), 2**63, None):
            self.assertIsNone(timing.decimal_ns(invalid))
        with patch.object(timing.time, 'monotonic_ns', side_effect=OSError('clock unavailable')):
            self.assertIsNone(timing.sample_ns())

    def test_startup_context_is_detached_self_observation_and_failure_is_optional(self):
        context = timing.capture_clock_context()
        self.assertEqual(context['proof_status'], 'self_observed')
        self.assertRegex(context['context_id'], r'^[0-9a-f]{64}$')
        self.assertEqual(context['clock_source'], 'CLOCK_MONOTONIC')
        copied = timing.trace_clock_context(context)
        copied['process_pid'] = -1
        self.assertNotEqual(context, copied)
        with patch.object(timing.Path, 'open', side_effect=OSError('unavailable')):
            unavailable = timing.capture_clock_context()
        self.assertEqual(unavailable['proof_status'], 'unavailable')
        self.assertIsNone(unavailable['context_id'])
        for invalid in (None, [], dict(context, extra=True), object()):
            self.assertIsNone(timing.trace_clock_context(invalid))

    def test_failed_original_mark_cannot_be_replaced_and_metadata_is_bounded(self):
        value = timing.console_timing(dict(context_id="c"*64), session_id='a'*32, generation=1,
            selection_id='d'*32, expected_session_id='b'*32, kind='approve', plan_request_id='e'*32,
            http_received_ns=None, submit_enter_ns='1')
        record = dict(console_timing=value)
        timing.mark_timing(record, 'http_handler_enter_ns', '999')
        self.assertIsNone(value['stamps']['http_handler_enter_ns'])
        for name in timing.STAMP_FIELDS:
            timing.mark_timing(record, name, None)
            timing.mark_timing(record, name, '999')
        self.assertEqual(len(value['observed_fields']), len(timing.STAMP_FIELDS))
        self.assertLessEqual(len(value['missing_reasons']), 8)
        self.assertLess(len(json.dumps(value).encode()), 2048)
        timing.mark_timing(dict(console_timing=[]), 'reserved_ns', '1')
        timing.mark_timing({}, 'reserved_ns', '1')
        timing.mark_timing(record, 'unrecognized', '1')
        self.assertNotIn('unrecognized', value['stamps'])

    def test_trace_events_use_one_original_sample_and_keep_old_metrics(self):
        origin = 2**60 + 17
        clock = Mock(side_effect=[origin + 1, origin + 123, origin + 456])
        trace = RequestTrace('a'*32, received_ns=origin, clock=clock)
        event = trace.event('first_motion', monotonic_ns='forged', simulation_sec=50.)
        self.assertEqual(clock.call_count, 2)
        self.assertEqual(trace.data['clock']['origin_ns'], origin)
        self.assertEqual(trace.data['clock']['origin_monotonic_ns'], str(origin))
        self.assertEqual(event['monotonic_ns'], str(origin + 123))
        self.assertEqual(event['wall_sec'], 123/1e9)
        trace.finish('succeeded')
        self.assertEqual(trace.data['metrics']['received_to_first_motion_wall_sec'], 123/1e9)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'events.jsonl'
            trace.append_events(path)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(rows[1]['event']['monotonic_ns'], str(origin + 123))
        self.assertIsNone(trace.data['clock']['context'])


class ConsoleTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(find_spec('mujoco_ros2_manipulation.operator_console.ros_adapter').origin)
        tree = ast.parse(source.read_text())
        bridge = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'ConsoleBridge')
        methods = [deepcopy(node) for node in bridge.body if isinstance(node, ast.FunctionDef)
                   and node.name in ('send', 'status_message')]
        namespace = dict(queue=queue, json=json, String=lambda **kwargs:NS(**kwargs))
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), namespace)
        cls.send = staticmethod(namespace['send'])

    def setUp(self):
        fixture = console_fixtures.ConsoleHookTests(methodName='runTest')
        fixture.setUp()
        self.addCleanup(fixture.tearDown)
        self.app, self.state = fixture.app, fixture.state
        self.payload = command(dict(l=.2, r=.2))
        self.bridge = NS(state=self.state, outbox=self.app.bridge.outbox,
            commands=NS(publish=Mock()), plans=NS(publish=Mock()), pose_plans=NS(publish=Mock()),
            approvals=NS(publish=Mock()), cancels=NS(publish=Mock()))

    def record(self):
        return self.state.history[self.payload['request_id']]

    def submit(self):
        with patch('mujoco_ros2_manipulation.operator_console.application.sample_ns', side_effect=['101', '102', '103']):
            return self.app.submit(self.payload, http_received_ns='100')

    def test_successful_marks_keep_original_outbox_pair_and_dispatch_once(self):
        self.submit()
        row = self.record()['console_timing']
        self.assertEqual({key: row['stamps'][key] for key in timing.STAMP_FIELDS[:4]}, dict(
            http_handler_enter_ns='100', console_submit_enter_ns='101', reserved_ns='102', enqueued_ns='103'))
        self.assertEqual(self.bridge.outbox.queue[0], ('command', self.payload))
        with patch.object(timing, 'capture_clock_context', side_effect=AssertionError('No hot-path context I/O')):
            with patch.object(timing, 'sample_ns', side_effect=['104', '105']):
                self.send(self.bridge)
            self.send(self.bridge)
        self.bridge.commands.publish.assert_called_once()
        self.assertEqual(row['stamps']['publish_begin_ns'], '104')
        self.assertEqual(row['stamps']['publish_returned_ns'], '105')
        self.assertEqual(row['diagnostic_status'], 'available')
        self.assertNotIn('console_timing', json.loads(self.bridge.commands.publish.call_args.args[0].data))

    def test_concurrent_duplicate_and_changed_duplicate_preserve_first_marks(self):
        barrier, failures = threading.Barrier(3), []
        def submit():
            barrier.wait()
            try:
                self.app.submit(self.payload, http_received_ns='100')
            except Exception as error:
                failures.append(error)
        threads = [threading.Thread(target=submit) for _ in range(2)]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(timeout=5)
            self.assertFalse(thread.is_alive())
        self.assertFalse(failures)
        self.assertEqual(self.bridge.outbox.qsize(), 1)
        original = deepcopy(self.record()['console_timing'])
        self.app.submit(self.payload, http_received_ns='999')
        altered = deepcopy(self.payload)
        altered['target_rad']['l'] = .3
        with self.assertRaises(ValueError):
            self.app.submit(altered, http_received_ns='999')
        with self.assertRaises(ValueError):
            self.app.create_plan(self.payload, http_received_ns='999')
        self.assertEqual(self.record()['console_timing'], original)
        self.assertEqual(self.bridge.outbox.qsize(), 1)

    def test_queue_full_has_no_enqueue_or_publish_proof_and_remains_bounded(self):
        self.app.bridge.outbox = queue.Queue(maxsize=1)
        self.app.bridge.outbox.put(('cancel', {'request_id':'f'*32}))
        with self.assertRaises(RuntimeError):
            self.submit()
        timing_row = self.record()['console_timing']
        self.assertEqual(self.record()['status'], 'rejected')
        self.assertIsNone(self.state.active)
        self.assertIsNone(timing_row['stamps']['enqueued_ns'])
        self.assertIsNone(timing_row['stamps']['publish_begin_ns'])
        for index in range(45):
            payload = command(dict(l=.2, r=.2))
            with self.assertRaises(RuntimeError):
                self.app.submit(payload)
        self.assertEqual(len(self.state.history), 40)
        self.assertEqual(len(self.state.ui_ids), 40)

    def test_only_correlated_ros_receipts_mark_first_status_and_ignore_injected_local_namespace(self):
        self.submit()
        identifier = self.payload['request_id']
        queued = dict(session_id='a'*32, request_id=identifier, status='queued', console_timing={'forged':True})
        self.state.status(queued, received_ns='110')
        self.state.status(dict(queued, status='running'), received_ns='120')
        self.state.status(dict(queued, status='succeeded'), received_ns='130')
        self.state.status(dict(queued, status='succeeded'), received_ns='140')
        local = self.record()['console_timing']
        self.assertEqual(local['stamps']['first_status_received_ns'], '110')
        self.assertEqual(local['stamps']['first_terminal_received_ns'], '130')
        self.assertNotIn('forged', local)
        self.assertNotIn('console_timing', self.state.server)
        self.state.status(dict(queued, request_id='e'*32), received_ns='150')
        self.assertNotIn('console_timing', self.state.history['e'*32])

    def test_failed_first_receipt_and_synthetic_rejection_never_manufacture_later_original_time(self):
        self.submit()
        identifier = self.payload['request_id']
        queued = dict(session_id='a'*32, request_id=identifier, status='queued')
        self.state.status(queued, received_ns=None)
        self.state.status(queued, received_ns='999')
        self.assertIsNone(self.record()['console_timing']['stamps']['first_status_received_ns'])
        # A local rejection has no source receipt argument.
        self.state.status(dict(queued, status='rejected'))
        local = self.record()['console_timing']
        self.assertNotIn('first_terminal_received_ns', local['observed_fields'])

    def test_server_replacement_or_terminal_record_keeps_old_session_and_drops_queued_publish(self):
        self.submit()
        self.state.status(dict(session_id='b'*32, request_id=None, status='ready'), received_ns='999')
        self.send(self.bridge)
        self.bridge.commands.publish.assert_not_called()
        local = self.record()['console_timing']
        self.assertEqual(local['expected_motion_session_id'], 'a'*32)
        self.assertIsNone(local['stamps']['publish_begin_ns'])
        self.assertIsNone(local['stamps']['first_status_received_ns'])

    def test_optional_clock_failure_does_not_block_command_or_cancel_and_publish_error_is_visible(self):
        with patch('mujoco_ros2_manipulation.operator_console.application.sample_ns', return_value=None):
            self.app.submit(self.payload)
            self.app.cancel(self.payload['request_id'])
        with patch.object(timing, 'sample_ns', return_value=None):
            self.send(self.bridge)
            self.send(self.bridge)
        self.bridge.commands.publish.assert_called_once()
        self.bridge.cancels.publish.assert_called_once()
        local = self.record()['console_timing']
        self.assertIsNone(local['stamps']['publish_begin_ns'])
        self.assertIsNone(local['stamps']['cancel_publish_returned_ns'])

    def test_cancel_enqueue_observation_precedes_competing_consumer_publication(self):
        self.submit()
        self.bridge.outbox.get_nowait()  # Remove the already tested normal queue item.
        consumed = threading.Event()
        failures, threads = [], []
        original_put, original_get = self.bridge.outbox.put_nowait, self.bridge.outbox.get_nowait
        counter = iter(str(value) for value in range(200, 210))
        def consume():
            try:
                self.send(self.bridge)
            except Exception as error:
                failures.append(error)
        def get():
            item = original_get()
            consumed.set()
            return item
        def put(item):
            original_put(item)
            thread = threading.Thread(target=consume)
            threads.append(thread)
            thread.start()
            self.assertTrue(consumed.wait(timeout=2))
        with patch.object(self.bridge.outbox, 'put_nowait', side_effect=put), \
             patch.object(self.bridge.outbox, 'get_nowait', side_effect=get), \
             patch('mujoco_ros2_manipulation.operator_console.application.sample_ns', side_effect=lambda:next(counter)), \
             patch.object(timing, 'sample_ns', side_effect=lambda:next(counter)):
            self.app.cancel(self.payload['request_id'], http_received_ns='199')
            for thread in threads:
                thread.join(timeout=2)
                self.assertFalse(thread.is_alive())
        self.assertFalse(failures)
        self.bridge.cancels.publish.assert_called_once()
        stamps = self.record()['console_timing']['stamps']
        self.assertLess(int(stamps['cancel_enqueued_ns']), int(stamps['cancel_publish_begin_ns']))

    def test_publisher_exception_is_not_swallowed_or_retried_by_diagnostics(self):
        self.submit()
        self.bridge.commands.publish.side_effect = RuntimeError('transport failure')
        with patch.object(timing, 'sample_ns', return_value='104'):
            with self.assertRaisesRegex(RuntimeError, 'transport failure'):
                self.send(self.bridge)
        self.send(self.bridge)
        self.bridge.commands.publish.assert_called_once()
        local = self.record()['console_timing']
        self.assertEqual(local['stamps']['publish_begin_ns'], '104')
        self.assertIsNone(local['stamps']['publish_returned_ns'])

    def test_http_trusted_entry_time_is_not_taken_from_payload_or_header(self):
        value, response, _ = handler('/api/command', method='POST', payload=self.payload)
        value.server.application = self.app
        value.headers['X-Workspace-Selection'] = self.app.tasks.selection_id
        value.headers['X-Monotonic-Ns'] = 'forged'
        with patch('mujoco_ros2_manipulation.operator_console.http_server.sample_ns', return_value='100'):
            value.do_POST()
        self.assertEqual(response.call_args.args[1], 202)
        self.assertEqual(self.record()['console_timing']['stamps']['http_handler_enter_ns'], '100')
        self.assertEqual(self.bridge.outbox.queue[0][1], self.payload)

    def test_legacy_trace_without_new_fields_remains_readable_and_bytes_unchanged(self):
        self.submit()
        identifier = self.payload['request_id']
        path = self.app.cell.root/'runtime'/(identifier+'.json')
        path.parent.mkdir()
        original = json.dumps(dict(request_id=identifier, clock=dict(origin_ns=123), events=[], spans=[], metrics={})).encode()
        path.write_bytes(original)
        self.record()['trace_path'] = str(path)
        detail = self.app.request_details(identifier)
        self.assertEqual(detail['clock'], dict(origin_ns=123))
        self.assertEqual(path.read_bytes(), original)
        self.assertIsNone(detail['clock'].get('origin_monotonic_ns'))


class RecorderSourceTimingTests(unittest.TestCase):
    def setUp(self):
        fixture = recorder_fixtures.ReviewServerTests(methodName='runTest')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.fixture, self.node = fixture, fixture.node
        self.context = timing.capture_clock_context()
        self.node.timing_clock_context = self.context

    def test_normal_source_binds_original_clock_and_session_and_duplicate_keeps_raw_bytes(self):
        payload = command(dict(left=.1, right=.1))
        self.fixture.send(payload, 'command')
        path = self.node.directory/(payload['request_id']+'.json')
        raw = path.read_bytes()
        trace = json.loads(raw)
        self.assertEqual(trace['session_id'], self.node.session_id)
        self.assertEqual(trace['clock']['context'], self.context)
        self.assertEqual(trace['clock']['context_id'], self.context['context_id'])
        self.assertEqual(trace['clock']['origin_monotonic_ns'], str(trace['clock']['origin_ns']))
        self.assertGreaterEqual(int(trace['events'][0]['monotonic_ns']), trace['clock']['origin_ns'])
        self.fixture.send(payload, 'command')
        self.assertEqual(path.read_bytes(), raw)
        self.node.dispatch.assert_not_called()

    def test_plan_approval_and_pose_rejection_retain_distinct_source_identity(self):
        plan, plan_trace, approval = self.fixture.make_review()
        self.fixture.send(approval, 'approve')
        approval_trace = json.loads((self.node.directory/(approval['request_id']+'.json')).read_bytes())
        self.assertEqual(approval_trace['session_id'], self.node.session_id)
        self.assertEqual(approval_trace['clock']['context_id'], self.context['context_id'])
        self.assertNotEqual(plan_trace['request_id'], approval_trace['request_id'])
        self.assertEqual(approval_trace['approval']['plan_request_id'], plan['request_id'])
        self.assertEqual(approval_trace['dispatched_goals'], 0)
        self.node.pending = None
        for instance in self.node.cell.instances:
            instance.prefix, instance.group = instance.id + '_', instance.id + '_arm'
            instance.profile = NS(label=instance.id, ros=dict(tip_frame='wrist3_link'))
        malformed_pose = dict(request_id='f'*32)
        self.node.receive_pose_plan(NS(data=json.dumps(malformed_pose)))
        pose_trace = json.loads((self.node.directory/('f'*32+'.json')).read_bytes())
        self.assertEqual(pose_trace['status'], 'rejected')
        self.assertEqual(pose_trace['clock']['context_id'], self.context['context_id'])
        self.assertEqual(pose_trace['session_id'], self.node.session_id)
        self.node.dispatch.assert_not_called()

    def test_invalid_optional_context_keeps_existing_admission_and_source_times(self):
        self.node.timing_clock_context = object()
        payload = command(dict(left=.1, right=.1))
        self.fixture.send(payload, 'command')
        trace = json.loads((self.node.directory/(payload['request_id']+'.json')).read_bytes())
        self.assertEqual(trace['status'], 'queued')
        self.assertIsNone(trace['clock']['context'])
        self.assertIsNone(trace['clock']['context_id'])
        self.assertEqual(trace['clock']['origin_monotonic_ns'], str(trace['clock']['origin_ns']))


if __name__ == '__main__':
    unittest.main()
