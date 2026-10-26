"""Actual application hooks; no ports, ROS, MuJoCo or real child process."""
import ast
from copy import deepcopy
from importlib.util import find_spec
import json
from pathlib import Path
import queue
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

ENGINE_MODULES_BEFORE = {name for name in ('rclpy','mujoco') if name in sys.modules}
from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.state import ConsoleState
from mujoco_ros2_manipulation.operator_console.console_views import ConsoleViews
from mujoco_ros2_manipulation.workcells.motion_request import command

APPLICATION = 'mujoco_ros2_manipulation.operator_console.application'
JOBS = 'mujoco_ros2_manipulation.operator_console.demo_jobs'


def state_fixture():
    now = [3.]
    config = dict(home_rad=dict(l=0., r=0.), joint_limits={n:dict(lower=-1., upper=1.) for n in ('l','r')})
    state = ConsoleState(config, clock=lambda: now[0])
    state.started = 0
    state.graph = dict(clock=1, joints=1, status=1, commands=1, guards=1)
    state.identity = True
    state.guard = dict(latched=False, phase='ready')
    state.joints(['l','r'], [0.,0.], [0.,0.], source_stamp=1.)
    state.tick(1.)
    state.status(dict(session_id='a'*32, request_id=None, status='ready'))
    return state


class ConsoleHookTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.state = state_fixture()
        with patch.object(ConsoleApplication, 'watch', return_value=None):
            self.app = ConsoleApplication(NS(root=self.root), self.state, NS(outbox=queue.Queue()), self.root/'console', render=False)
            self.app.monitor.join(timeout=1)
    def tearDown(self):
        self.app.children.clear()
        self.app.demos.process = None
        self.app.demos.stopping = False
        self.app.views.host = None
        self.app.close()
        self.temp.cleanup()

    def test_render_false_minimal_cell_import_and_lifecycle_remain_headless(self):
        self.assertEqual(self.app.display_mode, 'web3d')
        self.assertFalse(self.app.image_render_enabled)
        self.assertIsNone(self.app.views.host)
        self.assertIsNone(self.app.views.base)
        snapshot = self.app.snapshot()
        self.assertEqual(snapshot['lifecycle']['generation'], 0)
        self.assertFalse(snapshot['view']['enabled'])
        self.assertIsNone(snapshot['view']['config_source_id'])
        self.assertEqual({name for name in ('rclpy','mujoco') if name in sys.modules}, ENGINE_MODULES_BEFORE)

    def test_preview_guards_are_checked_under_both_application_and_state_locks(self):
        app = self.app
        class Probe:
            @property
            def preview(self):
                if not app.lock._is_owned() or not app.state.lock._is_owned():
                    raise AssertionError('Preview admission was checked without both locks')
                return True
        previous = app.views
        try:
            app.views = Probe()
            with self.assertRaisesRegex(RuntimeError, '미리보기'):
                app.start(dict(native_gui=False, planning_case='none'))
            with self.assertRaisesRegex(RuntimeError, '미리보기'):
                app.submit(command(dict(l=.2,r=.2)))
            self.assertIn('미리보기', app.demos.reason())
        finally:
            app.views = previous
        self.assertTrue(app.bridge.outbox.empty())

    def test_preview_snapshot_blocks_submit_without_retargeting_the_configured_cell(self):
        self.app.views.mode = 'model_preview'
        snapshot = self.app.snapshot()
        self.assertFalse(snapshot['can_submit'])
        self.assertIn('미리보기', snapshot['reason'])
        self.assertEqual(snapshot['positions_rad'], dict(l=0.,r=0.))

    def test_display_switch_only_stops_owned_renderer_and_advances_its_generation(self):
        self.app.render_enabled = True
        commands, simulation, renderer = object(), object(), object()
        self.app.children = dict(commands=commands, simulation=simulation, renderer=renderer)
        with patch(JOBS+'.stop_process', return_value=False) as stop:
            self.assertEqual(self.app.set_display('image'), dict(display_mode='image'))
            self.assertTrue(self.app.image_render_enabled)
            self.assertEqual(self.app.generation, 1)
            self.assertEqual(self.app.children, dict(commands=commands, simulation=simulation))
            self.assertEqual(stop.call_args.args, (renderer,))
            second = object(); self.app.children['renderer'] = second
            self.app.set_display('web3d')
            self.assertFalse(self.app.image_render_enabled)
            self.assertEqual(self.app.generation, 2)
            self.assertEqual([call.args for call in stop.call_args_list], [(renderer,), (second,)])
            self.assertEqual(self.app.children, dict(commands=commands, simulation=simulation))

    def test_failed_renderer_stop_retains_ownership_and_previous_display_mode(self):
        self.app.render_enabled = self.app.image_render_enabled = True
        self.app.display_mode = 'image'
        renderer = object(); self.app.children['renderer'] = renderer
        with patch(JOBS+'.stop_process', side_effect=RuntimeError('cleanup failed')):
            with self.assertRaises(RuntimeError):
                self.app.set_display('web3d')
        self.assertIs(self.app.children['renderer'], renderer)
        self.assertEqual(self.app.display_mode, 'image')
        self.assertTrue(self.app.image_render_enabled)

    def test_display_mode_is_strict_and_active_or_preview_changes_are_refused(self):
        for mode in (None, 'remote', {}, True):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                self.app.set_display(mode)
        with self.assertRaises(RuntimeError): self.app.set_display('image')
        self.app.render_enabled = True
        for phase in ('starting','stopping','demo'):
            self.app.phase = phase
            with self.assertRaises(RuntimeError): self.app.set_display('image')
        self.app.phase = 'idle'
        self.state.active = 'f'*32
        with self.assertRaises(RuntimeError): self.app.set_display('image')
        self.state.active = None
        self.app.views.mode = 'plan_preview'
        with self.assertRaises(RuntimeError): self.app.set_display('image')

    def test_view_tick_failure_and_failed_logging_cannot_end_supervision(self):
        private_path = str(self.root/'private-scene.xml')
        self.app.views.tick = Mock(side_effect=RuntimeError(private_path))
        self.app.views.host = NS(invalidate=Mock())
        with patch.object(self.app, 'audit', side_effect=OSError('disk full')):
            self.app.update_views()
        self.app.views.host.invalidate.assert_called_once()
        public = json.dumps(self.app.views.error)
        self.assertNotIn(private_path, public)
        self.assertEqual(self.app.views.error['code'], 'view_update_failed')

    def test_observation_reset_clears_both_identity_rpc_epochs(self):
        self.state.identity_requested_at = 1.
        self.state.identity_checked_at = 2.
        self.app.reset_observation()
        self.assertIsNone(self.state.identity_requested_at)
        self.assertIsNone(self.state.identity_checked_at)

    def test_watch_feeds_views_at_100ms_but_keeps_supervision_at_500ms(self):
        class Closing:
            def __init__(self): self.calls = 0; self.waits = []
            def wait(self, duration): self.waits.append(duration); self.calls += 1; return self.calls > 10
            def is_set(self): return self.calls > 10
            def set(self): self.calls = 11
        previous = self.app.closing
        self.app.closing = Closing()
        self.app.views.tick = Mock()
        self.app.demos.tick = Mock()
        try:
            with patch(APPLICATION+'.time.monotonic', side_effect=[index/10 for index in range(10)]), \
                 patch.object(self.app,'spawn') as spawn:
                self.app.watch()
            self.assertEqual(self.app.views.tick.call_count, 10)
            self.assertEqual(self.app.demos.tick.call_count, 2)
            self.assertEqual(self.app.closing.waits, [.1]*11)
            spawn.assert_not_called()
        finally:
            self.app.closing = previous


class Future:
    def __init__(self, result=None): self.value=result; self.complete=False; self.cancelled=False
    def done(self): return self.complete
    def result(self): return self.value
    def cancel(self): self.cancelled=True


class DispatchReservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Run the actual publisher method with pure collaborators; importing the
        # ROS adapter itself would unnecessarily require rclpy and message types.
        source = Path(find_spec('mujoco_ros2_manipulation.operator_console.ros_adapter').origin)
        tree = ast.parse(source.read_text())
        bridge = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ConsoleBridge')
        method = deepcopy(next(n for n in bridge.body if isinstance(n,ast.FunctionDef) and n.name=='send'))
        namespace = dict(queue=queue,json=json,String=lambda **kwargs:NS(**kwargs))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        cls.send = staticmethod(namespace['send'])

    def setUp(self):
        self.state = state_fixture()
        self.payload = command(dict(l=.2,r=.2))
        self.assertTrue(self.state.reserve(self.payload))
        self.bridge = NS(state=self.state,outbox=queue.Queue(),commands=NS(publish=Mock()),cancels=NS(publish=Mock()))
        self.bridge.outbox.put(('command',self.payload))

    def test_current_local_reservation_publishes_once_under_state_lock(self):
        def published(message):
            self.assertTrue(self.state.lock._is_owned())
            self.assertEqual(json.loads(message.data),self.payload)
        self.bridge.commands.publish.side_effect = published
        self.send(self.bridge)
        self.send(self.bridge)
        self.bridge.commands.publish.assert_called_once()
        self.assertEqual(self.state.active,self.payload['request_id'])
        self.assertEqual(self.state.history[self.payload['request_id']]['status'],'submitting')

    def test_server_replacement_allows_preview_but_cannot_publish_old_queue(self):
        self.state.status(dict(session_id='b'*32,request_id=None,status='ready'))
        identifier = self.payload['request_id']
        self.assertIsNone(self.state.active)
        self.assertEqual(self.state.history[identifier]['status'],'unknown')
        self.assertIsNone(self.state.reason())
        # This is the real preview admission boundary, without loading models.
        view = ConsoleViews.__new__(ConsoleViews)
        view.app = NS(state=self.state,closing=threading.Event(),demos=NS(busy=False),phase='idle',display_mode='web3d')
        view.enabled,view.mode = True,'live'
        view._idle()
        view.mode = 'model_preview'
        self.assertTrue(view.preview)
        before = deepcopy(self.state.history[identifier])
        self.send(self.bridge)
        self.bridge.commands.publish.assert_not_called()
        self.assertEqual(self.state.history[identifier],before)
        self.assertTrue(self.bridge.outbox.empty())

    def test_cancelled_and_other_terminal_outcomes_are_not_rejected_or_republished(self):
        for status in ('cancelled','succeeded','failed','rejected'):
            with self.subTest(status=status):
                self.setUp()
                identifier = self.payload['request_id']
                self.state.status(dict(session_id='a'*32,request_id=identifier,status=status,
                    stage='terminal',metrics={'witness':1},error={'message':'retained'}))
                self.state.admission_suspended = True
                before = deepcopy(self.state.history[identifier])
                self.send(self.bridge)
                self.bridge.commands.publish.assert_not_called()
                self.assertEqual(self.state.history[identifier],before)
                self.assertTrue(self.state.admission_suspended)

    def test_only_an_owned_unacknowledged_exact_reservation_can_publish(self):
        for change in ('other_active','unowned','changed_command','queued','running','session_bound'):
            with self.subTest(change=change):
                self.setUp()
                identifier = self.payload['request_id']
                record = self.state.history[identifier]
                if change=='other_active': self.state.active='c'*32
                elif change=='unowned': self.state.ui_ids.clear()
                elif change=='changed_command': record['command']['target_rad']['l']=.3
                elif change in ('queued','running'):
                    self.state.status(dict(session_id='a'*32,request_id=identifier,status=change,stage='received'))
                else: record['session_id']='a'*32
                before = deepcopy(self.state.history[identifier])
                self.send(self.bridge)
                self.bridge.commands.publish.assert_not_called()
                self.assertEqual(self.state.history[identifier],before)

    def test_manual_suspension_still_rejects_a_current_unpublished_reservation(self):
        self.state.admission_suspended = True
        self.send(self.bridge)
        self.bridge.commands.publish.assert_not_called()
        self.assertEqual(self.state.history[self.payload['request_id']]['status'],'rejected')
        self.assertEqual(self.state.history[self.payload['request_id']]['stage'],'pre_dispatch')
        self.assertTrue(self.state.admission_suspended)


class IdentityRpcTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(find_spec('mujoco_ros2_manipulation.operator_console.ros_adapter').origin)
        tree = ast.parse(source.read_text())
        definition = deepcopy(next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='ConsoleBridge'))
        definition.bases = [ast.Name(id='object',ctx=ast.Load())]
        module = ast.fix_missing_locations(ast.Module(body=[definition],type_ignores=[]))
        cls.clock = [100.]
        namespace = dict(ET=ET,time=NS(monotonic=lambda:cls.clock[0]),STATUS_TOPIC='/status',
            GetParameters=NS(Request=lambda **kwargs:kwargs))
        exec(compile(module,str(source),'exec'),namespace)
        cls.Bridge = namespace['ConsoleBridge']

    def setUp(self):
        self.clock[0] = 100.
        self.state = state_fixture()
        self.state.identity = False
        self.graph = dict(clock=1,joints=1)
        self.future = Future()
        self.bridge = self.Bridge.__new__(self.Bridge)
        self.bridge.state, self.bridge.cell = self.state, NS(joints=['l','r'],output=Path('/trusted/base'))
        self.bridge.identity_future = self.bridge.identity_requested_at = None
        self.bridge.last_identity = 0.
        self.bridge.count_publishers = lambda topic: self.graph['clock'] if topic=='/clock' else self.graph['joints'] if topic=='/joint_states' else 1
        self.bridge.commands = NS(get_subscription_count=lambda:1)
        self.bridge.plans = NS(get_subscription_count=lambda:1)
        self.bridge.approvals = NS(get_subscription_count=lambda:1)
        self.bridge.parameters = NS(service_is_ready=lambda:True,call_async=Mock(return_value=self.future))

    def response(self, model='/trusted/base/cell.xml', plugin='mujoco_ros2_control/MujocoSystemInterface', sim=True):
        xml = f'<robot><ros2_control><hardware><plugin>{plugin}</plugin><param name="mujoco_model">{model}</param></hardware><joint name="l"/><joint name="r"/></ros2_control></robot>'
        self.future.value = NS(values=[NS(string_value=xml),NS(bool_value=sim)])
        self.future.complete = True

    def test_rpc_start_time_is_exposed_only_after_a_valid_unique_response(self):
        self.bridge.discover()
        self.assertIsNone(self.state.identity_requested_at)
        self.assertIsNone(self.state.identity_checked_at)
        self.clock[0] = 100.2; self.response(); self.bridge.discover()
        self.assertTrue(self.state.identity)
        self.assertEqual(self.state.identity_requested_at, 100.)
        self.assertEqual(self.state.identity_checked_at, 100.2)

    def test_lost_unique_graph_cancels_pending_identity_and_never_accepts_its_reply(self):
        self.bridge.discover(); self.response()
        self.graph['joints'] = 2
        self.bridge.discover()
        self.assertTrue(self.future.cancelled)
        self.assertFalse(self.state.identity)
        self.assertIsNone(self.state.observed_model_path)
        self.assertIsNone(self.state.identity_requested_at)
        self.assertIsNone(self.state.identity_checked_at)
        self.assertEqual(self.bridge.parameters.call_async.call_count, 1)

    def test_bad_response_and_timeout_clear_identity_epochs(self):
        self.bridge.discover(); self.response(sim=False); self.bridge.discover()
        self.assertFalse(self.state.identity)
        self.assertIsNone(self.state.identity_requested_at)
        self.clock[0] = 103.
        self.future = Future(); self.bridge.parameters.call_async.return_value=self.future
        self.bridge.discover()
        self.clock[0] = 107.
        self.bridge.discover()
        self.assertTrue(self.future.cancelled)
        self.assertIsNone(self.state.identity_requested_at)
        self.assertIsNone(self.state.identity_checked_at)

    def test_verified_custom_demo_path_is_observed_but_never_admits_base_commands(self):
        self.bridge.discover(); self.response(model='/trusted/run/contact_test.xml'); self.bridge.discover()
        self.assertFalse(self.state.identity)
        self.assertEqual(self.state.observed_model_path, '/trusted/run/contact_test.xml')
        self.assertEqual(self.state.identity_requested_at, 100.)
        self.assertEqual(self.state.identity_checked_at, 100.)


if __name__ == '__main__':
    unittest.main()
