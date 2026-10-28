"""Pose admission uses its own topic, exact catalog and existing one-use review.

No ROS, MuJoCo, sockets or child process is used here. The actual publisher
method is executed with pure collaborators to test the final publication gate.
"""
import ast
from copy import deepcopy
from email.message import Message
import io
import json
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.catalog import validate_pose_registration
from mujoco_ros2_manipulation.operator_console.http_server import Handler
from mujoco_ros2_manipulation.operator_console.state import ConsoleState
from mujoco_ros2_manipulation.workcells.motion_request import command
from mujoco_ros2_manipulation.workcells.pose_goal import ToolDefinition

SESSION = 'a' * 32
PLAN = 'b' * 32
DIGEST = 'c' * 64
TOOL = ToolDefinition('left_flange_reference', 'left_tip', source='Unit-test origin reference')


def fixture():
    now = [0.]
    row = dict(arm_id='left', label='FR5 test reference',
        tool=dict(TOOL.record(), sha256=TOOL.sha256), frame_id='world', position_unit='m',
        orientation_format='quaternion_xyzw', scope='flange_reference_only')
    config = dict(home_rad=dict(l=0., r=0.),
        joint_limits={name: dict(lower=-1., upper=1.) for name in ('l', 'r')}, pose_tools=[row])
    state = ConsoleState(config, clock=lambda: now[0])
    now[0] = 3.
    state.graph = dict(clock=1, joints=1, status=1, commands=1, guards=1, plans=1, approvals=1, pose_plans=1)
    state.identity = True
    state.guard = dict(latched=False, phase='ready')
    state.joints(['l', 'r'], [0., 0.], [0., 0.])
    state.tick(1.)
    state.status(dict(session_id=SESSION, request_id=None, status='ready', review=None,
                      pose_tools=deepcopy(config['pose_tools'])))
    goal = dict(schema_version=1, request_id=PLAN, arm_id='left', tool_id=TOOL.id,
        tool_sha256=TOOL.sha256, frame_id='world', position_m=[.1, .2, .3],
        quaternion_xyzw=[0., 0., 0., 1.])
    return state, now, goal


def planned(state, goal):
    state.reserve_pose_plan(goal)
    review = dict(state='awaiting_approval', plan_request_id=PLAN, session_id=SESSION,
        plan_sha256=DIGEST, expires_after_sec=60., remaining_sec=60.)
    state.status(dict(session_id=SESSION, request_id=PLAN, status='succeeded', stage='plan_ready',
        mode='plan_only', command=command(dict(l=.1, r=0.), identifier=PLAN),
        pose_goal=deepcopy(goal), review=review))
    state.status(dict(session_id=SESSION, request_id=None, status='ready', review=review,
        pose_tools=deepcopy(state.config['pose_tools'])))
    return dict(schema_version=1, request_id='d'*32, plan_request_id=PLAN,
                plan_sha256=DIGEST, session_id=SESSION)


class PoseCatalogTests(unittest.TestCase):
    def test_registered_reference_resolves_without_implying_tool_geometry(self):
        state, _, goal = fixture()
        self.assertEqual(validate_pose_registration(state.config, goal), goal)
        checked = validate_pose_registration(state.config, goal)
        checked['position_m'][0] = 999
        self.assertEqual(goal['position_m'][0], .1)
        self.assertEqual(state.config['pose_tools'][0]['scope'], 'flange_reference_only')

    def test_unknown_arm_tool_hash_and_unregistered_transform_are_refused(self):
        for field, value in (('arm_id', 'other'), ('tool_id', 'other'), ('tool_sha256', 'f'*64), ('frame_id', 'base')):
            state, _, goal = fixture()
            goal[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                state.reserve_pose_plan(goal)
            self.assertIsNone(state.active)
        state, _, goal = fixture()
        altered = ToolDefinition(TOOL.id, TOOL.parent_tip, (.0, .0, .1), source=TOOL.source)
        state.config['pose_tools'][0]['tool'] = dict(altered.record(), sha256=altered.sha256)
        goal['tool_sha256'] = altered.sha256
        with self.assertRaises(ValueError): state.reserve_pose_plan(goal)
        self.assertIsNone(state.active)

    def test_duplicate_catalog_entries_and_wrong_reference_scope_are_not_ambiguous(self):
        for change in ('duplicate', 'scope', 'digest'):
            state, _, goal = fixture()
            if change == 'duplicate': state.config['pose_tools'].append(deepcopy(state.config['pose_tools'][0]))
            elif change == 'scope': state.config['pose_tools'][0]['scope'] = 'physical_gripper'
            else: state.config['pose_tools'][0]['tool']['sha256'] = 'f'*64
            with self.subTest(change=change), self.assertRaises(ValueError):
                state.reserve_pose_plan(goal)
            self.assertIsNone(state.active)


class PoseStateTests(unittest.TestCase):
    def test_old_server_does_not_enable_pose_but_preserves_joint_plans(self):
        state, _, goal = fixture()
        state.server.pop('pose_tools')
        self.assertFalse(state.snapshot()['can_pose_plan'])
        self.assertTrue(state.snapshot()['can_plan'])
        with self.assertRaises(RuntimeError): state.reserve_pose_plan(goal)
        self.assertIsNone(state.active)
        self.assertTrue(state.reserve_plan(command(dict(l=.1, r=.2))))

    def test_pose_reserves_single_owner_plan_only_and_repeated_id_never_resubmits(self):
        state, _, goal = fixture()
        self.assertTrue(state.snapshot()['can_pose_plan'])
        self.assertTrue(state.reserve_pose_plan(goal))
        self.assertFalse(state.reserve_pose_plan(deepcopy(goal)))
        self.assertEqual(state.history[PLAN]['mode'], 'plan_only')
        self.assertEqual(state.history[PLAN]['pose_goal'], goal)
        self.assertIsNone(state.pending_plan)
        with self.assertRaises(RuntimeError): state.reserve_plan(command(dict(l=.1, r=.2)))
        changed = dict(goal, position_m=[.2, .2, .3])
        with self.assertRaises(ValueError): state.reserve_pose_plan(changed)

    def test_same_base_stale_clock_contact_and_busy_rules_apply(self):
        for change in ('stale', 'contact', 'busy', 'duplicate_topic'):
            state, now, goal = fixture()
            if change == 'stale': now[0] += 1
            elif change == 'contact': state.guard['latched'] = True
            elif change == 'busy': state.active = 'e'*32
            else: state.graph['pose_plans'] = 2
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                state.reserve_pose_plan(goal)
            self.assertNotIn(PLAN, state.history)

    def test_review_retains_pose_when_trace_command_is_resolved_joint_goal(self):
        state, _, goal = fixture()
        approval = planned(state, goal)
        self.assertEqual(state.history[PLAN]['pose_goal'], goal)
        self.assertNotIn('arm_id', state.history[PLAN]['command'])
        self.assertTrue(state.reserve_approval(approval))
        self.assertEqual(state.pending_plan['state'], 'approval_queued')

    def test_changed_tool_capability_cannot_approve_existing_pose_review(self):
        for change in ('catalog', 'runtime', 'topic'):
            state, _, goal = fixture()
            approval = planned(state, goal)
            if change == 'catalog': state.config['pose_tools'][0]['tool']['source'] = 'changed'
            elif change == 'runtime': state.server['pose_tools'] = []
            else: state.graph['pose_plans'] = 0
            with self.subTest(change=change), self.assertRaises(RuntimeError):
                state.reserve_approval(approval)
            self.assertIsNone(state.active)
            self.assertEqual(state.pending_plan['state'], 'awaiting_approval')


class PoseBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path('src/mujoco_ros2_manipulation/operator_console/ros_adapter.py')
        tree = ast.parse(source.read_text())
        definition = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ConsoleBridge')
        method = deepcopy(next(n for n in definition.body if isinstance(n, ast.FunctionDef) and n.name == 'send'))
        namespace = dict(queue=queue, json=json, String=lambda **kwargs: NS(**kwargs))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
        cls.send = staticmethod(namespace['send'])

    def bridge(self, state):
        return NS(state=state, outbox=queue.Queue(), commands=NS(publish=Mock()), plans=NS(publish=Mock()),
            pose_plans=NS(publish=Mock()), approvals=NS(publish=Mock()), cancels=NS(publish=Mock()))

    def test_only_pose_topic_publishes_once_under_exact_owned_reservation(self):
        state, _, goal = fixture(); state.reserve_pose_plan(goal)
        bridge = self.bridge(state); bridge.outbox.put(('pose_plan', goal))
        self.send(bridge); self.send(bridge)
        bridge.pose_plans.publish.assert_called_once()
        self.assertEqual(json.loads(bridge.pose_plans.publish.call_args.args[0].data), goal)
        bridge.commands.publish.assert_not_called(); bridge.plans.publish.assert_not_called()
        bridge.approvals.publish.assert_not_called()

    def test_catalog_disconnect_or_replacement_drops_queued_pose_without_replay(self):
        for change in ('tool', 'topic', 'session'):
            state, _, goal = fixture(); state.reserve_pose_plan(goal)
            bridge = self.bridge(state); bridge.outbox.put(('pose_plan', goal))
            if change == 'tool': state.server['pose_tools'] = []
            elif change == 'topic': state.graph['pose_plans'] = 0
            else: state.status(dict(session_id='e'*32, request_id=None, status='ready'))
            self.send(bridge)
            bridge.pose_plans.publish.assert_not_called(); bridge.commands.publish.assert_not_called()
            self.assertIn(state.history[PLAN]['status'], ('rejected', 'unknown'))
            self.assertTrue(bridge.outbox.empty())

    def test_pose_approval_uses_existing_topic_without_blocking_its_own_reservation(self):
        state, _, goal = fixture(); approval = planned(state, goal)
        state.reserve_approval(approval)
        bridge = self.bridge(state); bridge.outbox.put(('approve', approval))
        self.send(bridge)
        bridge.approvals.publish.assert_called_once()
        bridge.pose_plans.publish.assert_not_called(); bridge.commands.publish.assert_not_called()

    def test_tool_change_after_approval_reservation_refuses_publication(self):
        state, _, goal = fixture(); approval = planned(state, goal)
        state.reserve_approval(approval)
        bridge = self.bridge(state); bridge.outbox.put(('approve', approval))
        state.server['pose_tools'] = []
        self.send(bridge)
        bridge.approvals.publish.assert_not_called()
        self.assertEqual(state.history[approval['request_id']]['status'], 'rejected')


class PoseApplicationTests(unittest.TestCase):
    def test_pose_plan_uses_own_queue_kind_and_preview_blocks_it(self):
        state, _, goal = fixture()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with patch.object(ConsoleApplication, 'watch', return_value=None):
                app = ConsoleApplication(NS(root=root), state, NS(outbox=queue.Queue()), root/'console', render=False)
                app.monitor.join(timeout=1)
            app.create_pose_plan(goal)
            self.assertEqual(app.bridge.outbox.get_nowait(), ('pose_plan', goal))
            state.status(dict(session_id=SESSION, request_id=PLAN, status='cancelled', stage='terminal'))
            app.views.mode = 'model_preview'
            self.assertFalse(app.snapshot()['can_pose_plan'])
            with self.assertRaisesRegex(RuntimeError, '미리보기'):
                app.create_pose_plan(dict(goal, request_id='e'*32))
            self.assertTrue(app.bridge.outbox.empty())
            app.close()


class PoseHTTPTests(unittest.TestCase):
    def request(self, token='secret'):
        _, _, goal = fixture()
        handler = object.__new__(Handler); handler.path = '/api/pose/plan'; handler.headers = Message()
        handler.headers['Host'] = '127.0.0.1:8890'; handler.headers['Origin'] = 'http://127.0.0.1:8890'
        handler.headers['X-Operator-Token'] = token; handler.headers['Content-Type'] = 'application/json'
        raw = json.dumps(goal).encode(); handler.headers['Content-Length'] = str(len(raw)); handler.rfile = io.BytesIO(raw)
        app = NS(create_pose_plan=Mock(return_value=dict(request_id=PLAN)), create_plan=Mock(), submit=Mock())
        handler.server = NS(application=app, server_address=('127.0.0.1', 8890), token='secret')
        handler.respond = Mock(); handler.do_POST()
        return handler, app, goal

    def test_pose_route_never_falls_back_to_joint_or_direct_execution(self):
        handler, app, goal = self.request()
        app.create_pose_plan.assert_called_once_with(goal)
        app.create_plan.assert_not_called(); app.submit.assert_not_called()
        self.assertEqual(handler.respond.call_args.args[1], 202)

    def test_pose_route_preserves_operator_token_gate(self):
        handler, app, _ = self.request(token='wrong')
        self.assertEqual(handler.respond.call_args.args[1], 403)
        app.create_pose_plan.assert_not_called()


if __name__ == '__main__':
    unittest.main()
