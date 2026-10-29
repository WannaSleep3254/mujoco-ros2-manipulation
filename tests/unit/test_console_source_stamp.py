"""Exact source timestamps do not participate in existing command admission."""
import ast
from copy import deepcopy
from importlib.util import find_spec
from pathlib import Path
from types import SimpleNamespace
import unittest

from tests.unit.test_operator_console import fixture


def bridge_joints():
    path = Path(find_spec('mujoco_ros2_manipulation.operator_console.ros_adapter').origin)
    tree = ast.parse(path.read_text())
    bridge = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ConsoleBridge')
    method = next(n for n in bridge.body if isinstance(n, ast.FunctionDef) and n.name == 'joints')
    module = ast.Module(body=[deepcopy(method)], type_ignores=[])
    namespace = {}
    exec(compile(ast.fix_missing_locations(module), str(path), 'exec'), namespace)
    return namespace['joints']


class ConsoleSourceStampTests(unittest.TestCase):
    def test_large_integer_nanoseconds_are_not_reconstructed_from_float_seconds(self):
        state, _ = fixture()
        first = 1_793_234_567_123_456_789
        self.assertEqual(first / 1e9, (first + 1) / 1e9)
        for stamp in (first, first + 1, first - 1):
            state.joints(['l','r'], [.1,.2], [0.,0.],
                         source_stamp=stamp / 1e9, source_stamp_ns=stamp)
            self.assertEqual(state.joint_state_stamp_ns, stamp)
            self.assertEqual(state.joint_state_stamp_simulation_sec, stamp / 1e9)
            self.assertTrue(state.snapshot()['can_submit'])
            self.assertFalse(state.clock_backwards)

    def test_invalid_exact_stamps_clear_metadata_without_rejecting_valid_joints(self):
        state, _ = fixture()
        for invalid in (None, -1, True, 1.5, '123', 2**63):
            with self.subTest(invalid=invalid):
                state.joints(['l','r'], [.1,.2], [0.,0.], source_stamp=1., source_stamp_ns=123)
                state.joints(['l','r'], [.2,.3], [0.,0.], source_stamp=2., source_stamp_ns=invalid)
                self.assertIsNone(state.joint_state_stamp_ns)
                self.assertEqual(state.positions, {'l':.2, 'r':.3})
                self.assertEqual(state.joint_state_stamp_simulation_sec, 2.)
                self.assertTrue(state.snapshot()['can_submit'])

    def test_rejected_joint_message_does_not_replace_exact_stamp(self):
        state, _ = fixture()
        state.joints(['l','r'], [.1,.2], [0.,0.], source_stamp=1., source_stamp_ns=123)
        state.joints(['l'], [.9], [0.], source_stamp=2., source_stamp_ns=456)
        self.assertEqual(state.joint_state_stamp_ns, 123)
        self.assertEqual(state.positions, {'l':.1, 'r':.2})

    def test_ros_bridge_adds_seconds_and_nanoseconds_as_exact_integers(self):
        state, _ = fixture()
        method = bridge_joints()
        message = SimpleNamespace(name=['l','r'], position=[.1,.2], velocity=[0.,0.],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1_793_234_567, nanosec=123_456_789)))
        method(SimpleNamespace(state=state), message)
        self.assertEqual(state.joint_state_stamp_ns, 1_793_234_567_123_456_789)
        message.header.stamp.nanosec += 1
        method(SimpleNamespace(state=state), message)
        self.assertEqual(state.joint_state_stamp_ns, 1_793_234_567_123_456_790)
        self.assertTrue(state.snapshot()['can_submit'])
        message.header.stamp.nanosec = 1_000_000_000
        method(SimpleNamespace(state=state), message)
        self.assertIsNone(state.joint_state_stamp_ns)
        self.assertEqual(state.positions, {'l':.1, 'r':.2})


if __name__ == '__main__':
    unittest.main()
