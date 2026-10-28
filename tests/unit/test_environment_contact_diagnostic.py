"""Intentional-contact diagnostics cannot accidentally use an unproven path."""
import argparse
import ast
import contextlib
import io
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def functions():
    path = Path(__file__).resolve().parents[2]/'src/mujoco_ros2_manipulation/workcells/verify_environment_contact.py'
    tree = ast.parse(path.read_text(), filename=str(path))
    body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.Assign))]
    namespace = dict(require=require, math=math, argparse=argparse, Path=Path,
        require_unused_domain=Mock(), run_verification=Mock())
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(path), 'exec'), namespace)
    return namespace


class EnvironmentContactDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.namespace = functions()
        self.gate = self.namespace['contact_path_gate']
        self.contact = dict(distance_m=-.02, bodies=['left_wrist3_link', 'environment_contact_box'])

    def test_clear_start_and_interarm_space_with_imported_box_overlap_is_required(self):
        self.gate(.074, .186, self.contact, sample_count=1001)
        for home, inter, contact, samples in (
            (.001, .186, self.contact, 1001),
            (.074, .02, self.contact, 1001),
            (.074, .186, dict(self.contact, distance_m=.001), 1001),
            (.074, .186, dict(self.contact, bodies=['left_wrist3_link', 'other_obstacle']), 1001),
            (.074, .186, dict(self.contact, bodies=['environment_contact_box', 'right_wrist']), 1001),
            (.074, .186, self.contact, 1000),
            (float('inf'), .186, self.contact, 1001),
            (.074, float('nan'), self.contact, 1001),
            (.074, .186, dict(self.contact, distance_m=float('-inf')), 1001)):
            with self.subTest(home=home, inter=inter, contact=contact, samples=samples):
                with self.assertRaises(RuntimeError):
                    self.gate(home, inter, contact, sample_count=samples)

    def test_missing_explicit_contact_authorization_rejects_before_ros_probe(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as caught:
            self.namespace['main'](['--domain', '230', '--execution-environment', '/unread',
                '--execution-manifest-sha256', 'a'*64])
        self.assertEqual(caught.exception.code, 2)
        self.namespace['require_unused_domain'].assert_not_called()
        self.namespace['run_verification'].assert_not_called()

    def test_raw_goal_must_match_fk_anchor_order_target_and_linear_time_contract(self):
        initial, goal = dict(a=.1, b=.2), dict(a=.7, b=.28)
        def trajectory():
            return NS(joint_names=['a', 'b'], points=[
                NS(positions=[.1,.2], velocities=[], accelerations=[], time_from_start=NS(sec=0,nanosec=0)),
                NS(positions=[.7,.28], velocities=[], accelerations=[], time_from_start=NS(sec=4,nanosec=0))])
        contract = self.namespace['raw_goal_contract'](trajectory(), ['a','b'], initial, goal)
        self.assertTrue(contract['exact_fk_goal_anchor_match'])
        mutations = [lambda value:value.joint_names.reverse(),
            lambda value:value.points[0].positions.__setitem__(0, .10001),
            lambda value:value.points[-1].positions.__setitem__(1, .28001),
            lambda value:setattr(value.points[-1].time_from_start, 'sec', 5),
            lambda value:setattr(value.points[0], 'velocities', [0.,0.])]
        for mutation in mutations:
            value = trajectory(); mutation(value)
            with self.assertRaises(RuntimeError):
                self.namespace['raw_goal_contract'](value, ['a','b'], initial, goal)
