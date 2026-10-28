"""Environment clearance is explicit, independent, and fails closed without native evidence."""
from copy import deepcopy
from pathlib import Path
import tempfile
import hashlib
from types import SimpleNamespace as NS
import unittest

import yaml

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.environment_clearance import (
    ENVIRONMENT_RESULT_SCOPE, MAX_ENVIRONMENT_PAIRS, load_environment_policy,
    required_environment_distance, require_environment_result, environment_policy_snapshot,
)


def policy():
    return dict(schema_version=1, min_environment_distance_m=.005, tracking_reserve_m=.01)


def observation():
    return dict(distance_m=.074, bodies=["left_forearm_link", "workcell_floor"],
                nearest_points_m=[[0., 0., .1], [0., 0., .026]], pair_count=56,
                scope=ENVIRONMENT_RESULT_SCOPE)


class EnvironmentPolicyTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.path = self.root / "config/planning/environment_clearance.yaml"
        self.path.parent.mkdir(parents=True)
        self.cell = NS(root=self.root, id="any_workcell")

    def write(self, value):
        self.path.write_text(yaml.safe_dump(value))

    def test_shipped_policy_requires_fifteen_mm_independently_of_inter_arm_policy(self):
        path, value = load_environment_policy(NS(root=ROOT, id="fr5_ur5e_eval_post"))
        self.assertEqual(path, ROOT / "config/planning/environment_clearance.yaml")
        self.assertEqual(value, policy())
        self.assertEqual(required_environment_distance(value), .015)
        self.assertEqual(set(value), {"schema_version", "min_environment_distance_m", "tracking_reserve_m"})

    def test_policy_is_global_without_fake_workcell_wildcards(self):
        self.write(policy())
        self.assertEqual(load_environment_policy(self.cell), (self.path, policy()))
        self.cell.id = "different_workcell"
        self.assertEqual(load_environment_policy(self.cell)[1], policy())

    def test_policy_snapshot_digest_identifies_the_interpreted_bytes(self):
        self.write(policy())
        raw = self.path.read_bytes()
        path, value, digest = environment_policy_snapshot(self.cell)
        self.assertEqual(path, self.path)
        self.assertEqual(value, policy())
        self.assertEqual(digest, hashlib.sha256(raw).hexdigest())
        self.write(dict(policy(), tracking_reserve_m=.02))
        self.assertNotEqual(digest, hashlib.sha256(self.path.read_bytes()).hexdigest())
        self.assertEqual(required_environment_distance(value), .015)

    def test_policy_rejects_missing_unknown_schema_and_non_mapping_fields(self):
        bad = [None, [], "policy", dict(policy(), workcell="*"), dict(policy(), schema_version=True),
               dict(policy(), schema_version=2), dict(policy(), schema_version=1.)]
        for name in policy():
            value = policy()
            del value[name]
            bad.append(value)
        for value in bad:
            with self.subTest(value=value):
                self.write(value)
                with self.assertRaises(ValueError):
                    load_environment_policy(self.cell)
                with self.assertRaises(ValueError):
                    required_environment_distance(value)

    def test_policy_numbers_are_finite_bounded_and_never_boolean(self):
        for name, values in (
            ("min_environment_distance_m", [True, False, "0.005", None, float("nan"), float("inf"), -.001, 0., .0009, .1001, 10 ** 1000]),
            ("tracking_reserve_m", [True, False, ".01", None, float("nan"), float("-inf"), -.001, .0501, 10 ** 1000]),
        ):
            for invalid in values:
                with self.subTest(name=name, invalid=invalid):
                    value = dict(policy(), **{name: invalid})
                    self.write(value)
                    with self.assertRaises(ValueError):
                        load_environment_policy(self.cell)
                    with self.assertRaises(ValueError):
                        required_environment_distance(value)
        for minimum, reserve in ((.001, 0.), (.1, .05)):
            value = dict(policy(), min_environment_distance_m=minimum, tracking_reserve_m=reserve)
            self.assertAlmostEqual(required_environment_distance(value), minimum + reserve)

    def test_missing_or_invalid_yaml_is_not_replaced_with_a_default(self):
        with self.assertRaises(FileNotFoundError):
            load_environment_policy(self.cell)
        self.path.write_text("schema_version: [")
        with self.assertRaises(yaml.YAMLError):
            load_environment_policy(self.cell)


class EnvironmentResultTests(unittest.TestCase):
    def test_valid_signed_distance_is_preserved_without_threshold_judgment(self):
        for distance in (.074, .015, .0017494, -.00014557):
            with self.subTest(distance=distance):
                value = dict(observation(), distance_m=distance)
                original = deepcopy(value)
                self.assertIs(require_environment_result(value), value)
                self.assertEqual(value, original)
        self.assertLess(.0017494, required_environment_distance(policy()))
        self.assertGreater(.074, required_environment_distance(policy()))

    def test_missing_extra_and_non_mapping_result_is_not_silently_ignored(self):
        bad = [None, [], "environment", dict(observation(), tips_m=[])]
        for name in observation():
            value = observation()
            del value[name]
            bad.append(value)
        for value in bad:
            with self.subTest(value=value), self.assertRaises(ValueError):
                require_environment_result(value)

    def test_distance_pair_count_and_scope_are_strict(self):
        bad = [("distance_m", value) for value in (True, False, None, "0.1", float("nan"), float("inf"), 10 ** 1000)]
        bad += [("pair_count", value) for value in (True, False, 0, -1, 1., "1", MAX_ENVIRONMENT_PAIRS + 1)]
        bad += [("scope", value) for value in (None, "", "cross_arm", "Moving robot versus environment")]
        for name, invalid in bad:
            with self.subTest(name=name, invalid=invalid), self.assertRaises(ValueError):
                require_environment_result(dict(observation(), **{name: invalid}))
        for count in (1, MAX_ENVIRONMENT_PAIRS):
            require_environment_result(dict(observation(), pair_count=count))

    def test_body_names_are_two_distinct_bounded_nonempty_strings(self):
        for invalid in (None, "left,world", (), [], ["left"], ["left", "right", "world"],
                        ["left", "left"], ["left", ""], ["left", " "], ["left", 1], ["left", "x" * 257]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                require_environment_result(dict(observation(), bodies=invalid))

    def test_witness_points_require_two_complete_finite_xyz_vectors(self):
        for invalid in (None, (), [], [[0., 0., 0.]], [[0., 0.], [0., 0., 0.]],
                        [[0., 0., 0., 0.], [0., 0., 0.]], ["xyz", [0., 0., 0.]],
                        [[True, 0., 0.], [0., 0., 0.]], [[float("nan"), 0., 0.], [0., 0., 0.]],
                        [[0., 0., 0.], [0., float("inf"), 0.]], [[0., 0., 0.], [0., 0., 10 ** 1000]]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                require_environment_result(dict(observation(), nearest_points_m=invalid))
