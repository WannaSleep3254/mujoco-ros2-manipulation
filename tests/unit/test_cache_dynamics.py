"""Continuous first-knot cache gates, separate from geometry and fresh planning."""
from copy import deepcopy
import json
import math
import unittest
from unittest.mock import patch

import numpy as np

from mujoco_ros2_manipulation.workcells.cache_dynamics import (
    CacheDynamicsRejected, MIN_SEGMENT_SEC, validate_reanchored_dynamics,
)
from mujoco_ros2_manipulation.workcells.interference import Spline


def path(*, start=0., goal=.1, duration=1., layout=2, v0=0., v1=0., a0=0., a1=0.):
    return [dict(names=["joint"], points=[
        dict(t=0., p=[start], v=[v0] if layout else [], a=[a0] if layout == 2 else []),
        dict(t=duration, p=[goal], v=[v1] if layout else [], a=[a1] if layout == 2 else [])])]


def limits(**changes):
    row = dict(lower=-1., upper=1., velocity=2., effort=100.)
    row.update(changes)
    return dict(joint=row)


class CacheDynamicsTests(unittest.TestCase):
    def test_linear_cubic_quintic_extrema_match_analytic_values(self):
        cases = [(0, .1, 0.), (1, .15, .6), (2, .1875, 1. / math.sqrt(3))]
        for layout, peak_velocity, peak_acceleration in cases:
            with self.subTest(layout=layout):
                record = path(layout=layout)
                original = deepcopy(record)
                result = validate_reanchored_dynamics(record, limits())
                self.assertEqual(record, original)
                self.assertEqual(result["status"], "passed")
                self.assertAlmostEqual(result["maximum_velocity_rad_s"], peak_velocity)
                self.assertAlmostEqual(result["maximum_acceleration_rad_s2"], peak_acceleration)
                self.assertAlmostEqual(result["joints"][0]["position_min_rad"], 0.)
                self.assertAlmostEqual(result["joints"][0]["position_max_rad"], .1)
                self.assertGreaterEqual(result["elapsed_wall_sec"], 0.)
                self.assertTrue(result["full_path_revalidation_required"])

    def test_velocity_peak_between_zero_velocity_knots_is_rejected(self):
        record = path(layout=2)
        self.assertEqual(record[0]["points"][0]["v"], [0.])
        self.assertEqual(record[0]["points"][1]["v"], [0.])
        with self.assertRaises(CacheDynamicsRejected) as caught:
            validate_reanchored_dynamics(record, limits(velocity=.15))
        diagnostic = caught.exception.diagnostic
        self.assertEqual([row["kind"] for row in diagnostic["violations"]], ["velocity"])
        self.assertAlmostEqual(diagnostic["maximum_velocity_rad_s"], .1875)
        self.assertAlmostEqual(diagnostic["maximum_velocity_ratio"], 1.25)
        self.assertIn(.5, diagnostic["joints"][0]["velocity_candidates_u"])

    def test_acceleration_peak_between_zero_acceleration_knots_is_rejected(self):
        record = path(layout=2)
        self.assertEqual(record[0]["points"][0]["a"], [0.])
        self.assertEqual(record[0]["points"][1]["a"], [0.])
        with self.assertRaises(CacheDynamicsRejected) as caught:
            validate_reanchored_dynamics(record, limits(), max_acceleration=.5)
        diagnostic = caught.exception.diagnostic
        self.assertEqual([row["kind"] for row in diagnostic["violations"]], ["acceleration"])
        self.assertAlmostEqual(diagnostic["maximum_acceleration_rad_s2"], 1. / math.sqrt(3))
        roots = diagnostic["joints"][0]["acceleration_candidates_u"]
        self.assertTrue(any(abs(value - (3 - math.sqrt(3)) / 6) < 1e-10 for value in roots))

    def test_position_overshoot_between_in_limit_knots_is_rejected(self):
        # Cubic u-u^2 has both endpoint positions zero and maximum .25 at u=.5.
        record = path(goal=0., layout=1, v0=1., v1=-1.)
        with self.assertRaises(CacheDynamicsRejected) as caught:
            validate_reanchored_dynamics(record, limits(upper=.2), max_acceleration=10.)
        diagnostic = caught.exception.diagnostic
        self.assertEqual([row["kind"] for row in diagnostic["violations"]], ["position_upper"])
        self.assertAlmostEqual(diagnostic["joints"][0]["position_max_rad"], .25)
        self.assertEqual(diagnostic["joints"][0]["position_candidates_u"], [0., .5, 1.])

    def test_short_reanchor_can_pass_every_position_sample_but_violate_dynamics(self):
        for duration, expected in ((.001, {"acceleration"}), (.00001, {"velocity", "acceleration"})):
            with self.subTest(duration=duration):
                original = path(goal=0., duration=duration)
                self.assertEqual(validate_reanchored_dynamics(original, limits())["status"], "passed")
                candidate = deepcopy(original)
                candidate[0]["points"][0]["p"][0] = .0001
                spline = Spline(**candidate[0])
                for index in range(101):
                    position = spline.position(duration * index / 100)[0]
                    self.assertTrue(-1 <= position <= 1)
                with self.assertRaises(CacheDynamicsRejected) as caught:
                    validate_reanchored_dynamics(candidate, limits())
                kinds = {row["kind"] for row in caught.exception.diagnostic["violations"]}
                self.assertEqual(kinds, expected)
                self.assertGreater(caught.exception.diagnostic["maximum_acceleration_rad_s2"], 500)

    def test_short_linear_reanchor_rejects_high_velocity_without_inventing_acceleration(self):
        record = path(start=.0001, goal=0., duration=.00001, layout=0)
        with self.assertRaises(CacheDynamicsRejected) as caught:
            validate_reanchored_dynamics(record, limits())
        diagnostic = caught.exception.diagnostic
        self.assertEqual([row["kind"] for row in diagnostic["violations"]], ["velocity"])
        self.assertAlmostEqual(diagnostic["maximum_velocity_rad_s"], 10.)
        self.assertEqual(diagnostic["maximum_acceleration_rad_s2"], 0.)

    def test_normalized_time_derivatives_use_seconds_and_preserve_terminal_acceleration(self):
        record = path(duration=2., a1=.1)
        result = validate_reanchored_dynamics(record, limits())
        spline = Spline(**record[0])
        # Numerical differences are a cross-check, not the implementation gate.
        step = 1e-4
        positions = [spline.position(float(value))[0] for value in np.arange(0., 2. + step / 2, step)]
        velocities = np.diff(positions) / step
        accelerations = np.diff(velocities) / step
        self.assertAlmostEqual(result["maximum_velocity_rad_s"], float(max(abs(velocities))), places=6)
        self.assertAlmostEqual(result["maximum_acceleration_rad_s2"], float(max(abs(accelerations))), places=4)
        self.assertEqual(record[0]["points"][-1]["a"], [.1])

    def test_only_first_segment_is_claimed_and_whole_coverage_is_required(self):
        record = path()
        record[0]["points"].append(dict(t=1.00001, p=[.9], v=[0.], a=[0.]))
        result = validate_reanchored_dynamics(record, limits())
        self.assertEqual(result["scope"], "continuous_first_segment_only")
        self.assertEqual(result["status"], "passed")
        self.assertTrue(result["full_path_revalidation_required"])
        other = path(goal=-.1)
        other[0]["names"] = ["other"]
        whole = dict(limits(), other=dict(lower=-1., upper=1., velocity=2.))
        passed = validate_reanchored_dynamics(path() + other, whole)
        self.assertEqual({row["joint"] for row in passed["joints"]}, {"joint", "other"})
        with self.assertRaises(CacheDynamicsRejected):
            validate_reanchored_dynamics(path(), whole)
        with self.assertRaises(CacheDynamicsRejected):
            validate_reanchored_dynamics(path() + path(), limits())

    def test_nonfinite_malformed_and_tiny_duration_candidates_are_typed_rejections(self):
        mutations = [lambda item: item[0]["points"][0]["p"].__setitem__(0, float("nan")),
                     lambda item: item[0]["points"][1]["a"].__setitem__(0, float("inf")),
                     lambda item: item[0]["points"][0]["v"].__setitem__(0, True),
                     lambda item: item[0]["points"][1].update(t=MIN_SEGMENT_SEC / 2),
                     lambda item: item[0]["points"][1].update(t=0.),
                     lambda item: item[0]["points"][1].update(a=[]),
                     lambda item: item[0].update(names=["missing"]),
                     lambda item: item[0]["points"][0].update(extra=True)]
        for index, mutate in enumerate(mutations):
            record = path()
            mutate(record)
            with self.subTest(index=index), self.assertRaises(CacheDynamicsRejected) as caught:
                validate_reanchored_dynamics(record, limits())
            self.assertEqual(caught.exception.diagnostic["status"], "rejected")
            self.assertGreaterEqual(caught.exception.diagnostic["elapsed_wall_sec"], 0.)

    def test_bad_configuration_is_plain_value_error_and_not_a_cache_candidate_failure(self):
        invalid = [{}, dict(joint=dict(lower=1., upper=-1., velocity=2.)),
                   dict(joint=dict(lower=-1., upper=1., velocity=0.)),
                   dict(joint=dict(lower=-1., upper=1., velocity=True)),
                   dict(joint=dict(lower=-1., upper=1., velocity=float("inf"))),
                   dict(joint=dict(lower=-1., upper=1., velocity=2., unknown=1.))]
        for value in invalid:
            with self.subTest(limits=value), self.assertRaises(ValueError) as caught:
                validate_reanchored_dynamics(path(), value)
            self.assertNotIsInstance(caught.exception, CacheDynamicsRejected)
        for acceleration in (False, float("nan"), 0., -1.):
            with self.subTest(acceleration=acceleration), self.assertRaises(ValueError) as caught:
                validate_reanchored_dynamics(path(), limits(), max_acceleration=acceleration)
            self.assertNotIsInstance(caught.exception, CacheDynamicsRejected)

    def test_tiny_positive_limits_reject_without_nonfinite_diagnostics(self):
        with self.assertRaises(CacheDynamicsRejected) as caught:
            validate_reanchored_dynamics(path(), limits(velocity=1e-320), max_acceleration=1e-320)
        result = caught.exception.diagnostic
        self.assertTrue(result["ratio_overflow"])
        self.assertIsNone(result["maximum_velocity_ratio"])
        self.assertIsNone(result["maximum_acceleration_ratio"])
        self.assertEqual({value["kind"] for value in result["violations"]}, {"velocity", "acceleration"})
        json.dumps(result, allow_nan=False)

    def test_nonfinite_or_ambiguous_roots_cannot_pass_and_interrupt_is_not_swallowed(self):
        with patch("numpy.roots", return_value=np.array([float("nan")])):
            with self.assertRaises(CacheDynamicsRejected) as caught:
                validate_reanchored_dynamics(path(), limits())
            self.assertEqual(caught.exception.diagnostic["failure_kind"], "root_solver_uncertain")
        # Its position derivative is (u-.5)^2. A tiny complex split is safe only
        # when the conservative Bernstein hull certifies the joint limits.
        record = path(goal=1. / 12, layout=1, v0=.25, v1=.25)
        original_roots = np.roots
        calls = [0]
        def split_repeated_endpoint(values):
            calls[0] += 1
            return np.array([.5 + 1e-6j, .5 - 1e-6j]) if calls[0] == 1 else original_roots(values)
        with patch("numpy.roots", side_effect=split_repeated_endpoint):
            passed = validate_reanchored_dynamics(record, limits())
        self.assertEqual(passed["status"], "passed")
        self.assertTrue(passed["joints"][0]["conservative_ranges"]["position"])
        calls[0] = 0
        def split_interior_velocity(values):
            roots = original_roots(values)
            if len(values) == 4 and abs(np.polynomial.polynomial.polyval(.5, values[::-1])) < 1e-10:
                roots = np.asarray(roots, dtype=complex)
                roots[np.argmin(abs(roots - .5))] += 1e-8j
            return roots
        with patch("numpy.roots", side_effect=split_interior_velocity):
            with self.assertRaises(CacheDynamicsRejected) as caught:
                validate_reanchored_dynamics(path(), limits(velocity=.2))
            self.assertEqual(caught.exception.diagnostic["failure_kind"], "root_solver_uncertain")
        with patch("numpy.roots", side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                validate_reanchored_dynamics(path(), limits())

    def test_uncertain_solver_or_residual_passes_only_with_whole_polynomial_enclosure(self):
        record = path(start=1., goal=1. + 1e-15, duration=.02)
        original = deepcopy(record)
        original_roots = np.roots
        def poor_residual(values):
            roots = np.asarray(original_roots(values), dtype=complex)
            roots[0] = .123456
            return roots
        scenarios = [dict(side_effect=np.linalg.LinAlgError("ill conditioned")),
                     dict(return_value=np.array([float("nan")])),
                     dict(side_effect=poor_residual)]
        for scenario in scenarios:
            with self.subTest(scenario=list(scenario)), patch("numpy.roots", **scenario):
                result = validate_reanchored_dynamics(record, limits(upper=2.))
            self.assertEqual(result["status"], "passed")
            row = result["joints"][0]
            self.assertTrue(any(value == "bernstein_enclosure" for value in row["extrema_methods"].values()))
            self.assertTrue(any(row["root_uncertainty_reasons"].values()))
            self.assertGreaterEqual(result["maximum_velocity_rad_s"], result["maximum_sampled_velocity_rad_s"])
            self.assertGreaterEqual(result["maximum_acceleration_rad_s2"], result["maximum_sampled_acceleration_rad_s2"])
            self.assertEqual(result["numerical_uncertainties"], [])
            json.dumps(result, allow_nan=False)
            self.assertEqual(record, original)
        # With an uncertain solver, loose Bernstein bounds are not permission to
        # move: this enclosure exceeds the unchanged acceleration cap.
        with patch("numpy.roots", side_effect=np.linalg.LinAlgError("ill conditioned")):
            with self.assertRaises(CacheDynamicsRejected) as caught:
                validate_reanchored_dynamics(path(), limits())
        result = caught.exception.diagnostic
        self.assertEqual(result["failure_kind"], "root_solver_uncertain")
        self.assertGreater(result["maximum_acceleration_rad_s2"], 1.)
        self.assertAlmostEqual(result["maximum_sampled_acceleration_rad_s2"], 0., places=12)
        self.assertTrue(result["numerical_uncertainties"])

    def test_preserved_real_first_segments_with_irrelevant_large_roots_pass(self):
        # Public FR5+UR5e fixture, functional-b's original cold path plus its
        # observed warm initial position. Raw files are retained separately:
        # fee25d2eafc543cf87d5d142c2886b0d.json SHA256
        # 024eda2de94cc40a49e682376fe2485f65c37111885cb59dd93f84cc540481c5
        # ec0ddda9d9fc49b3b3afa5a3151599a5.json SHA256
        # baa5416f0d21468b6539e5d5c216ce64e0503dcf6f41509065f328918b0ea922
        cases = [(-1.5000037056925442, -1.500003738479747, -3.279000657804064e-06),
                 (-1.527011779220666e-06, -1.0371110182991692e-06, 4.89889486732171e-05),
                 (5.3483548856902955e-06, 2.5054013574128554e-05, .0019705691765304245),
                 (-1.5707932707254353, -1.5707933890312553, -1.1830306634566688e-05),
                 (9.672860174411422e-06, 9.441131086743853e-06, -2.317291188679342e-05)]
        for start, goal, end_velocity in cases:
            with self.subTest(start=start):
                record = path(start=start, goal=goal, v1=end_velocity, duration=.02)
                result = validate_reanchored_dynamics(record, limits(lower=-2., upper=2.))
                self.assertEqual(result["status"], "passed")
                self.assertLess(result["maximum_acceleration_rad_s2"], 1.)
                self.assertLess(result["maximum_velocity_rad_s"], 2.)
                self.assertEqual(result["violations"], [])


if __name__ == "__main__":
    unittest.main()
