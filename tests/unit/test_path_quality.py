"""Keep the schedule capacity, knot coverage and interpolation limits unchanged."""
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.path_quality import (
    MAX_SCHEDULE_SAMPLES, ScheduleSamplingBudgetExceeded, prepare_sample_schedule, sample_schedule,
)


def curve(duration, middle=None, target=.1):
    times = [0., duration] if middle is None else [0., middle, duration]
    return Spline(["joint"], [dict(t=t, p=[target * t / duration], v=[], a=[]) for t in times])


class SampleScheduleTests(unittest.TestCase):
    def test_capacity_is_checked_before_any_interpolation_and_the_exact_bound_still_passes(self):
        allowed = curve(9.998)
        self.assertEqual(MAX_SCHEDULE_SAMPLES, 5000)
        self.assertEqual(len(list(sample_schedule([allowed], dict(joint=0.), .002))), 5000)
        rejected = curve(10.)
        rejected.position = Mock(side_effect=AssertionError("oversized candidate was interpolated"))
        with self.assertRaises(ScheduleSamplingBudgetExceeded) as caught:
            prepare_sample_schedule([rejected], dict(joint=0.), .002)
        self.assertEqual(caught.exception.diagnostic["required_samples"], 5001)
        self.assertEqual(caught.exception.diagnostic["maximum_samples"], 5000)
        self.assertEqual(caught.exception.diagnostic["sample_dt_sec"], .002)
        rejected.position.assert_not_called()

    def test_existing_lazy_api_does_not_move_capacity_errors_outside_iteration_guards(self):
        samples = iter(sample_schedule([curve(10.)], dict(joint=0.), .002))
        with self.assertRaises(ScheduleSamplingBudgetExceeded):
            next(samples)

    def test_all_knots_maximum_time_gap_and_lazy_interpolation_are_retained(self):
        source = curve(1.003, middle=.013)
        original = source.position
        source.position = Mock(wraps=original)
        samples = sample_schedule([source], dict(joint=0.), .01)
        source.position.assert_not_called()
        rows = list(samples)
        stamps = [stamp for stamp, _ in rows]
        self.assertTrue(set(source.times).issubset(stamps))
        self.assertEqual(stamps[0], 0.)
        self.assertEqual(stamps[-1], source.duration)
        self.assertTrue(all(0 < right-left <= .01 for left, right in zip(stamps, stamps[1:])))
        self.assertEqual(rows[-1][1], dict(joint=.1))

    def test_interpolation_joint_step_error_is_not_relabelled_as_capacity(self):
        with self.assertRaisesRegex(ValueError, "joint sampling step exceeds 0.025rad") as caught:
            list(sample_schedule([curve(.1, target=10.)], dict(joint=0.), .01))
        self.assertNotIsInstance(caught.exception, ScheduleSamplingBudgetExceeded)
