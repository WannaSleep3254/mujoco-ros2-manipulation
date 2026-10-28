"""Physical fixture gates are deliberately separate from FR5/ROS verification."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.grasping.fixture import FixtureSpec
from mujoco_ros2_manipulation.grasping.run import CASES, Experiment, run_case, run_suite


class FailureArtifactTests(unittest.TestCase):
    def test_initialization_failure_is_saved_for_every_case_and_never_passes(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "new-run"
            with patch("mujoco_ros2_manipulation.grasping.run.Experiment",
                       side_effect=ImportError("Synthetic missing engine")):
                receipt = run_suite(directory)
            self.assertFalse(receipt["passed"])
            self.assertEqual(len(receipt["cases"]), len(CASES))
            for row in receipt["cases"]:
                raw = (directory / row["result"]).read_bytes()
                result = json.loads(raw)
                self.assertEqual(result["outcome"], "error")
                self.assertEqual(result["execution_error"]["type"], "ImportError")
                self.assertFalse(result["expectation"]["passed"])
                self.assertEqual(hashlib.sha256(raw).hexdigest(), row["result_sha256"])
                self.assertTrue((directory / row["mjcf"]).is_file())
            before = (directory / "summary.json").read_bytes()
            with self.assertRaises(FileExistsError):
                run_suite(directory)
            self.assertEqual((directory / "summary.json").read_bytes(), before)


@unittest.skipUnless(importlib.util.find_spec("mujoco") and importlib.util.find_spec("numpy"),
                     "MuJoCo and NumPy are required for independent physical fixture tests")
class SyntheticPhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = {case: run_case(case)[1] for case in CASES}

    def test_normal_free_object_is_lifted_carried_and_physically_released(self):
        result = self.results["normal"]
        self.assertEqual(result["outcome"], "succeeded")
        self.assertTrue(result["expectation"]["passed"])
        self.assertGreaterEqual(result["lift_evidence"]["lift_m"], .06)
        self.assertLessEqual(result["lift_evidence"]["relative_position_error_m"], .005)
        self.assertLessEqual(abs(result["final"]["object_position_m"][0]-.15), .01)
        self.assertGreater(result["final"]["table_object_normal_force_n"], .1)
        self.assertEqual(result["physics"]["weld_or_attachment_constraints"], 0)
        self.assertEqual(result["physics"]["object_pose_overwrites"], 0)
        self.assertEqual(result["task"]["planning_attachment"], "world")
        self.assertEqual(result["task"]["physical_grasp"], "released")
        self.assertIn("not FR5", result["scope"])

    def test_missing_and_one_sided_contact_do_not_advance_to_lift(self):
        for case in ("missing_object", "one_sided_contact"):
            with self.subTest(case=case):
                result = self.results[case]
                self.assertEqual(result["outcome"], "failed")
                self.assertTrue(result["expectation"]["passed"])
                self.assertEqual(result["task"]["physical_grasp"], "unconfirmed")
                self.assertEqual(result["task"]["planning_attachment"], "world")
                self.assertNotIn("extract", [row["phase"] for row in result["segments"]])
                self.assertNotIn("carry", [row["phase"] for row in result["segments"]])

    def test_slip_failure_stays_failure_even_when_the_expectation_passes(self):
        result = self.results["slip"]
        self.assertEqual(result["outcome"], "failed")
        self.assertTrue(result["expectation"]["passed"])
        self.assertEqual(result["task"]["physical_grasp"], "slipped")
        self.assertEqual(result["task"]["planning_attachment"], "world")
        self.assertGreater(result["lift_evidence"]["relative_position_error_m"], .005)
        self.assertNotIn("carry", [row["phase"] for row in result["segments"]])

    def test_unexpected_contact_terminates_without_invalid_transition_or_close(self):
        result = self.results["unexpected_contact"]
        self.assertEqual(result["outcome"], "failed")
        self.assertTrue(result["expectation"]["passed"])
        self.assertIsNone(result["execution_error"])
        self.assertIn(["block_geom", "palm_geom"], result["physics"]["unexpected_contact_pairs"])
        self.assertNotIn("close", [row["phase"] for row in result["segments"]])

    def test_cancel_records_separate_stop_and_retained_object_observations(self):
        result = self.results["cancel"]
        self.assertEqual(result["outcome"], "cancelled")
        self.assertTrue(result["expectation"]["passed"])
        self.assertTrue(result["cancellation"]["stop_confirmed"])
        self.assertGreaterEqual(result["cancellation"]["stop_observed_sec"], .2)
        self.assertTrue(result["cancellation"]["grasp_still_observed"])
        self.assertFalse(result["cancellation"]["automatic_release"])
        self.assertEqual(result["task"]["planning_attachment"], "attached")
        self.assertNotIn("open", [row["phase"] for row in result["segments"]])
        self.assertEqual(result["task"]["events"][-1]["event"], "cancelled_hold_observed")

    def test_observation_exception_is_not_an_expected_missing_object_rejection(self):
        with patch.object(Experiment, "observe", side_effect=RuntimeError("Synthetic observation failure")):
            _, result = run_case("missing_object")
        self.assertEqual(result["outcome"], "failed")
        self.assertFalse(result["expectation"]["passed"])
        self.assertEqual(result["execution_error"]["message"], "Synthetic observation failure")
        self.assertEqual(result["physics"]["integrated_steps"], 1)
        self.assertIsNone(result["final"])
        self.assertEqual(result["final_observation_error"]["type"], "RuntimeError")

    def test_invalid_motion_bounds_fail_before_a_physics_step(self):
        experiment = Experiment(FixtureSpec(), case="normal")
        conditions = [dict(target=[0., 0., 0., 0.], duration=value)
                      for value in (0., -1., 3., float("nan"), "one", True, .00001)]
        conditions += [dict(target=[.31, 0., 0., 0.], duration=.2),
                       dict(target=[0., 0., 0., 0.], duration=.2, cancel_at_fraction=0.),
                       dict(target=[0., 0., 0., 0.], duration=.2, cancel_at_fraction=float("inf"))]
        for condition in conditions:
            with self.subTest(condition=condition), self.assertRaises(ValueError):
                experiment.segment("invalid_input", **condition)
            self.assertEqual(experiment.steps, 0)


if __name__ == "__main__":
    unittest.main()
