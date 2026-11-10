"""Reject unsupported physical fixtures before inspecting or creating outputs."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.grasping.ros_model import build_model, _block_mass


class BlockFixtureSelectionTests(unittest.TestCase):
    def test_two_declared_fixtures_have_the_exact_checked_masses(self):
        self.assertEqual(_block_mass("block_80g"), .08)
        self.assertEqual(_block_mass("block_120g"), .12)

    def test_unsupported_selectors_fail_before_source_reads_or_output_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "must-not-exist"
            validator = Mock(side_effect=AssertionError("source must remain untouched"))
            for value in (True, False, None, 0, -1, .08, .12, float("nan"), float("inf"),
                          "", "block_100g", "../block_80g", [], {}):
                with self.subTest(selector=repr(value)):
                    with self.assertRaisesRegex(ValueError, "declared block"):
                        build_model(None, output, block_fixture=value, base_validator=validator)
                    self.assertFalse(output.exists())
            validator.assert_not_called()
