"""Matched evidence must reject altered conditions, stale plans and failed runs."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.interference_comparison import (
    load_bundle,
    compare_reports,
    MODES,
)


def curve(names, start, end):
    return dict(
        names=names,
        points=[dict(t=0, p=start, v=[], a=[]), dict(t=2, p=end, v=[], a=[])],
    )


class MatchedComparisonTests(unittest.TestCase):
    def fixture(self, root):
        casepath = root / "case.yaml"
        casepath.write_text("test-case")
        (root / "source.json").write_text("test-model-manifest")
        cell = NS(
            joints=["a", "b"],
            home=[-0.2, -0.2],
            output=root,
            instances=[NS(id="left", joints=["a"]), NS(id="right", joints=["b"])],
        )
        case = dict(start_rad={"a": 0, "b": 0}, goal_rad={"a": 0.5, "b": 0.5})
        data = dict(
            schema_version=1,
            case_id="cross",
            case_sha256=file_sha256(casepath),
            model_manifest_sha256=file_sha256(root / "source.json"),
            initial_rad=case["start_rad"],
            goal_rad=case["goal_rad"],
            curves=dict(
                setup=curve(["a", "b"], [-0.2, -0.2], [0, 0]),
                left=curve(["a"], [0], [0.5]),
                right=curve(["b"], [0], [0.5]),
                avoidance=curve(["a", "b"], [0, 0], [0.5, 0.5]),
            ),
        )
        return cell, casepath, case, data

    def test_bundle_rejects_changed_source_start_ownership_and_time_axis(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            cell, casepath, case, data = self.fixture(root)
            path = root / "bundle.json"
            with patch(
                "mujoco_ros2_manipulation.workcells.interference_comparison.load_case",
                return_value=(casepath, case),
            ):
                path.write_text(json.dumps(data))
                load_bundle(path, cell)
                for change in (
                    "model",
                    "case",
                    "start",
                    "ownership",
                    "duration",
                    "nonfinite",
                ):
                    broken = deepcopy(data)
                    if change == "model":
                        broken["model_manifest_sha256"] = "stale"
                    if change == "case":
                        broken["case_sha256"] = "stale"
                    if change == "start":
                        broken["curves"]["left"]["points"][0]["p"] = [0.1]
                    if change == "ownership":
                        broken["curves"]["left"]["names"] = ["b"]
                    if change == "duration":
                        broken["curves"]["left"]["points"][1]["t"] = 3
                    if change == "nonfinite":
                        broken["initial_rad"]["a"] = float("nan")
                    path.write_text(json.dumps(broken))
                    with self.subTest(change=change), self.assertRaises(ValueError):
                        load_bundle(path, cell)

    def reports(self):
        reports = []
        for mode in MODES:
            collision = mode == "collision"
            reports.append(
                dict(
                    comparison_mode=mode,
                    run_id=mode,
                    bundle_sha256="matched",
                    model_manifest_sha256="model",
                    case_file_sha256="case",
                    initial_commanded_rad={"a": 0},
                    goal_constraint_rad={"a": 1},
                    command_hashes={"left": "same"},
                    comparison_duration_sec=24,
                    status="passed",
                    motion_checks_passed=True,
                    physics_checks_passed=True,
                    shutdown_clean=True,
                    process_group_gone=True,
                    verifier_cancel_requests=0,
                    contact_injection_requests=0,
                    physics=dict(
                        stop_reason="contact" if collision else "",
                        stop_latched=collision,
                        contacts=int(collision),
                    ),
                    remaining_motion_at_contact_sec=3 if collision else None,
                    post_stop_drift_rad=0 if collision else None,
                    goal_terminal_statuses=[5, 5]
                    if collision
                    else ([4, 4] if mode == "avoidance" else [4]),
                    motion_duration_sec=6,
                )
            )
        return reports

    def test_summary_requires_four_fresh_successful_runs_with_identical_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = [Path(tmp) / (m + ".json") for m in MODES]
            good = self.reports()

            def write(reports):
                for p, r in zip(paths, reports):
                    p.write_text(json.dumps(r))

            write(good)
            self.assertTrue(compare_reports(paths)["matched"])
            for field, value in [
                ("bundle_sha256", "different"),
                ("command_hashes", {}),
                ("status", "failed"),
                ("verifier_cancel_requests", 1),
                ("contact_injection_requests", 1),
                ("post_stop_drift_rad", 0.01),
                ("goal_terminal_statuses", [4, 4]),
                ("remaining_motion_at_contact_sec", 0),
                ("run_id", "left"),
            ]:
                broken = deepcopy(good)
                broken[2][field] = value
                write(broken)
                with self.subTest(field=field), self.assertRaises(ValueError):
                    compare_reports(paths)
            write(good)
            with self.assertRaises(ValueError):
                compare_reports(paths[:3])
