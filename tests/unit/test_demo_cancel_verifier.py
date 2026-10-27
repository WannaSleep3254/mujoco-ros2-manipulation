"""Cancelled jobs must retain their exact nested report and physical condition."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.operator_console.verify_demos import checked_cancelled_report


class CancelledEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root / "runtime/validation/runs/owned-run/result.json"
        self.path.parent.mkdir(parents=True)
        header = self.root / "native/step_monitor.hpp"
        header.parent.mkdir()
        header.write_text("monitor source fixture")
        physics = dict(schema_version=1, run_id="owned-run", complete=True,
            monitor_source_sha256=file_sha256(header), steps=100, engine_steps=100,
            timestep_sec=.002, simulated_sec=.2, wall_span_sec=.2, paced_real_time_factor=1,
            invalid_values=0, time_discontinuities=0, pair_overflow=0,
            physics_step=dict(count=100, mean_sec=.0001, max_sec=.001),
            monitor_step=dict(count=100, mean_sec=.000001, max_sec=.00001),
            stop_enabled=True, stop_latched=True, contact_steps=1, contacts=2,
            active_contacts=2, stop_step=100, hold_ticks=50, hold_time_sec=.3,
            stop_time_sec=.2, last_state_time_sec=.2, stopped_qpos=[0.] * 12,
            stop_reason="contact",
            pairs=[dict(occurrences=2, bodies=["left_forearm_link", "demo_obstacle"], min_distance_m=-.001)])
        self.value = dict(run_id="owned-run", ros_domain_id=201, status="interrupted",
                          started_utc="2026-10-27T10:01:00+00:00", shutdown_clean=True,
                          process_group_gone=True, failure_stage="approach_fixed_obstacle",
                          error=dict(type="DemoCancelled", message="stop"),
                          physics=physics)
        self.job = dict(status="cancelled", comparison=None, results=[dict(
            variant="contact", run_id="owned-run", report_path=str(self.path))])
        self.started = "2026-10-27T10:00:00+00:00"
        self.write()

    def write(self):
        physics = self.path.parent / "physics.json"
        physics.write_text(json.dumps(self.value["physics"]))
        self.value["physics_report_sha256"] = file_sha256(physics)
        self.path.write_text(json.dumps(self.value))
        self.job["results"][0]["report_sha256"] = file_sha256(self.path)

    def check(self, **options):
        options.setdefault("expected_timestep", .002)
        return checked_cancelled_report(self.job, self.root, 201, self.started, **options)

    def test_latched_cancel_verifies_physical_condition_and_clean_shutdown(self):
        self.assertEqual(self.check(require_latched=True), self.value)

    def test_early_cancel_is_valid_but_does_not_prove_latched_cancel(self):
        self.value["physics"] = dict(complete=True, stop_latched=False, contacts=0, hold_ticks=0)
        self.write()
        self.check()
        with self.assertRaisesRegex(RuntimeError, "latched contact"):
            self.check(require_latched=True)

    def test_stale_wrong_domain_changed_bytes_and_wrong_run_are_rejected(self):
        for field, value in (("started_utc", "2026-10-26T10:00:00+00:00"),
                             ("ros_domain_id", 200), ("run_id", "other")):
            with self.subTest(field=field):
                old = self.value[field]
                self.value[field] = value
                self.write()
                with self.assertRaises(RuntimeError):
                    self.check()
                self.value[field] = old
        self.write()
        self.path.write_text(self.path.read_text() + " ")
        with self.assertRaisesRegex(RuntimeError, "changed"):
            self.check()

    def test_completion_and_cleanup_failures_cannot_pass_as_cancelled(self):
        for field, value in (("status", "passed"), ("shutdown_clean", False),
                             ("process_group_gone", False), ("error", dict(type="RuntimeError"))):
            with self.subTest(field=field):
                original = deepcopy(self.value)
                self.value[field] = value
                self.write()
                with self.assertRaisesRegex(RuntimeError, "clean owned shutdown"):
                    self.check()
                self.value = original

    def test_latch_requires_held_contact_in_expected_stage(self):
        original = deepcopy(self.value)
        for field, value in (("complete", False), ("stop_latched", False),
                             ("contacts", 0), ("hold_ticks", 9)):
            with self.subTest(field=field):
                self.value = deepcopy(original)
                self.value["physics"][field] = value
                self.write()
                with self.assertRaisesRegex(RuntimeError, "latched contact"):
                    self.check(require_latched=True)
        self.value = deepcopy(original)
        self.value["failure_stage"] = "visible_obstacle_discovery"
        self.write()
        with self.assertRaisesRegex(RuntimeError, "latched contact"):
            self.check(require_latched=True)

    def test_wrong_path_completed_comparison_and_next_variant_are_rejected(self):
        original = deepcopy(self.job)
        for change in (dict(comparison={"matched": True}), dict(status="succeeded"),
                       dict(results=[dict(variant="control")])):
            self.job = dict(deepcopy(original), **change)
            with self.assertRaises(RuntimeError):
                self.check()
        self.job = deepcopy(original)
        outside = self.root / "outside.json"
        outside.write_text(json.dumps(self.value))
        self.job["results"][0].update(report_path=str(outside), report_sha256=file_sha256(outside))
        with self.assertRaisesRegex(RuntimeError, "escaped"):
            self.check()

    def test_latched_cancel_rejects_bad_native_coverage_source_and_run(self):
        original = deepcopy(self.value)
        for field, value in (("run_id", "foreign"), ("monitor_source_sha256", "old"),
                             ("invalid_values", 1), ("engine_steps", 99),
                             ("timestep_sec", .003), ("stop_step", 99)):
            with self.subTest(field=field):
                self.value = deepcopy(original)
                self.value["physics"][field] = value
                self.write()
                with self.assertRaises(ValueError):
                    self.check(require_latched=True)

    def test_latched_cancel_requires_exact_physics_bytes_and_content(self):
        physics = self.path.parent / "physics.json"
        physics.write_text(physics.read_text() + " ")
        with self.assertRaisesRegex(RuntimeError, "differs"):
            self.check(require_latched=True)

        self.write()
        other = deepcopy(self.value["physics"])
        other["hold_ticks"] += 1
        physics.write_text(json.dumps(other))
        self.value["physics_report_sha256"] = file_sha256(physics)
        self.path.write_text(json.dumps(self.value))
        self.job["results"][0]["report_sha256"] = file_sha256(self.path)
        with self.assertRaisesRegex(RuntimeError, "differs"):
            self.check(require_latched=True)

    def test_latched_cancel_rejects_other_stop_reason_pair_or_deep_overlap(self):
        original = deepcopy(self.value)
        for field, value in (("stop_reason", "clock_stall"),
                             ("pairs", [dict(occurrences=2, bodies=["left_a", "right_b"], min_distance_m=0)]),
                             ("pairs", [dict(occurrences=2, bodies=["left_a", "demo_obstacle"], min_distance_m=-.02)])):
            with self.subTest(field=field):
                self.value = deepcopy(original)
                self.value["physics"][field] = value
                self.write()
                with self.assertRaisesRegex(RuntimeError, "stop reason or contact pair"):
                    self.check(require_latched=True)


if __name__ == "__main__":
    unittest.main()
