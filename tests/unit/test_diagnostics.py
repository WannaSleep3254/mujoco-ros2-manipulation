"""Missing, stale, partial and contact-bearing native reports cannot pass."""
from copy import deepcopy
import tempfile
from pathlib import Path
import unittest
from mujoco_ros2_manipulation.verification.diagnostics import (
    check_physics,
    process_sample,
)


def clean_report():
    return dict(
        schema_version=1,
        complete=True,
        run_id="run",
        monitor_source_sha256="hash",
        steps=100,
        engine_steps=100,
        timestep_sec=0.002,
        simulated_sec=0.2,
        wall_span_sec=0.2,
        paced_real_time_factor=1,
        invalid_values=0,
        time_discontinuities=0,
        pair_overflow=0,
        contacts=0,
        contact_steps=0,
        active_contacts=0,
        pairs=[],
        physics_step=dict(count=100, mean_sec=0.0001, max_sec=0.001),
        monitor_step=dict(count=100, mean_sec=0.000001, max_sec=0.00001),
    )


class DiagnosticTests(unittest.TestCase):
    def validate(self, data):
        return check_physics(data, run_id="run", source_sha256="hash", timestep=0.002)

    def test_valid_complete_report(self):
        self.validate(clean_report())

    def test_partial_stale_contact_nonfinite_and_missing_steps_rejected(self):
        for field, value in [
            ("complete", False),
            ("run_id", "old"),
            ("monitor_source_sha256", "old"),
            ("engine_steps", 99),
            ("steps", 0),
            ("simulated_sec", 0.199),
            ("simulated_sec", float("nan")),
            ("timestep_sec", 0.003),
            ("invalid_values", 1),
            ("time_discontinuities", 1),
            ("pair_overflow", 1),
            ("contacts", 1),
            ("contact_steps", 1),
            ("active_contacts", 1),
            ("pairs", [{"geoms": ["a", "b"]}]),
            ("physics_step", dict(count=99)),
            ("monitor_step", dict(count=99)),
        ]:
            data = deepcopy(clean_report())
            data[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.validate(data)
        for field in clean_report():
            data = clean_report()
            del data[field]
            with self.subTest(missing=field), self.assertRaises(ValueError):
                self.validate(data)

    def test_process_sampling_excludes_other_groups_and_handles_spaces_in_names(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for pid, group in [(10, 7), (11, 8)]:
                path = root / str(pid)
                path.mkdir()
                fields = ["0"] * 22
                fields[0] = "S"
                fields[2] = str(group)
                fields[11] = "20"
                fields[12] = "5"
                fields[21] = "10"
                (path / "stat").write_text(
                    f"{pid} (name with ) spaces) " + " ".join(fields)
                )
            sample = process_sample(7, root)
            self.assertEqual(set(sample), {10})
            self.assertEqual(sample[10]["cpu_ticks"], 25)
            self.assertGreater(sample[10]["rss_bytes"], 0)


class StopReportTests(unittest.TestCase):
    def stopped_report(self):
        value = clean_report()
        value.update(
            stop_enabled=True,
            stop_latched=True,
            contact_steps=1,
            contacts=1,
            active_contacts=1,
            stop_step=100,
            hold_ticks=20,
            hold_time_sec=0.24,
            stop_time_sec=0.2,
            last_state_time_sec=0.2,
            stopped_qpos=[0.0] * 12,
            pairs=[dict(occurrences=1)],
        )
        return value

    def test_normal_run_requires_ready_native_interlock(self):
        data = clean_report()
        with self.assertRaisesRegex(ValueError, "interlock"):
            check_physics(
                data,
                run_id="run",
                source_sha256="hash",
                timestep=0.002,
                require_interlock=True,
            )
        data.update(stop_enabled=True, stop_latched=False)
        check_physics(
            data,
            run_id="run",
            source_sha256="hash",
            timestep=0.002,
            require_interlock=True,
        )

    def test_expected_contact_requires_latch_coverage_and_hold_ticks(self):
        from mujoco_ros2_manipulation.verification.diagnostics import check_stop_physics

        check_stop_physics(
            self.stopped_report(), run_id="run", source_sha256="hash", timestep=0.002
        )
        for key, value in [
            ("stop_latched", False),
            ("stop_enabled", False),
            ("contact_steps", 2),
            ("active_contacts", 0),
            ("stop_step", 99),
            ("hold_ticks", 0),
            ("hold_time_sec", 0.1),
            ("stop_time_sec", 0.1),
            ("stopped_qpos", []),
            ("stopped_qpos", [float("nan")]),
            ("pairs", [dict(occurrences=2)]),
        ]:
            data = self.stopped_report()
            data[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                check_stop_physics(
                    data, run_id="run", source_sha256="hash", timestep=0.002
                )
