"""Validation of the benchmark's frozen input, without a GL/ROS dependency."""
import importlib.util
import hashlib
import json
from pathlib import Path
import statistics
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[2] / "scripts/viewer_display_benchmark.py"
spec = importlib.util.spec_from_file_location("viewer_display_benchmark", SOURCE)
benchmark = importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)


class RecordingTests(unittest.TestCase):
    def test_portable_recording_is_moving_finite_and_reproducible(self):
        with tempfile.TemporaryDirectory() as directory:
            model, recording = benchmark.fixture(Path(directory))
            first, digest = benchmark.load_recording(recording)
            _, again = benchmark.load_recording(recording)
            self.assertEqual(digest, again)
            self.assertTrue(model.is_file())
            self.assertNotEqual(first["samples"][0]["positions"], first["samples"][10]["positions"])

    def test_recording_requires_complete_finite_increasing_states(self):
        good = dict(schema_version=1, joint_names=["joint"], samples=[
            dict(time_sec=0, positions=dict(joint=0)), dict(time_sec=1, positions=dict(joint=.1))])
        changes = [lambda v: v["joint_names"].append("joint"),
            lambda v: v["samples"][1].update(time_sec=0),
            lambda v: v["samples"][1].update(time_sec=True),
            lambda v: v["samples"][1]["positions"].update(joint=float("nan")),
            lambda v: v["samples"][1]["positions"].update(extra=0),
            lambda v: v["samples"][1].update(positions={}),
            lambda v: v["samples"].pop()]
        with tempfile.TemporaryDirectory() as directory:
            recording = Path(directory) / "recording.json"
            for change in changes:
                value = json.loads(json.dumps(good))
                change(value)
                recording.write_text(json.dumps(value))
                with self.assertRaises(ValueError):
                    benchmark.load_recording(recording)

    def test_served_page_has_only_local_display_script_and_imports(self):
        html = benchmark.page().decode()
        self.assertNotIn("https://", html)
        self.assertNotIn("api/command", html)
        self.assertIn("/benchmark-page.mjs", html)


class EvidenceTests(unittest.TestCase):
    def test_measured_trials_and_artifacts_remain_consistent(self):
        self.check_evidence("2026-10-26-viewer-moving-display")

    def test_hardware_trials_record_the_actual_renderer(self):
        evidence = self.check_evidence("2026-10-26-viewer-moving-hardware")
        self.assertTrue(evidence["conditions"]["hardware_gl_required"])
        for trial in evidence["trials"]:
            self.assertTrue(trial["hardware_gl_required"])
            self.assertIn("--disable-software-rasterizer", trial["browser_args"])
            renderer = trial["snapshot"].get("gl") or trial["snapshot"]["config"]["native_gl"]
            self.assertIn("NVIDIA", renderer["renderer"])

    def check_evidence(self, directory):
        root = SOURCE.parents[1] / "docs/reports" / directory
        evidence = json.loads((root / "evidence.json").read_text())
        for name, expected in evidence["artifact_sha256"].items():
            self.assertEqual(hashlib.sha256((root / name).read_bytes()).hexdigest(), expected)
        trials = evidence["trials"]
        self.assertEqual(len(trials), 7)
        self.assertEqual(len({r["independent_browser_pid"] for r in trials}), 7)
        self.assertEqual(len({r["independent_server_pid"] for r in trials}), 7)
        for trial in trials:
            config = trial["snapshot"]["config"]
            self.assertEqual(config["model_sha256"], evidence["model_sha256"])
            self.assertEqual(config["record_sha256"], evidence["recording_sha256"])
            self.assertEqual(trial["snapshot"]["canvas_pixels"], dict(width=640, height=480))
            self.assertEqual(trial["snapshot"]["dpr"], 1)
            self.assertFalse(trial["page_errors"])
            self.assertFalse(trial["snapshot"]["errors"])
        soak = trials[-1]
        self.assertTrue(soak["soak"])
        self.assertGreaterEqual(soak["duration_sec"], 120)
        self.assertEqual(len(soak["stability"]["checks"]), 10)
        self.assertTrue(all(check["pass"] for check in soak["stability"]["checks"]))
        for mode in ("image", "web3d"):
            values = [r for r in trials if r["mode"] == mode and not r["soak"]]
            self.assertEqual(len(values), 3)
            for metric in ("frames_per_sec", "unique_pose_or_image_per_sec", "response_body_bytes_per_sec"):
                self.assertAlmostEqual(statistics.median(r[metric] for r in values), evidence["groups"][mode][metric]["median"])
        return evidence


if __name__ == "__main__":
    unittest.main()
