"""Publication integrity, clock/sample contracts, and portable site boundaries."""
import copy
import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from mujoco_ros2_manipulation.reporting.data import ReportError, json_text, load_report, read_json, validate
from mujoco_ros2_manipulation.reporting.phase1 import phase1_report
from mujoco_ros2_manipulation.reporting.publish import build, collect, export_site
from mujoco_ros2_manipulation.reporting.render import metrics_csv, report_html

ROOT = Path(__file__).resolve().parents[2]


class PublicationReportTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.temporary = Path(temporary.name)
        self.root = self.temporary / "reports"
        self.directory = self.root / "sample"
        self.directory.mkdir(parents=True)
        self.asset = self.directory / "evidence.txt"
        self.asset.write_text("test-only measurement fixture\n")
        self.data = dict(schema_version=1, report_id="sample", title="시험 리포트", published_date="2026-10-21",
            status="preliminary", overview="테스트 픽스처. 실제 로봇 성능 아님.", findings=["계약 검증"], limitations=["측정 아님"],
            provenance=dict(repository="https://example.com/repo", commit="a" * 40, workcell="test",
                            models=["test"], scenario="test", input_kind="other"),
            environment=dict(os="test", cpu="test", python="test", ros="test", native_mujoco="test"),
            sample_groups=[dict(id="trial", label="test", kind="offline", unique_cases=1, observations=1,
                successes=1, failures=0, seed_policy="fixed fixture", description="테스트 전용")],
            stages=[dict(id="compute", label="계산", clock="wall", coverage="measured", note="fixture")],
            metrics=[dict(id="latency", label="지연", unit="s", clock="wall", scope="test", note="fixture",
                values=[dict(group_id="trial", value=2.0, sample_count=1, statistic="single")])],
            assets=[dict(path="evidence.txt", label="근거", role="data", sha256=hashlib.sha256(self.asset.read_bytes()).hexdigest())],
            references=[])
        self.save()

    def save(self):
        path = self.directory / "report.json"
        path.write_text(json_text(self.data))
        return path

    def test_repeated_build_is_deterministic_and_check_does_not_write(self):
        build(self.root)
        before = {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in self.root.rglob("*") if p.is_file()}
        build(self.root)
        build(self.root, check=True)
        self.assertEqual(before, {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in before})

    def test_stale_html_is_rejected_without_overwriting_it(self):
        build(self.root)
        html = self.directory / "index.html"
        html.write_text("stale output")
        with self.assertRaisesRegex(ReportError, "stale"):
            build(self.root, check=True)
        self.assertEqual(html.read_text(), "stale output")

    def test_tampered_evidence_cannot_be_published(self):
        self.asset.write_text("replaced after review")
        with self.assertRaisesRegex(ReportError, "hash mismatch"):
            build(self.root)
        self.assertFalse((self.root / "index.html").exists())

    def test_site_exports_only_declared_files_and_records_exact_hashes(self):
        (self.directory / "private.env").write_text("do not publish")
        (self.root / "local-notes.txt").write_text("not in catalog")
        build(self.root)
        output = self.temporary / "site"
        export_site(self.root, output)
        self.assertFalse((output / "sample/private.env").exists())
        self.assertFalse((output / "local-notes.txt").exists())
        self.assertEqual((output / "sample/evidence.txt").read_bytes(), self.asset.read_bytes())
        manifest = read_json(output / "site-manifest.json")
        for name, metadata in manifest["files"].items():
            self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), metadata["sha256"])

    def test_existing_site_directory_is_preserved(self):
        build(self.root)
        output = self.temporary / "existing"
        output.mkdir()
        sentinel = output / "important.txt"
        sentinel.write_text("keep")
        with self.assertRaisesRegex(ReportError, "new directory"):
            export_site(self.root, output)
        self.assertEqual(sentinel.read_text(), "keep")

    def test_export_cannot_write_inside_publication_sources(self):
        build(self.root)
        with self.assertRaisesRegex(ReportError, "separate"):
            export_site(self.root, self.root / "site")

    def test_asset_traversal_and_absolute_paths_are_rejected(self):
        for path in ("../outside.txt", "/tmp/outside.txt", "a/../b", "a\\b", "a%2fb", "a?download=1", ".env"):
            with self.subTest(path=path):
                changed = copy.deepcopy(self.data)
                changed["assets"][0]["path"] = path
                with self.assertRaises(ReportError):
                    validate(changed, self.directory)

    def test_symlink_asset_cannot_escape_publication_directory(self):
        self.asset.unlink()
        outside = self.temporary / "outside.txt"
        outside.write_text("outside")
        self.asset.symlink_to(outside)
        with self.assertRaisesRegex(ReportError, "Symlink"):
            build(self.root)

    def test_unknown_group_and_duplicate_metric_ids_are_rejected(self):
        self.data["metrics"][0]["values"][0]["group_id"] = "unknown"
        with self.assertRaisesRegex(ReportError, "unknown"):
            validate(self.data, self.directory)
        self.data["metrics"][0]["values"][0]["group_id"] = "trial"
        self.data["metrics"].append(copy.deepcopy(self.data["metrics"][0]))
        with self.assertRaisesRegex(ReportError, "Duplicate"):
            validate(self.data, self.directory)

    def test_missing_clock_wrong_units_and_invalid_dates_are_rejected(self):
        for field, value in (("clock", "not_applicable"), ("unit", "rad")):
            changed = copy.deepcopy(self.data)
            changed["metrics"][0][field] = value
            with self.assertRaisesRegex(ReportError, "Time units"):
                validate(changed, self.directory)
        del self.data["metrics"][0]["clock"]
        with self.assertRaisesRegex(ReportError, "Schema"):
            validate(self.data, self.directory)
        self.data["metrics"][0]["clock"] = "wall"
        self.data["published_date"] = "2026-02-30"
        with self.assertRaisesRegex(ReportError, "Schema"):
            validate(self.data, self.directory)

    def test_unmeasured_is_null_and_cannot_look_like_zero_latency(self):
        item = self.data["metrics"][0]["values"][0]
        item.update(value=None, sample_count=0, statistic="not_measured")
        validate(self.data, self.directory)
        self.assertIn("미측정", report_html(self.data))
        item["value"] = 0
        with self.assertRaisesRegex(ReportError, "finite number and samples"):
            validate(self.data, self.directory)

    def test_all_failures_must_be_counted(self):
        self.data["sample_groups"][0]["observations"] = 2
        with self.assertRaisesRegex(ReportError, "include every observation"):
            validate(self.data, self.directory)

    def test_sample_overclaim_and_small_tail_statistics_are_rejected(self):
        item = self.data["metrics"][0]["values"][0]
        item.update(sample_count=5, statistic="median")
        with self.assertRaisesRegex(ReportError, "exceeds"):
            validate(self.data, self.directory)
        self.data["sample_groups"][0].update(observations=5, successes=None, failures=None)
        item["statistic"] = "p95"
        with self.assertRaisesRegex(ReportError, "Tail quantiles"):
            validate(self.data, self.directory)

    def test_html_and_csv_do_not_execute_supplied_text(self):
        self.data["title"] = '</title><script>alert("x")</script>'
        self.data["metrics"][0]["note"] = '=HYPERLINK("https://example.com")'
        html = report_html(validate(self.data, self.directory))
        self.assertNotIn('<script>alert("x")', html)
        self.assertIn('&lt;script&gt;', html)
        row = list(csv.DictReader(io.StringIO(metrics_csv(self.data))))[0]
        self.assertTrue(row["note"].startswith("'="))
        self.data["references"] = [dict(label="bad", url="javascript:alert(1)")]
        with self.assertRaisesRegex(ReportError, "Schema"):
            validate(self.data, self.directory)

    def test_duplicate_and_nonfinite_json_is_rejected(self):
        path = self.directory / "invalid.json"
        for content in ('{"a":1,"a":2}', '{"a":NaN}'):
            path.write_text(content)
            with self.assertRaises(ReportError):
                read_json(path)
        self.data["metrics"][0]["values"][0]["value"] = float("inf")
        with self.assertRaisesRegex(ReportError, "finite"):
            validate(self.data, self.directory)

    def test_generated_names_cannot_be_claimed_as_assets(self):
        self.data["assets"][0]["path"] = "index.html"
        with self.assertRaisesRegex(ReportError, "conflicts"):
            validate(self.data, self.directory)

    def test_broken_legacy_links_prevent_site_publication(self):
        path = self.directory / "legacy.html"
        path.write_text('<img src="missing.png">')
        self.data["assets"].append(dict(path="legacy.html", label="legacy", role="legacy_report",
            sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        self.save()
        with self.assertRaisesRegex(ReportError, "Broken HTML link"):
            build(self.root)

    def test_missing_or_cyclic_revision_links_are_rejected(self):
        self.data["supersedes"] = "not-present"
        self.save()
        with self.assertRaisesRegex(ReportError, "supersedes"):
            collect(self.root)
        self.data["supersedes"] = "sample"
        self.save()
        with self.assertRaisesRegex(ReportError, "supersedes"):
            collect(self.root)


class PhaseOneCompatibilityTests(unittest.TestCase):
    def test_standard_input_matches_preserved_measurements(self):
        directory = ROOT / "docs/reports/2026-10-21-phase1"
        result = phase1_report(directory)
        self.assertEqual(result, load_report(directory / "report.json"))
        metrics = {m["id"]: m for m in result["metrics"]}
        self.assertEqual(metrics["total-with-cleanup-sec"]["values"][0]["value"], 2.133431133)
        self.assertEqual(metrics["command-to-motion"]["values"][0]["value"], None)
        self.assertEqual(metrics["motion"]["clock"], "simulation")
        self.assertEqual(metrics["motion"]["values"][1]["value"], 8.621950734)
        self.assertEqual(metrics["observed-gap"]["values"][1]["value"], 30.64359811263672)

    def test_original_html_and_archived_assets_are_unchanged(self):
        directory = ROOT / "docs/reports/2026-10-21-phase1"
        original = read_json(directory / "manifest.json")
        for name, metadata in original["files"].items():
            self.assertEqual(hashlib.sha256((directory / name).read_bytes()).hexdigest(), metadata["sha256"], name)
