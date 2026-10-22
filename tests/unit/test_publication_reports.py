"""Publication integrity, clock/sample contracts, and portable site boundaries."""
import copy
import csv
import hashlib
import io
import json
from pathlib import Path
from html.parser import HTMLParser
import tempfile
import unittest

from mujoco_ros2_manipulation.reporting.data import ReportError, json_text, load_report, read_json, validate
from mujoco_ros2_manipulation.reporting.phase1 import phase1_report
from mujoco_ros2_manipulation.reporting.operator_demos import collect_demos
from mujoco_ros2_manipulation.reporting.publish import build, collect, export_site
from mujoco_ros2_manipulation.reporting.render import catalog_html, metrics_csv, report_html

ROOT = Path(__file__).resolve().parents[2]


class ReportLinks(HTMLParser):
    def __init__(self, content):
        super().__init__()
        self.links = []
        self.reports = 0
        self.sections = []
        self.article_sections = []
        self.section_stack = []
        self.feed(content)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == "a":
            self.links.append(attrs)
        if "data-report" in attrs:
            self.reports += 1
        if tag == "section":
            self.sections.append(attrs.get("id"))
            self.section_stack.append(attrs.get("id"))
        for marker in ("data-demo", "data-design-report"):
            if marker in attrs:
                self.article_sections.append((marker, self.section_stack[-1]))

    def handle_endtag(self, tag):
        if tag == "section":
            self.section_stack.pop()


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

    def demo_fixture(self):
        demo = self.temporary / "operator" / "demo"
        demo.mkdir(parents=True, exist_ok=True)
        (demo / "index.html").write_text('<h1>Demo</h1><video src="motion.mp4"></video><a href="manifest.json">Evidence</a>')
        (demo / "motion.mp4").write_bytes(b"test-only media fixture")
        self.demo_manifest = {"schema_version": 1, "assets": {
            name: hashlib.sha256((demo / name).read_bytes()).hexdigest()
            for name in ("index.html", "motion.mp4")}}
        self.demo_registry = {"schema_version": 1, "demos": [dict(
            id="demo", title="시연 <자료>", published_date="2026-10-22",
            overview="단일 기능 사례이며 성능 표본에 합산하지 않음", manifest_sha256="0" * 64)]}
        self.save_demo(demo)
        return demo

    def save_demo(self, demo):
        manifest = demo / "manifest.json"
        manifest.write_text(json_text(self.demo_manifest))
        self.demo_registry["demos"][0]["manifest_sha256"] = hashlib.sha256(manifest.read_bytes()).hexdigest()
        (self.root / "operator-demos.json").write_text(json_text(self.demo_registry))

    def test_demo_export_keeps_measured_catalog_separate_and_uses_reviewed_allowlist(self):
        demo = self.demo_fixture()
        (demo / "private.env").write_text("not reviewed")
        unlisted = demo.parent / "unreviewed"
        unlisted.mkdir()
        (unlisted / "index.html").write_text("do not publish")
        build(self.root)
        output = self.temporary / "demo-site"
        export_site(self.root, output)
        self.assertEqual(len(read_json(output / "catalog.json")["reports"]), 1)
        catalog = ReportLinks((output / "index.html").read_text())
        self.assertEqual(catalog.reports, 1)
        self.assertEqual(catalog.article_sections, [("data-demo", "demos")])
        self.assertNotIn("design-reports", catalog.sections)
        self.assertTrue(any(a["href"] == "operator/demo/index.html" for a in catalog.links))
        self.assertIn("시연 &lt;자료&gt;", (output / "index.html").read_text())
        names = {str(p.relative_to(output)) for p in output.rglob("*") if p.is_file()}
        self.assertEqual({n for n in names if n.startswith("operator/")}, {
            "operator/demo/index.html", "operator/demo/motion.mp4", "operator/demo/manifest.json"})
        manifest = read_json(output / "site-manifest.json")
        self.assertEqual(set(manifest["files"]), names - {"site-manifest.json"})
        for name, metadata in manifest["files"].items():
            raw = (output / name).read_bytes()
            self.assertEqual(metadata, dict(bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest()))
        build(self.root, check=True)

    def test_legacy_and_explicit_functional_demo_types_are_compatible(self):
        demo = self.demo_fixture()
        for registry_type, manifest_type in ((None, None), ("functional_demo", None),
                                             (None, "functional_demo"), ("functional_demo", "functional_demo")):
            with self.subTest(registry_type=registry_type, manifest_type=manifest_type):
                for target, value in ((self.demo_registry["demos"][0], registry_type),
                                      (self.demo_manifest, manifest_type)):
                    target.pop("document_type", None)
                    if value is not None:
                        target["document_type"] = value
                self.save_demo(demo)
                entries, _ = collect_demos(self.root)
                self.assertEqual(entries[0]["document_type"], "functional_demo")

    def test_registry_rejects_unknown_document_types(self):
        demo = self.demo_fixture()
        for value in ("performance_report", "", None, 1, []):
            with self.subTest(document_type=value):
                self.demo_registry["demos"][0]["document_type"] = value
                self.save_demo(demo)
                with self.assertRaisesRegex(ReportError, "Demo registry schema"):
                    collect_demos(self.root)

    def test_document_types_must_match_the_reviewed_manifest(self):
        demo = self.demo_fixture()
        for registry_type, manifest_type in ((None, "design_review"), ("functional_demo", "design_review"),
                                             ("design_review", None), ("design_review", "functional_demo")):
            with self.subTest(registry_type=registry_type, manifest_type=manifest_type):
                for target, value in ((self.demo_registry["demos"][0], registry_type),
                                      (self.demo_manifest, manifest_type)):
                    target.pop("document_type", None)
                    if value is not None:
                        target["document_type"] = value
                self.save_demo(demo)
                with self.assertRaisesRegex(ReportError, "document type mismatch"):
                    collect_demos(self.root)
        self.demo_registry["demos"][0].pop("document_type", None)
        for value in ("performance_report", None, 1, ["design_review"]):
            with self.subTest(manifest_type=value):
                self.demo_manifest["document_type"] = value
                self.save_demo(demo)
                with self.assertRaisesRegex(ReportError, "Unsupported demo manifest document type"):
                    collect_demos(self.root)

    def test_design_review_export_has_its_own_section_and_preserves_measured_catalog(self):
        self.demo_fixture()
        review = self.temporary / "operator" / "design-review"
        review.mkdir()
        review_html = b'<h1>Design review</h1><a href="manifest.json">Evidence</a>'
        (review / "index.html").write_bytes(review_html)
        review_manifest = json_text(dict(schema_version=1, document_type="design_review",
            assets={"index.html": hashlib.sha256(review_html).hexdigest()})).encode()
        (review / "manifest.json").write_bytes(review_manifest)
        (review / "private.env").write_text("not reviewed")
        self.demo_registry["demos"].append(dict(id="design-review", document_type="design_review",
            title='설계 <자료> & "검토"', published_date="2026-10-22",
            overview='<img src=x onerror="alert(1)">',
            manifest_sha256=hashlib.sha256(review_manifest).hexdigest()))
        (self.root / "operator-demos.json").write_text(json_text(self.demo_registry))
        entries, _ = collect_demos(self.root)
        self.assertEqual({entry["id"]: entry["document_type"] for entry in entries},
                         {"demo": "functional_demo", "design-review": "design_review"})
        build(self.root)
        output = self.temporary / "design-site"
        export_site(self.root, output)
        self.assertEqual([entry["report_id"] for entry in read_json(output / "catalog.json")["reports"]],
                         ["sample"])
        page = (output / "index.html").read_text()
        catalog = ReportLinks(page)
        self.assertEqual(catalog.reports, 1)
        self.assertEqual(catalog.article_sections,
                         [("data-demo", "demos"), ("data-design-report", "design-reports")])
        self.assertTrue(any(a["href"] == "#design-reports" for a in catalog.links))
        self.assertTrue(any(a["href"] == "operator/design-review/index.html" for a in catalog.links))
        self.assertIn('<section id="design-reports"><h2>설계·개발 리포트</h2>', page)
        self.assertIn('설계 &lt;자료&gt; &amp; &quot;검토&quot;', page)
        self.assertIn('&lt;img src=x onerror=&quot;alert(1)&quot;&gt;', page)
        self.assertNotIn('<img src=x', page)
        self.assertEqual((output / "operator/design-review/index.html").read_bytes(), review_html)
        self.assertEqual((output / "operator/design-review/manifest.json").read_bytes(), review_manifest)
        self.assertFalse((output / "operator/design-review/private.env").exists())
        build(self.root, check=True)

    def test_design_review_links_and_text_are_escaped_without_a_demo_section(self):
        url = 'operator/review/index.html?title="detail"&next=<review>'
        page = catalog_html([], [dict(document_type="design_review", url=url,
            title='검토 </a><script>alert("x")</script>', published_date="2026-10-22",
            overview='내용 <img src=x onerror="alert(1)">')])
        catalog = ReportLinks(page)
        self.assertEqual(catalog.reports, 0)
        self.assertEqual(catalog.article_sections, [("data-design-report", "design-reports")])
        self.assertNotIn("demos", catalog.sections)
        self.assertFalse(any(a["href"] == "#demos" for a in catalog.links))
        self.assertTrue(any(a["href"] == url for a in catalog.links))
        self.assertIn('href="operator/review/index.html?title=&quot;detail&quot;&amp;next=&lt;review&gt;"', page)
        self.assertNotIn('<script>alert("x")</script>', page)
        self.assertNotIn('<img src=x', page)

    def test_demo_cannot_overwrite_an_existing_report_asset(self):
        self.demo_fixture()
        new_directory = self.root / "operator"
        self.directory.rename(new_directory)
        self.directory = new_directory
        self.data["report_id"] = "operator"
        asset = self.directory / "demo/index.html"
        asset.parent.mkdir()
        asset.write_text("<p>Preserve reviewed report asset</p>")
        self.data["assets"].append(dict(path="demo/index.html", label="report evidence", role="legacy_report",
            sha256=hashlib.sha256(asset.read_bytes()).hexdigest()))
        self.save()
        with self.assertRaisesRegex(ReportError, "Duplicate publication paths"):
            collect(self.root)
        self.assertIn("Preserve reviewed report asset", asset.read_text())

    def test_demo_manifest_and_video_tampering_block_publication(self):
        demo = self.demo_fixture()
        (demo / "manifest.json").write_text(json_text({**self.demo_manifest, "extra": "changed after review"}))
        with self.assertRaisesRegex(ReportError, "manifest hash mismatch"):
            collect(self.root)
        self.save_demo(demo)
        (demo / "motion.mp4").write_bytes(b"replaced video")
        with self.assertRaisesRegex(ReportError, "asset hash mismatch"):
            collect(self.root)

    def test_demo_registry_rejects_duplicate_ids_and_unrecognized_fields(self):
        self.demo_fixture()
        self.demo_registry["demos"].append(copy.deepcopy(self.demo_registry["demos"][0]))
        path = self.root / "operator-demos.json"
        path.write_text(json_text(self.demo_registry))
        with self.assertRaisesRegex(ReportError, "Duplicate demo IDs"):
            collect(self.root)
        self.demo_registry["demos"].pop()
        self.demo_registry["demos"][0]["source_path"] = "/tmp/unreviewed"
        path.write_text(json_text(self.demo_registry))
        with self.assertRaisesRegex(ReportError, "Demo registry schema"):
            collect(self.root)

    def test_demo_ids_and_dates_cannot_escape_registered_directories(self):
        demo = self.demo_fixture()
        for field, value in (("id", "../demo"), ("id", "/tmp/demo"), ("id", "demo\n"),
                             ("published_date", "2026-02-30"), ("manifest_sha256", "bad")):
            with self.subTest(field=field, value=value):
                changed = copy.deepcopy(self.demo_registry)
                changed["demos"][0][field] = value
                (self.root / "operator-demos.json").write_text(json_text(changed))
                with self.assertRaisesRegex(ReportError, "Demo registry schema"):
                    collect(self.root)
        self.save_demo(demo)

    def test_demo_asset_paths_and_symlinked_assets_are_rejected(self):
        demo = self.demo_fixture()
        original = copy.deepcopy(self.demo_manifest)
        for name in ("../outside.txt", "/tmp/outside.txt", "nested/../../outside.txt", "a%2Fb", "x#fragment"):
            with self.subTest(name=name):
                self.demo_manifest = copy.deepcopy(original)
                self.demo_manifest["assets"][name] = "a" * 64
                self.save_demo(demo)
                with self.assertRaisesRegex(ReportError, "relative publication path"):
                    collect(self.root)
        self.demo_manifest = original
        self.save_demo(demo)
        (demo / "motion.mp4").unlink()
        (demo / "motion.mp4").symlink_to(self.asset)
        with self.assertRaisesRegex(ReportError, "Symlink"):
            collect(self.root)

    def test_symlinked_demo_registry_parent_directory_and_manifest_are_rejected(self):
        demo = self.demo_fixture()
        for path in (self.root / "operator-demos.json", demo / "manifest.json", demo, demo.parent):
            with self.subTest(path=path):
                saved = path.with_name(path.name + "-saved")
                path.rename(saved)
                path.symlink_to(saved, target_is_directory=saved.is_dir())
                try:
                    with self.assertRaisesRegex(ReportError, "[Ss]ymlink"):
                        collect(self.root)
                finally:
                    path.unlink()
                    saved.rename(path)

    def test_demo_requires_entrypoint_known_version_and_valid_digests(self):
        demo = self.demo_fixture()
        original = copy.deepcopy(self.demo_manifest)
        variants = [dict(schema_version=2, assets=original["assets"]),
                    dict(schema_version=True, assets=original["assets"]),
                    dict(schema_version=1, assets={"motion.mp4": "a" * 64}),
                    dict(schema_version=1, assets={**original["assets"], "manifest.json": "a" * 64}),
                    dict(schema_version=1, assets={**original["assets"], "motion.mp4": 123})]
        for variant in variants:
            with self.subTest(variant=variant):
                self.demo_manifest = variant
                self.save_demo(demo)
                with self.assertRaises(ReportError):
                    collect(self.root)

    def test_demo_missing_assets_and_nonportable_links_block_export(self):
        demo = self.demo_fixture()
        (demo / "motion.mp4").unlink()
        with self.assertRaisesRegex(ReportError, "Missing asset"):
            collect(self.root)
        (demo / "motion.mp4").write_bytes(b"test-only media fixture")
        for url in ("../../../README.md", "http://127.0.0.1:8765/", "missing.html"):
            with self.subTest(url=url):
                (demo / "index.html").write_text('<a href="' + url + '">bad link</a>')
                self.demo_manifest["assets"]["index.html"] = hashlib.sha256((demo / "index.html").read_bytes()).hexdigest()
                self.save_demo(demo)
                with self.assertRaisesRegex(ReportError, "Nonportable|Broken HTML"):
                    collect(self.root)

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

    def test_details_and_video_are_directly_reachable_from_catalog_and_header(self):
        fixtures = [
            ("details/상세 report.html", "영상·분석 <원본>", "legacy_report", b"<p>test-only detail</p>"),
            ("extra.html", "추가 분석", "legacy_report", b"<p>test-only extra</p>"),
            ("motion sample.mp4", "시험 영상", "video", b"test-only media link fixture"),
        ]
        for name, label, role, content in fixtures:
            path = self.directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
            self.data["assets"].append(dict(path=name, label=label, role=role,
                sha256=hashlib.sha256(content).hexdigest()))
        self.save()
        build(self.root)
        output = self.temporary / "site"
        export_site(self.root, output)
        entry = read_json(output / "catalog.json")["reports"][0]
        expected = [
            ("legacy_report", "sample/details/%EC%83%81%EC%84%B8%20report.html"),
            ("legacy_report", "sample/extra.html"),
            ("video", "sample/motion%20sample.mp4"),
        ]
        self.assertEqual([(item["kind"], item["url"]) for item in entry["resources"]], expected)
        catalog = ReportLinks((output / "index.html").read_text())
        self.assertEqual(catalog.reports, 1)
        self.assertEqual([a["href"] for a in catalog.links if a.get("data-report-link") == "summary"],
                         ["sample/index.html"])
        self.assertEqual([(a["data-report-resource"], a["href"]) for a in catalog.links
                          if "data-report-resource" in a], expected)
        summary = (output / "sample/index.html").read_text()
        header = ReportLinks(summary.split("</header>")[0])
        self.assertEqual([(a["data-report-resource"], a["href"]) for a in header.links
                          if "data-report-resource" in a],
                         [(kind, url.removeprefix("sample/")) for kind, url in expected])
        self.assertIn("영상·분석 &lt;원본&gt;", summary)
        self.assertNotIn("영상·분석 <원본>", summary)

    def test_reports_without_details_do_not_advertise_missing_resources(self):
        build(self.root)
        self.assertEqual(read_json(self.root / "catalog.json")["reports"][0]["resources"], [])
        for path in (self.root / "index.html", self.directory / "index.html"):
            self.assertFalse(any("data-report-resource" in a for a in ReportLinks(path.read_text()).links))
        self.assertTrue(any(a.get("data-report-link") == "summary"
                            for a in ReportLinks((self.root / "index.html").read_text()).links))

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
