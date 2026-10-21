"""A stale or incomplete HTTP publication must not be reported as deployed."""
import hashlib
from contextlib import redirect_stderr
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from urllib.error import URLError

from mujoco_ros2_manipulation.reporting.published import PublishedReportError, fetch_bytes, main, verify_site


class PublishedReportsTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.manifest = self.root / "site-manifest.json"
        self.contents = {"index.html": b"<html>report index</html>", "sample/report.json": b'{"value":null}'}
        self.data = {"schema_version": 1, "files": {name: {
            "bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()} for name, raw in self.contents.items()}}
        self.save()
        self.base = "https://example.com/robot-reports/"

    def save(self):
        self.manifest.write_text(json.dumps(self.data))

    def fetch(self, url, limit):
        self.assertTrue(url.startswith(self.base))
        name = url[len(self.base):]
        raw = self.manifest.read_bytes() if name == "site-manifest.json" else self.contents[name]
        return raw[:limit]

    def test_all_files_and_project_subpath_are_verified(self):
        result = verify_site(self.base.rstrip("/"), self.manifest, fetch=self.fetch)
        self.assertEqual(result["files"], 3)
        self.assertEqual(result["url"], self.base)

    def test_stale_file_is_not_success_even_if_manifest_matches(self):
        self.contents["sample/report.json"] = b'{"value":0}'
        with self.assertRaisesRegex(PublishedReportError, "does not match"):
            verify_site(self.base, self.manifest, fetch=self.fetch)

    def test_old_manifest_is_rejected_before_loading_other_files(self):
        with self.assertRaisesRegex(PublishedReportError, "site-manifest"):
            verify_site(self.base, self.manifest, fetch=lambda url, limit: b"old publication")

    def test_invalid_paths_cannot_escape_the_site_root(self):
        for name in ("../secret", "/index.html", "x%2fy", "x?query", "x\\y", ".env"):
            self.data["files"] = {name: {"bytes": 0, "sha256": "a" * 64}}
            self.save()
            with self.subTest(name=name), self.assertRaisesRegex(PublishedReportError, "path"):
                verify_site(self.base, self.manifest, fetch=lambda *_: self.fail("No network call expected"))

    def test_invalid_metadata_is_rejected(self):
        self.data["files"]["index.html"]["bytes"] = True
        self.save()
        with self.assertRaisesRegex(PublishedReportError, "metadata"):
            verify_site(self.base, self.manifest, fetch=self.fetch)

    def test_nonobject_manifest_is_rejected_without_network(self):
        self.manifest.write_text('[]')
        with self.assertRaisesRegex(PublishedReportError, "manifest"):
            verify_site(self.base, self.manifest, fetch=lambda *_: self.fail("No network call expected"))

    def test_credentials_or_non_http_url_are_rejected(self):
        for url in ("file:///tmp/site", "https://user:password@example.com/", self.base + "?secret=1"):
            with self.subTest(url=url), self.assertRaisesRegex(PublishedReportError, "HTTP"):
                verify_site(url, self.manifest, fetch=self.fetch)

    def test_http_failure_is_reported(self):
        with patch("mujoco_ros2_manipulation.reporting.published.urlopen", side_effect=URLError("not ready")):
            with self.assertRaisesRegex(PublishedReportError, "Cannot read"):
                fetch_bytes(self.base, 100)

    def test_default_manifest_uses_workspace_even_outside_checkout(self):
        root = Path(__file__).resolve().parents[2]
        previous = Path.cwd()
        try:
            os.chdir(self.root)
            with patch.dict(os.environ, {"MUJOCO_ROS2_WORKSPACE": str(root)}), \
                 patch("mujoco_ros2_manipulation.reporting.published.verify_site", return_value={"files": 3}) as verify, \
                 patch("builtins.print"):
                self.assertEqual(main(["--url", self.base]), 0)
            self.assertEqual(verify.call_args.args[1], root / "docs/reports/site-manifest.json")
        finally:
            os.chdir(previous)

    def test_deployment_propagation_retry_is_bounded(self):
        with patch("mujoco_ros2_manipulation.reporting.published.verify_site", side_effect=[PublishedReportError("stale"), {"files": 3}]) as verify, \
             patch("mujoco_ros2_manipulation.reporting.published.time.sleep") as sleep, patch("builtins.print"):
            self.assertEqual(main(["--url", self.base, "--attempts", "2"]), 0)
        self.assertEqual(verify.call_count, 2)
        sleep.assert_called_once_with(5)

    def test_persistent_failure_cannot_become_success(self):
        with patch("mujoco_ros2_manipulation.reporting.published.verify_site", side_effect=PublishedReportError("stale")), \
             patch("mujoco_ros2_manipulation.reporting.published.time.sleep"), patch("builtins.print"), redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                main(["--url", self.base, "--attempts", "2"])
            self.assertEqual(error.exception.code, 1)
