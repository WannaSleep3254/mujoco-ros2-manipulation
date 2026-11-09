"""Check the shell launcher's lock and CLI contract without starting ROS."""
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[2]


class ViewerLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="viewer-launcher-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.launcher = self.root / "start-viewer.sh"
        shutil.copy2(ROOT / "start-viewer.sh", self.launcher)
        (self.root / "manipulation_env.sh").write_text(": > env-loaded\n")
        (self.root / "operator_console.py").write_text(textwrap.dedent("""\
            import argparse
            import json
            import os
            import sys

            parser = argparse.ArgumentParser()
            parser.add_argument("--port", type=int)
            parser.add_argument("--domain", type=int)
            arguments, forwarded = parser.parse_known_args()
            print(json.dumps({
                "port": arguments.port,
                "domain": arguments.domain,
                "forwarded": forwarded,
                "argv": sys.argv[1:],
                "lock": os.readlink("/proc/self/fd/9"),
                "ros_log_dir": os.environ["ROS_LOG_DIR"],
            }))
        """))

    def launch(self, *arguments, **overrides):
        environment = os.environ.copy()
        for name in ("VIEWER_PORT", "VIEWER_ROS_DOMAIN"):
            environment.pop(name, None)
        environment.update(overrides)
        return subprocess.run(
            ["/bin/bash", str(self.launcher), *arguments],
            cwd=self.root.parent,
            env=environment,
            text=True,
            capture_output=True,
            timeout=10,
        )

    def successful_launch(self, *arguments, **overrides):
        result = self.launch(*arguments, **overrides)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return json.loads(result.stdout)

    def assert_effective_port(self, result, expected):
        self.assertEqual(result["port"], expected)
        self.assertEqual(Path(result["lock"]), self.root / "runtime" / f"viewer-{expected}.lock")

    def hold_lock(self, port):
        directory = self.root / "runtime"
        directory.mkdir(exist_ok=True)
        stream = (directory / f"viewer-{port}.lock").open("w")
        self.addCleanup(stream.close)
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        return stream

    def test_defaults_use_the_same_listening_and_locked_port(self):
        result = self.successful_launch()
        self.assert_effective_port(result, 8933)
        self.assertEqual(result["domain"], 218)
        self.assertEqual(Path(result["ros_log_dir"]), self.root / "runtime/ros-logs")

    def test_environment_defaults_are_forwarded_and_locked(self):
        result = self.successful_launch(VIEWER_PORT="8940", VIEWER_ROS_DOMAIN="221")
        self.assert_effective_port(result, 8940)
        self.assertEqual(result["domain"], 221)

    def test_final_cli_port_overrides_environment_in_both_forms(self):
        for arguments in (
            ("--port", "8941", "--port=8942"),
            ("--port=8941", "--port", "8942"),
        ):
            with self.subTest(arguments=arguments):
                result = self.successful_launch(*arguments, VIEWER_PORT="8940")
                self.assert_effective_port(result, 8942)

    def test_supported_port_abbreviations_select_the_same_lock(self):
        for option in ("--p", "--po", "--por"):
            for arguments in ((option, "8947"), (option + "=8947",)):
                with self.subTest(arguments=arguments):
                    result = self.successful_launch(*arguments, VIEWER_PORT="8940")
                    self.assert_effective_port(result, 8947)

    def test_explicit_port_overrides_an_invalid_environment_default(self):
        result = self.successful_launch("--port=8943", VIEWER_PORT="invalid")
        self.assert_effective_port(result, 8943)

    def test_invalid_ports_are_rejected_before_loading_the_environment(self):
        cases = (
            (("--port", "1023"), {}),
            (("--port=65536",), {}),
            (("--port", "invalid"), {}),
            (("--port",), {}),
            ((), {"VIEWER_PORT": "invalid"}),
            ((), {"VIEWER_PORT": "0"}),
        )
        for arguments, overrides in cases:
            with self.subTest(arguments=arguments, overrides=overrides):
                result = self.launch(*arguments, **overrides)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((self.root / "env-loaded").exists())
                self.assertFalse((self.root / "runtime").exists())

    def test_same_effective_port_cannot_bypass_a_held_lock(self):
        self.hold_lock(8944)
        for argument in ("--port=8944", "--por=8944"):
            with self.subTest(argument=argument):
                result = self.launch(argument, VIEWER_PORT="8945")
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("Viewer port 8944 is already running", result.stdout + result.stderr)
                self.assertFalse((self.root / "env-loaded").exists())

    def test_distinct_effective_port_ignores_the_environment_ports_lock(self):
        self.hold_lock(8933)
        result = self.successful_launch(
            "--port", "8946", "--domain", "222", "--workcell", "fixture",
            "--ros-grasp-domain", "223", VIEWER_PORT="8933",
        )
        self.assert_effective_port(result, 8946)
        self.assertEqual(result["domain"], 222)
        self.assertEqual(result["forwarded"], ["--workcell", "fixture", "--ros-grasp-domain", "223"])


if __name__ == "__main__":
    unittest.main()
