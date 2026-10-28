"""Fixed loopback demo admission, source binding and owned process lifecycle."""
from copy import deepcopy
import hashlib
import http.client
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.grasping.demo_server import DemoApplication, serve
from mujoco_ros2_manipulation.grasping.fr5_model import ARM_JOINTS
from mujoco_ros2_manipulation.grasping.fr5_run import FR5Prototype
from mujoco_ros2_manipulation.grasping.live_worker import ENGINE, preserve_sources, source_digests, source_documents
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.reports import atomic_json


class HandlerApplication:
    """No process or physics; these tests exercise the actual HTTP boundary."""
    def __init__(self):
        self.calls = []
        self.source = dict(model_sha256="a"*64)

    def snapshot(self):
        return dict(schema_version=1, source=self.source, job=None, view=None)

    def start(self, value):
        if type(value) is not dict or set(value) != {"request_id", "model_sha256"}:
            raise ValueError("Fixed fields required")
        if value["model_sha256"] != self.source["model_sha256"]:
            raise ValueError("Wrong fixed model")
        self.calls.append(("start", deepcopy(value)))
        return self.snapshot()

    def cancel(self, value):
        if type(value) is not dict or set(value) != {"job_id", "generation", "model_sha256"}:
            raise ValueError("Fixed cancel fields required")
        self.calls.append(("cancel", deepcopy(value)))
        return self.snapshot()

    def manifest_bytes(self, identifier):
        if identifier != "b"*64:
            raise ValueError("Wrong binding")
        return b"{}"

    pose_envelope = manifest_bytes

    def asset_bytes(self, digest):
        if digest != "c"*64:
            raise ValueError("Unknown asset")
        return b"fixed-asset"


class GraspHTTPBoundaryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = HandlerApplication()
        cls.server = serve(cls.app, 0)
        cls.port = cls.server.server_address[1]
        cls.origin = "http://127.0.0.1:"+str(cls.port)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def setUp(self):
        self.app.calls.clear()

    def request(self, method, path, value=None, *, headers=None, raw=None):
        supplied = {} if headers is None else dict(headers)
        if method == "POST":
            supplied = dict({"Origin": self.origin, "X-Operator-Token": self.server.token,
                             "Content-Type": "application/json"}, **supplied)
        body = raw if raw is not None else json.dumps(value).encode() if value is not None else None
        client = http.client.HTTPConnection("127.0.0.1", self.port, timeout=4)
        try:
            client.request(method, path, body, supplied)
            response = client.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            client.close()

    def test_state_contains_a_token_and_reuses_the_strict_response_policy(self):
        status, headers, raw = self.request("GET", "/api/demo")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(raw)["token"], self.server.token)
        self.assertEqual(headers["Cache-Control"], "no-store")
        self.assertEqual(headers["X-Frame-Options"], "DENY")
        self.assertIn("script-src 'self'", headers["Content-Security-Policy"])
        self.assertNotIn("Access-Control-Allow-Origin", headers)

    def test_wrong_host_and_foreign_origin_are_rejected(self):
        for headers in ({"Host": "attacker.invalid"}, {"Origin": "https://attacker.invalid"}):
            with self.subTest(headers=headers):
                self.assertEqual(self.request("GET", "/api/demo", headers=headers)[0], 403)

    def test_wrong_post_origin_and_token_never_admit_a_request(self):
        body = dict(request_id="1"*32, model_sha256="a"*64)
        for headers in ({"Origin": "https://attacker.invalid"}, {"Origin": ""},
                        {"X-Operator-Token": "wrong"}, {"X-Operator-Token": "é"}):
            with self.subTest(headers=headers):
                self.assertEqual(self.request("POST", "/api/demo/start", body, headers=headers)[0], 403)
        self.assertEqual(self.app.calls, [])

    def test_fixed_start_is_the_only_supported_start_request(self):
        body = dict(request_id="1"*32, model_sha256="a"*64)
        self.assertEqual(self.request("POST", "/api/demo/start", body)[0], 200)
        self.assertEqual(self.app.calls, [("start", body)])
        self.app.calls.clear()
        for value in (dict(body, goal=[1, 2, 3]), dict(body, path="/tmp/file"),
                      dict(body, model_sha256="0"*64), []):
            with self.subTest(value=value):
                self.assertEqual(self.request("POST", "/api/demo/start", value)[0], 400)
        self.assertEqual(self.app.calls, [])

    def test_invalid_json_limits_and_duplicate_fields_are_rejected(self):
        for raw, headers in ((b'{"request_id":"a","request_id":"b"}', {}),
                             (b'{"value":NaN}', {}), (b'\xff', {}),
                             (b'{}', {"Content-Length": "1025"}),
                             (b'{}', {"Content-Type": "text/plain"}),
                             (b'{}', {"Transfer-Encoding": "chunked"})):
            with self.subTest(raw=raw, headers=headers):
                self.assertEqual(self.request("POST", "/api/demo/start", raw=raw, headers=headers)[0], 400)
        self.assertEqual(self.app.calls, [])

    def test_other_control_routes_and_query_parameters_are_not_supported(self):
        for method, path in (("POST", "/api/recording/control"), ("POST", "/api/demo/start?path=x"),
                             ("GET", "/api/demo/start"), ("GET", "/api/demo?path=x"),
                             ("GET", "/run/result.json")):
            with self.subTest(method=method, path=path):
                self.assertEqual(self.request(method, path, {})[0], 404)
        self.assertEqual(self.app.calls, [])

    def test_binding_and_asset_allowlists_do_not_become_filesystem_paths(self):
        self.assertEqual(self.request("GET", "/api/view/manifest?binding_id="+"b"*64)[0], 200)
        for path in ("/api/view/pose", "/api/view/pose?binding_id=x", "/api/view/pose?binding_id=x&path=y",
                     "/api/view/assets/../../README.md", "/api/view/assets/%2e%2e/private",
                     "/api/view/assets/"+"0"*64):
            with self.subTest(path=path):
                self.assertEqual(self.request("GET", path)[0], 409)
        status, _, raw = self.request("GET", "/api/view/assets/"+"c"*64)
        self.assertEqual((status, raw), (200, b"fixed-asset"))

    def test_only_local_static_assets_are_served_with_module_rewrites(self):
        self.assertEqual(self.request("GET", "/")[0], 200)
        status, headers, raw = self.request("GET", "/demo.js")
        self.assertEqual(status, 200)
        self.assertIn("text/javascript", headers["Content-Type"])
        self.assertIn(b"LocalSceneView", raw)
        for path, expected in (("/viewer.js", b"from './vendor/three.module.js'"),
                               ("/vendor/OrbitControls.js", b"from './three.module.js'")):
            status, _, raw = self.request("GET", path)
            self.assertEqual(status, 200)
            self.assertIn(expected, raw)
            self.assertNotIn(b"from 'three'", raw)


class FakeProcess:
    def __init__(self):
        self.pid = 1_000_000_000
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired("fake-owned-worker", timeout)
        return self.returncode


@unittest.skipUnless(importlib.util.find_spec("mujoco") and importlib.util.find_spec("numpy"),
                     "MuJoCo/NumPy are required for the fixed model owner tests")
class GraspOwnedJobTests(unittest.TestCase):
    def setUp(self):
        self.source = ROOT/"runtime/generated/workcells/fr5_ur5e"
        if not (self.source/"cell.xml").is_file():
            self.skipTest("Checked generated FR5 assets are absent")
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.processes, self.spawn_calls = [], []
        self.signals = patch("mujoco_ros2_manipulation.grasping.demo_server.os.killpg", side_effect=ProcessLookupError())
        self.signal_mock = self.signals.start()
        self.addCleanup(self.signals.stop)
        self.now = 0.

        def spawn(*args, **kwargs):
            self.spawn_calls.append((args, kwargs))
            child = FakeProcess()
            self.processes.append(child)
            return child

        self.app = DemoApplication(self.source, self.root/"owner", clock=lambda: self.now,
                                   _process_factory=spawn)
        self.addCleanup(self.close_app)

    def close_app(self):
        for process in self.processes:
            if process.returncode is None:
                process.returncode = 130
        self.app.close()

    def start(self, identifier="1"*32):
        value = self.app.start(dict(request_id=identifier, model_sha256=self.app.source["model_sha256"]))
        return self.app.current, value

    def observation(self, job, *, sequence=1, age_sec=0.):
        folder = job.directory/"frame-model"
        folder.mkdir(exist_ok=True)
        (folder/"fr5_grasp.xml").write_bytes(self.app.model_raw)
        prototype = FR5Prototype(folder, self.app.physical_manifest)
        frame = prototype.observe("initial_settle")
        observed = time.monotonic_ns()-int(age_sec*1e9)
        frame["observed_monotonic_ns"] = observed
        return dict(schema_version=1, job_id=job.identifier, generation=job.generation,
            binding_id=job.binding_identifier, model_sha256=self.app.source["model_sha256"],
            manifest_sha256=self.app.source["manifest_sha256"], engine=ENGINE, sequence=sequence,
            observed_monotonic_ns=observed, frame=frame, task=prototype.life.snapshot(),
            integrated_steps=0, terminal_outcome=None)

    def test_start_is_idempotent_and_one_child_owns_the_fixed_scenario(self):
        job, first = self.start()
        _, replay = self.start()
        self.assertEqual(len(self.processes), 1)
        self.assertEqual(first["job"]["job_id"], replay["job"]["job_id"])
        self.assertFalse(replay["request_replay"]["new_job_started"])
        with self.assertRaises(RuntimeError):
            self.start("2"*32)
        args, kwargs = self.spawn_calls[0]
        self.assertTrue(kwargs["start_new_session"])
        self.assertEqual(kwargs["cwd"], ROOT)
        self.assertNotIn("ROS_DOMAIN_ID", kwargs["env"])
        self.assertNotIn("LD_LIBRARY_PATH", kwargs["env"])
        self.assertIn("mujoco_ros2_manipulation.grasping.live_worker", args[0])
        self.assertEqual(job.directory.parent, self.app.output_root)

    def test_wrong_model_job_or_generation_never_sends_a_signal(self):
        job, _ = self.start()
        valid = dict(job_id=job.identifier, generation=job.generation, model_sha256=self.app.source["model_sha256"])
        for changes in ({"job_id": "0"*32}, {"generation": True}, {"generation": job.generation+1},
                        {"model_sha256": "0"*64}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.app.cancel(dict(valid, **changes))
        self.assertEqual(self.signal_mock.call_count, 0)

    def test_cancel_is_idempotent_and_does_not_claim_physical_stop(self):
        job, _ = self.start()
        value = dict(job_id=job.identifier, generation=job.generation, model_sha256=self.app.source["model_sha256"])
        first = self.app.cancel(value)
        self.app.cancel(value)
        self.assertEqual(first["job"]["cancel_delivery"], "signal_sent")
        self.assertIsNone(first["job"]["physical_stop_confirmed"])
        self.assertEqual(first["job"]["action_state"], "not_applicable")
        calls = [call.args for call in self.signal_mock.call_args_list if call.args[1] != 0]
        self.assertEqual(calls, [(job.process.pid, signal.SIGINT)])

    def test_timeout_escalation_only_addresses_this_owned_process_group(self):
        job, _ = self.start()
        self.now = 61.
        self.app.snapshot()
        self.now = 64.
        self.app.snapshot()
        self.now = 66.
        self.app.snapshot()
        calls = [call.args for call in self.signal_mock.call_args_list if call.args[1] != 0]
        self.assertEqual(calls, [(job.process.pid, signal.SIGINT), (job.process.pid, signal.SIGTERM),
                                 (job.process.pid, signal.SIGKILL)])
        job.process.returncode = -9
        result = self.app.snapshot()["job"]
        self.assertTrue(result["timed_out"])
        self.assertTrue(result["process_group_gone"])
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["type"], "DemoTimeout")
        self.assertIsNone(result["physical_stop_confirmed"])
        self.assertTrue((job.directory/"owner-receipt.json").is_file())

    def test_actual_qpos_geometry_binding_and_stale_age_are_preserved(self):
        job, _ = self.start()
        value = self.observation(job, age_sec=1.)
        atomic_json(job.directory/"latest.json", value)
        result = self.app.snapshot()
        self.assertTrue(result["view"]["pose_ready"])
        self.assertEqual(result["job"]["freshness"], "stale")
        pose = self.app.pose_envelope(job.binding_identifier)
        self.assertGreaterEqual(pose["source_state_age_sec"], 1.)
        self.assertEqual(pose["source"]["sequence"], value["sequence"])
        self.assertEqual(set(pose["source"]["joint_positions_rad"]), set(ARM_JOINTS))
        self.assertEqual(pose["pose"]["model_sha256"], self.app.source["model_sha256"])
        with self.assertRaises(ValueError):
            self.app.pose_envelope("0"*64)

    def test_changed_model_frame_is_refused_without_rebinding_it(self):
        job, _ = self.start()
        value = self.observation(job)
        value["model_sha256"] = "0"*64
        atomic_json(job.directory/"latest.json", value)
        state = self.app.snapshot()
        self.assertFalse(state["view"]["pose_ready"])
        self.assertIsNotNone(state["job"]["error"])
        with self.assertRaises(ValueError):
            self.app.pose_envelope(job.binding_identifier)

    def test_same_sequence_with_changed_content_is_not_a_new_observation(self):
        job, _ = self.start()
        value = self.observation(job)
        atomic_json(job.directory/"latest.json", value)
        self.assertTrue(self.app.snapshot()["view"]["pose_ready"])
        accepted = deepcopy(job.frame)
        value["terminal_outcome"] = "failed"
        atomic_json(job.directory/"latest.json", value)
        self.assertFalse(self.app.snapshot()["view"]["pose_ready"])
        self.assertEqual(job.frame, accepted)

    def test_implementation_pin_refuses_changed_code_and_snapshot_has_exact_sources(self):
        documents = source_documents()
        changed = dict(documents, **{"grasping/fr5_run.py": documents["grasping/fr5_run.py"]+b"\n# changed"})
        with patch("mujoco_ros2_manipulation.grasping.demo_server.source_documents", return_value=changed):
            with self.assertRaises(RuntimeError):
                self.start()
        self.assertEqual(self.processes, [])
        folder = self.root/"snapshot"
        folder.mkdir()
        atomic_json(folder/"source-pin.json", source_digests(documents))
        receipt = preserve_sources(folder)
        for name, digest in receipt.items():
            raw = (folder/"implementation_sources"/name).read_bytes()
            self.assertEqual(raw, documents[name])
            self.assertEqual(hashlib.sha256(raw).hexdigest(), digest)


if __name__ == "__main__":
    unittest.main()
