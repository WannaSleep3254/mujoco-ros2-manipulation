"""Owned local observation/cancellation and compile-input race regressions."""
from copy import deepcopy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.grasping import live_worker
from mujoco_ros2_manipulation.grasping.fr5_model import write_model
from mujoco_ros2_manipulation.grasping.fr5_run import FR5Prototype, PrototypeFailure
from mujoco_ros2_manipulation.paths import ROOT


@unittest.skipUnless(importlib.util.find_spec("mujoco") and importlib.util.find_spec("numpy"),
                     "MuJoCo and NumPy are required")
class LiveWorkerTests(unittest.TestCase):
    def setUp(self):
        import mujoco
        self.mj = mujoco
        self.source = ROOT/"runtime/generated/workcells/fr5_ur5e"
        if not (self.source/"cell.xml").is_file():
            self.skipTest("Checked FR5 source assets are not present")
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def prototype(self, **kwargs):
        directory = self.root/"run"
        manifest = write_model(self.source, directory)
        return FR5Prototype(directory, manifest, **kwargs)

    def test_cancellation_stops_integration_before_any_subsequent_task_stage(self):
        published = []
        prototype = self.prototype(observer=published.append,
            cancel=lambda: bool(published and published[-1]["frame"]["phase"] == "close"))
        result = prototype.run()
        self.assertEqual(result["outcome"], "cancelled")
        self.assertEqual(result["cancellation"]["phase"], "close")
        self.assertFalse(result["cancellation"]["physical_stop_confirmed"])
        self.assertEqual(result["cancellation"]["action_state"], "not_applicable")
        self.assertEqual(result["cancellation"]["followup_task_stages"], 0)
        self.assertFalse(result["cancellation"]["automatic_release"])
        self.assertTrue(published)
        self.assertFalse(any(row["frame"]["phase"] in {"extract", "carry", "lower", "open", "retract"}
                             for row in published))
        self.assertFalse(any(row["phase"] in {"extract", "carry", "lower", "open", "retract"}
                             for row in result["segments"]))
        self.assertGreater(result["physics"]["integrated_steps"], 0)
        self.assertEqual(result["physics"]["integrated_steps"], result["cancellation"]["integrated_steps"])
        self.assertEqual(result["final"]["qpos"], prototype.data.qpos.tolist())

    def test_observer_mutation_cannot_modify_physical_or_lifecycle_evidence(self):
        def observer(payload):
            payload["frame"]["qpos"][0] = 999.
            payload["frame"]["unexpected_contacts"].append(["invented", "contact"])
            payload["task"]["stage"] = "succeeded"
        prototype = self.prototype(observer=observer)
        frames = prototype.hold("initial_settle", 0., .02)
        self.assertTrue(all(abs(row["qpos"][0]) < 1. for row in frames))
        self.assertTrue(all(not row["unexpected_contacts"] for row in frames))
        self.assertNotEqual(prototype.life.stage, "succeeded")
        self.assertGreater(frames[-1]["observed_monotonic_ns"], 0)

    def test_realtime_cancel_is_polled_before_next_physical_step(self):
        checks = []
        def cancel():
            checks.append(True)
            return len(checks) >= 2
        prototype = self.prototype(cancel=cancel, realtime=True)
        result = prototype.run()
        self.assertEqual(result["outcome"], "cancelled")
        self.assertEqual(result["physics"]["integrated_steps"], 0)
        self.assertEqual(result["segments"], [])
        self.assertFalse(result["cancellation"]["physical_stop_confirmed"])

    def test_observer_failure_is_preserved_as_failure_not_success(self):
        def broken(_):
            raise OSError("intentional reporting failure")
        result = self.prototype(observer=broken).run()
        self.assertEqual(result["outcome"], "failed")
        self.assertEqual(result["error"]["type"], "PrototypeFailure")
        self.assertIn("Observation publication failed", result["error"]["message"])

    def copied_compile_inputs(self):
        directory = self.root/"run"
        manifest = write_model(self.source, directory)
        xml = ET.fromstring((directory/"fr5_grasp.xml").read_bytes())
        declared = {}
        copied_mesh = None
        for index, element in enumerate(xml.findall("asset/mesh")):
            original = Path(element.attrib["file"])
            copied = self.root/("mesh-"+str(index)+original.suffix)
            copied.write_bytes(original.read_bytes())
            element.set("file", str(copied))
            declared[str(copied)] = hashlib.sha256(copied.read_bytes()).hexdigest()
            copied_mesh = copied
        raw = ET.tostring(xml)
        manifest = deepcopy(manifest)
        manifest["mesh_inputs_sha256"] = declared
        manifest["mjcf_sha256"] = hashlib.sha256(raw).hexdigest()
        manifest["manifest_sha256"] = hashlib.sha256(json.dumps(
            {key: value for key, value in manifest.items() if key != "manifest_sha256"},
            sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        (directory/"fr5_grasp.xml").write_bytes(raw)
        (directory/"manifest.json").write_text(json.dumps(manifest))
        return directory, manifest, copied_mesh

    def test_worker_rejects_mesh_mutation_during_compile_at_zero_actual_steps(self):
        directory, manifest, mesh = self.copied_compile_inputs()
        job = self.root
        pin = live_worker.source_digests(live_worker.source_documents())
        (job/"source-pin.json").write_text(json.dumps(pin))
        actual_compile = self.mj.MjModel.from_xml_string
        def tampered_compile(raw):
            model = actual_compile(raw)
            mesh.write_bytes(mesh.read_bytes()+b"fault injection on owned copy")
            return model
        with patch.object(live_worker, "write_model", return_value=manifest), \
                patch.object(self.mj.MjModel, "from_xml_string", side_effect=tampered_compile), \
                patch.object(self.mj, "mj_step") as step:
            code = live_worker.main(["--source", str(self.source), "--job-directory", str(job),
                "--job-id", "1"*32, "--generation", "1", "--binding-id", "2"*64,
                "--model-sha256", manifest["mjcf_sha256"],
                "--manifest-sha256", manifest["manifest_sha256"]])
        self.assertEqual(code, 1)
        step.assert_not_called()
        error = json.loads((job/"worker_error.json").read_text())
        self.assertEqual(error["actual_steps"], 0)
        self.assertFalse(error["physical_stop_confirmed"])
        self.assertIn("mesh digest", error["error"]["message"])
        self.assertTrue((job/"implementation_sources/grasping/fr5_run.py").is_file())
        self.assertFalse((directory/"result.json").exists())

    def test_xml_mutation_during_compile_is_rejected_before_first_step(self):
        directory, manifest, _ = self.copied_compile_inputs()
        actual_compile = self.mj.MjModel.from_xml_string
        def tampered_compile(raw):
            model = actual_compile(raw)
            path = directory/"fr5_grasp.xml"
            path.write_bytes(path.read_bytes()+b"\n<!-- changed during compile -->")
            return model
        with patch.object(self.mj.MjModel, "from_xml_string", side_effect=tampered_compile), \
                patch.object(self.mj, "mj_step") as step:
            with self.assertRaisesRegex(ValueError, "compiler inputs changed"):
                FR5Prototype(directory, manifest)
        step.assert_not_called()


if __name__ == "__main__":
    unittest.main()
