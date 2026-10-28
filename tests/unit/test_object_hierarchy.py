"""Same-compiled-model ownership and binding-specific scene host transport."""
from copy import deepcopy
import json
from pathlib import Path
import os
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import unittest

from mujoco_ros2_manipulation.operator_console.object_hierarchy import (
    compiled_object_hierarchy, from_compiled_hierarchy, normalize_robot_roots,
    validate_object_hierarchy)
from mujoco_ros2_manipulation.operator_console.scene_view import CompiledScene
from mujoco_ros2_manipulation.operator_console.scene_host import SceneHost, SceneSource, _canonical
from mujoco_ros2_manipulation.operator_console.display_contract import DisplayHostAdapter, from_compiled_manifest
from tests.unit.test_scene_host import Factory, Clock, binding


XML = '''<mujoco><worldbody>
  <geom name="floor" type="plane" size="2 2 .1"/>
  <body name="rack"><geom name="left_looks_like_robot" type="box" size=".1 .1 .1"/></body>
  <body name="mountAlpha"><joint name="qa"/>
    <geom name="arbitrary_visual" type="sphere" size=".1"/>
    <body name="unprefixed_child"><geom name="hidden_shape" type="box" size=".1 .1 .1" group="3"/>
      <body name="tool_named_but_unknown"><geom name="tool_geometry" type="sphere" size=".03"/></body>
    </body>
  </body>
  <body name="left_spoof"><geom name="spoof_shape" type="box" size=".1 .1 .1"/></body>
  <body name="mountBeta"><joint name="qb"/><geom name="right_shape" type="sphere" size=".1"/></body>
</worldbody></mujoco>'''
ROOTS = (("left", "mountAlpha"), ("right", "mountBeta"))


class ObjectHierarchyTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.xml = self.directory/"scene.xml"
        self.xml.write_text(XML)
        self.scene = CompiledScene(self.xml, ["qa", "qb"], dict(qa=0., qb=0.))

    def test_actual_compiled_parent_and_geom_body_ids_define_ownership_without_prefix_guesses(self):
        value = compiled_object_hierarchy(self.scene, ROOTS)
        model = self.scene._model
        self.assertEqual(len(value["bodies"]), model.nbody)
        self.assertEqual(len(value["geoms"]), model.ngeom)
        for row in value["bodies"][1:]:
            self.assertEqual(row["parent_body_id"], int(model.body_parentid[row["body_id"]]))
        for row in value["geoms"]:
            self.assertEqual(row["body_id"], int(model.geom_bodyid[row["geom_id"]]))
            self.assertIn(self.scene.scene_sha256, row["id"])
        bodies = {row["name"]:row for row in value["bodies"]}
        self.assertEqual(bodies["unprefixed_child"]["robot_instance_id"], "left")
        self.assertIsNone(bodies["left_spoof"]["robot_instance_id"])
        geoms = {row["name"]:row for row in value["geoms"]}
        self.assertIsNone(geoms["left_looks_like_robot"]["robot_instance_id"])
        self.assertEqual(geoms["hidden_shape"]["robot_instance_id"], "left")
        self.assertTrue(all(row["tool_role"] == "unknown" for row in value["bodies"]+value["geoms"]))
        self.assertEqual(self.scene.manifest()["schema_version"], 1)
        self.assertNotIn("bodies", self.scene.manifest())

    def test_no_declared_robots_means_unknown_even_when_names_look_like_robots_or_tools(self):
        value = compiled_object_hierarchy(self.scene)
        self.assertEqual(value["counts"]["robots"], 0)
        self.assertTrue(all(row["robot_instance_id"] is None for row in value["geoms"]))
        self.assertTrue(all(row["tool_role"] == "unknown" for row in value["geoms"]))

    def test_same_in_memory_model_is_used_after_source_file_is_replaced(self):
        original = compiled_object_hierarchy(self.scene, ROOTS)
        self.xml.write_text('<mujoco><worldbody><geom type="sphere" size="1"/></worldbody></mujoco>')
        self.assertEqual(compiled_object_hierarchy(self.scene, ROOTS), original)

    def test_missing_overlapping_duplicate_and_world_roots_are_rejected(self):
        for roots in ((("left", "missing"),), (("left", "world"),),
                      (("left", "mountAlpha"), ("tool", "unprefixed_child")),
                      (("left", "mountAlpha"), ("left", "mountBeta"))):
            with self.subTest(roots=roots), self.assertRaises(ValueError):
                compiled_object_hierarchy(self.scene, roots)

    def test_invalid_geometry_identity_parent_cycle_and_forged_ownership_fail_closed(self):
        original = compiled_object_hierarchy(self.scene, ROOTS)
        for modify in (lambda value:value.update(scene_sha256="0"*64),
                       lambda value:value["geoms"][0].update(geom_id=True),
                       lambda value:value["geoms"].pop(),
                       lambda value:value["bodies"][1].update(parent_body_id=1),
                       lambda value:value["geoms"][0].update(robot_instance_id="left"),
                       lambda value:value["counts"].update(robots=True)):
            value = deepcopy(original)
            modify(value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_object_hierarchy(value, self.scene.manifest(), robot_roots=ROOTS)

    def test_display_adapter_rebases_stable_ids_and_keeps_exact_native_source_identity(self):
        native = compiled_object_hierarchy(self.scene, ROOTS)
        manifest = from_compiled_manifest(self.scene.manifest())
        value = from_compiled_hierarchy(native, manifest)
        self.assertNotEqual(value["scene_sha256"], native["scene_sha256"])
        self.assertEqual(value["source_scene_sha256"], native["scene_sha256"])
        self.assertEqual([row["body_id"] for row in value["geoms"]], [row["body_id"] for row in native["geoms"]])
        self.assertTrue(all(value["scene_sha256"] in row["id"] for row in value["geoms"]+value["bodies"]))
        self.assertEqual(validate_object_hierarchy(value, manifest), value)
        manifest["provenance"]["source_sha256"] = "0"*64
        with self.assertRaises(ValueError):
            from_compiled_hierarchy(native, manifest)

    def test_metadata_is_detached_and_private_freeform_names_are_redacted(self):
        self.xml.write_text('<mujoco><worldbody><body name="/home/private"><geom name="https://secret-host/token" type="sphere" size=".1"/></body></worldbody></mujoco>')
        scene = CompiledScene(self.xml, [], {})
        value = compiled_object_hierarchy(scene)
        encoded = json.dumps(value, allow_nan=False)
        self.assertNotIn("/home/private", encoded)
        self.assertNotIn("secret-host", encoded)
        self.assertTrue(value["geoms"][0]["name_redacted"])
        value["geoms"][0]["body_id"] = 999
        self.assertNotEqual(value, compiled_object_hierarchy(scene))

    def test_actual_cpu_worker_publishes_same_scene_hierarchy_before_manifest(self):
        output = self.directory/"worker"
        output.mkdir()
        child = subprocess.Popen([sys.executable, "-m", "mujoco_ros2_manipulation.operator_console.scene_worker",
            "--model", str(self.xml), "--directory", str(output), "--parent-pid", str(os.getpid()),
            "--joint-names-json", '["qa","qb"]', "--home-json", '{"qa":0,"qb":0}',
            "--robot-roots-json", json.dumps(ROOTS)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            deadline = time.monotonic()+8.
            while not (output/"manifest.json").exists() and child.poll() is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertIsNone(child.poll(), (child.communicate() if child.poll() is not None else None))
            self.assertTrue((output/"manifest.json").exists(), "Worker manifest readiness timed out")
            manifest = json.loads((output/"manifest.json").read_bytes())
            hierarchy = json.loads((output/"hierarchy.json").read_bytes())
            self.assertEqual(validate_object_hierarchy(hierarchy, manifest, robot_roots=ROOTS), hierarchy)
            self.assertEqual(hierarchy, compiled_object_hierarchy(self.scene, ROOTS))
        finally:
            if child.poll() is None:
                child.terminate()
            child.communicate(timeout=5)
        self.assertEqual(child.returncode, 0)


class HierarchyHostTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.xml = self.root/"model.xml"
        self.xml.write_text('<mujoco><worldbody><geom name="fixture" type="box" size="1 1 1"/></worldbody></mujoco>')
        self.clock, self.factory = Clock(), Factory()
        self.source = SceneSource("workcell:test", "Small fixture", self.xml, ("j1",), {"j1":0.}, lambda:None)
        self.host = SceneHost(self.root/"host", self.root/"python", clock=self.clock,
            process_factory=self.factory, start_thread=False)
        self.addCleanup(self.host.close)

    def fixture_hierarchy(self, manifest):
        # Actual compiled shape, then rebind fixture identity to the fake worker's
        # independently validated manifest. This tests transport, not compilation.
        real = CompiledScene(self.xml, [], {})
        value = compiled_object_hierarchy(real)
        value.update(scene_sha256=manifest["scene_sha256"], source_scene_sha256=manifest["scene_sha256"],
                     compiled_model_sha256=manifest["compiled_model_sha256"])
        sha = manifest["scene_sha256"]
        value["bodies"][0]["id"] = "body:"+sha+":0"
        value["geoms"][0].update(id="geom:"+sha+":0", body_ref="body:"+sha+":0")
        return validate_object_hierarchy(value, manifest)

    def test_legacy_fake_worker_without_hierarchy_is_explicitly_unavailable(self):
        identifier = self.host.activate(binding(), self.source)
        self.host._step()
        self.assertIsNone(self.host.hierarchy_bytes(identifier))
        self.assertFalse(self.host.snapshot()["hierarchy_available"])

    def test_current_binding_only_frozen_hierarchy_and_display_digest_adapter(self):
        identifier = self.host.activate(binding(), self.source)
        self.host._step()
        process = self.factory.processes[-1]
        value = self.fixture_hierarchy(process.manifest)
        (process.directory/"hierarchy.json").write_bytes(_canonical(value))
        self.host._epoch["manifest_ready"] = False
        self.host._step()
        self.assertTrue(self.host.snapshot()["hierarchy_available"])
        native = self.host.hierarchy_bytes(identifier)
        (process.directory/"hierarchy.json").write_text("tampered after freeze")
        self.assertEqual(self.host.hierarchy_bytes(identifier), native)
        adapter = DisplayHostAdapter(self.host)
        adapted = json.loads(adapter.hierarchy_bytes(identifier))
        display = json.loads(adapter.manifest_bytes(identifier))
        self.assertEqual(adapted["scene_sha256"], display["scene_sha256"])
        self.assertEqual(adapted["source_scene_sha256"], value["scene_sha256"])
        self.assertEqual(adapter.snapshot()["hierarchy_descriptor"]["scene_sha256"], display["scene_sha256"])
        self.host.activate(binding(2), self.source)
        self.assertFalse(self.host.snapshot()["hierarchy_available"])
        with self.assertRaises(ValueError):
            adapter.hierarchy_bytes(identifier)

    def test_wrong_scene_hierarchy_fails_closed_before_worker_becomes_ready(self):
        identifier = self.host.activate(binding(), self.source)
        self.host._step()
        process = self.factory.processes[-1]
        value = self.fixture_hierarchy(process.manifest)
        value["scene_sha256"] = "0"*64
        (process.directory/"hierarchy.json").write_bytes(_canonical(value))
        self.host._epoch["manifest_ready"] = False
        self.host._step()
        self.assertEqual(self.host.snapshot()["status"], "error")
        with self.assertRaises(ValueError):
            self.host.hierarchy_bytes(identifier)

    def test_roots_are_detached_frozen_and_passed_only_as_bounded_worker_metadata(self):
        roots = [["left", "mountAlpha"]]
        source = SceneSource(self.source.id, self.source.label, self.xml, self.source.joint_names,
            self.source.home, self.source.validate, robot_roots=roots)
        roots[0][1] = "changed"
        self.assertEqual(source.robot_roots, (("left", "mountAlpha"),))
        self.host.activate(binding(), source)
        self.host._step()
        argv = self.factory.processes[-1].argv
        self.assertEqual(json.loads(argv[argv.index("--robot-roots-json")+1]), [["left", "mountAlpha"]])
        self.assertEqual(normalize_robot_roots(source.robot_roots), source.robot_roots)
