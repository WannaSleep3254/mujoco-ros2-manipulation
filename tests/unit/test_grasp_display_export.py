"""Observed-state display fixtures: no physics integration, ROS or HTTP server."""
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.grasping.display_export import (
    export_recording, FINGER_JOINTS, LiveGraspDisplay, ObservedModelDisplay)
from mujoco_ros2_manipulation.grasping.fr5_model import ARM_JOINTS, TCP_SITE
from mujoco_ros2_manipulation.operator_console.recording import Recording, RecordedSceneHost
from mujoco_ros2_manipulation.operator_console.scene_view import CompiledScene, SceneLimits
from mujoco_ros2_manipulation.paths import ROOT


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def manifest_digest(value):
    return digest(json.dumps({k: v for k, v in value.items() if k != "manifest_sha256"},
        sort_keys=True, separators=(",", ":"), allow_nan=False).encode())


@unittest.skipUnless(importlib.util.find_spec("mujoco") and importlib.util.find_spec("numpy"),
                     "MuJoCo/NumPy are required for observed display fixtures")
class GraspDisplayExportTests(unittest.TestCase):
    def setUp(self):
        import mujoco
        import numpy as np
        self.mj, self.np = mujoco, np
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.source = self.directory/"source"
        self.source.mkdir()
        body = ""
        for index, name in enumerate(ARM_JOINTS):
            kind = '<geom type="mesh" mesh="tetra" mass=".1"/>' if index == 0 else '<geom type="sphere" size=".025" mass=".1"/>'
            body += f'<body name="arm{index}" pos=".07 0 0"><joint name="{name}" axis="0 0 1"/>{kind}'
        body += f'<site name="{TCP_SITE}" pos=".04 0 0" size=".002"/>'
        for side, name, offset in (("left", FINGER_JOINTS[0], -.04), ("right", FINGER_JOINTS[1], .04)):
            body += (f'<body name="finger_{side}" pos="{offset} 0 0"><joint name="{name}" type="slide" axis="1 0 0"/>'
                     f'<geom name="{side}_finger_geom" type="box" size=".005 .01 .03" pos="0 0 .01" mass=".03"/></body>')
        body += "</body>"*6
        self.xml = (f'<mujoco><compiler angle="radian" meshdir="."/><asset>'
            '<mesh name="tetra" vertex="0 0 0 .04 0 0 0 .04 0 0 0 .04" face="0 2 1 0 1 3 0 3 2 1 2 3"/>'
            f'</asset><worldbody>{body}<body name="fr5_grasp_block" pos=".3 .1 .2"><freejoint name="fr5_block_free"/>'
            '<geom name="block_geom" type="box" size=".02 .015 .025" pos=".01 0 0" mass=".08"/>'
            '</body></worldbody></mujoco>').encode()
        self.model = mujoco.MjModel.from_xml_string(self.xml.decode())
        scalar = list(ARM_JOINTS + FINGER_JOINTS)
        layout = [dict(name=name, type="hinge" if name in ARM_JOINTS else "slide", qpos_address=i, qpos_width=1)
                  for i, name in enumerate(scalar)]
        layout.append(dict(name="fr5_block_free", type="free", qpos_address=8, qpos_width=7))
        contract = dict(nq=15, qpos_layout=layout,
            free_joint_format="world xyz metres followed by quaternion wxyz", scalar_joint_names=scalar,
            scalar_joint_units={name: "rad" if name in ARM_JOINTS else "m" for name in scalar},
            scalar_joint_positions="Actual MuJoCo qpos observations, never position-servo ctrl targets")
        self.manifest = dict(schema_version=1, prototype="fr5_parallel_gripper_v1", simulation_only=True,
            mesh_inputs_sha256={}, physical_engine_used_for_static_fk="MuJoCo "+mujoco.__version__,
            qpos_state_contract=contract, mjcf_sha256=digest(self.xml))
        self.manifest["manifest_sha256"] = manifest_digest(self.manifest)
        first = [.1, 0., 0., 0., 0., 0., .001, .002, .3, .1, .2, 1., 0., 0., 0.]
        half = math.sqrt(.5)
        second = [.2, 0., 0., 0., 0., 0., .01, .015, .35, .12, .26, half, 0., 0., half]
        self.result = dict(schema_version=1, prototype="fr5_parallel_gripper_v1", engine="MuJoCo "+mujoco.__version__,
            manifest_sha256=self.manifest["manifest_sha256"], qpos_state_contract=deepcopy(contract),
            outcome="succeeded", error=None, samples=[self.frame(first, .02), self.frame(second, .04)])
        self.write_source()

    def frame(self, qpos, stamp):
        data = self.mj.MjData(self.model)
        data.qpos[:] = qpos
        self.mj.mj_forward(self.model, data)
        tcp = self.model.site(TCP_SITE).id
        block = self.model.body("fr5_grasp_block").id
        rotation = data.site_xmat[tcp].reshape(3, 3)
        return dict(simulation_sec=stamp, qpos=list(qpos), arm_rad=list(qpos[:6]),
            scalar_joint_positions=dict(zip(ARM_JOINTS+FINGER_JOINTS, qpos[:8])),
            tcp_position_m=data.site_xpos[tcp].tolist(), block_position_m=data.xpos[block].tolist(),
            tcp_rotation_matrix=rotation.tolist(),
            relative_rotation_matrix=(rotation.T @ data.xmat[block].reshape(3, 3)).tolist())

    def write_source(self):
        (self.source/"fr5_grasp.xml").write_bytes(self.xml)
        (self.source/"manifest.json").write_text(json.dumps(self.manifest))
        (self.source/"result.json").write_text(json.dumps(self.result))

    def export(self, name="display", **kwargs):
        return export_recording(self.source, self.directory/name, **kwargs)

    def test_free_block_rotation_and_observed_fingers_reach_the_unchanged_common_contract(self):
        before = {p.name: p.read_bytes() for p in self.source.iterdir()}
        with patch.object(self.mj, "mj_step", side_effect=AssertionError("Export must not integrate physics")):
            receipt = self.export()
        recording = Recording.load(self.directory/"display/recording.json")
        self.assertEqual(receipt["physics_steps_during_export"], 0)
        self.assertFalse(receipt["robot_command_capability"])
        self.assertFalse(receipt["w07_complete"])
        self.assertEqual(recording.manifest["joint_names"], list(ARM_JOINTS))
        self.assertEqual(recording.frames[1]["positions_rad"], dict(zip(ARM_JOINTS, self.result["samples"][1]["arm_rad"])))
        for index, sample in enumerate(self.result["samples"]):
            data = self.mj.MjData(self.model); data.qpos[:] = sample["qpos"]
            self.mj.mj_forward(self.model, data)
            for geom in recording.frames[index]["pose"]["geoms"]:
                self.np.testing.assert_allclose(geom["position"], data.geom_xpos[geom["id"]], atol=1e-12)
                self.np.testing.assert_allclose(geom["rotation"], data.geom_xmat[geom["id"]], atol=1e-12)
        for name in ("left_finger_geom", "right_finger_geom", "block_geom"):
            geom_id = int(self.model.geom(name).id)
            self.assertNotEqual(recording.frames[0]["pose"]["geoms"][geom_id]["position"],
                                recording.frames[1]["pose"]["geoms"][geom_id]["position"])
        self.assertEqual(receipt["time"]["source_origin_sec"], .02)
        self.assertEqual([f["time_sec"] for f in recording.frames], [0., .02])
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.source.iterdir()})
        self.assertNotIn(str(self.directory), json.dumps(recording.manifest))
        self.assertEqual(receipt["recording_sha256"], recording.sha256)

    def test_mesh_buffers_are_content_addressed_and_tampering_is_rejected(self):
        self.export()
        recording = Recording.load(self.directory/"display/recording.json")
        self.assertEqual(len(recording.manifest["assets"]), 1)
        descriptor = recording.manifest["assets"][0]
        path = self.directory/"display/assets"/(descriptor["sha256"]+".bin")
        self.assertEqual(digest(path.read_bytes()), descriptor["sha256"])
        path.write_bytes(b"x"*descriptor["byte_length"])
        with self.assertRaisesRegex(ValueError, "digest"):
            Recording.load(self.directory/"display/recording.json")

    def test_missing_qpos_or_unobserved_finger_is_rejected_before_output(self):
        for field in ("qpos", "finger"):
            with self.subTest(field=field):
                source = deepcopy(self.result)
                if field == "qpos": del source["samples"][0]["qpos"]
                else: del source["samples"][0]["scalar_joint_positions"][FINGER_JOINTS[0]]
                (self.source/"result.json").write_text(json.dumps(source))
                with self.assertRaisesRegex(ValueError, "qpos|finger"):
                    self.export(field)
                self.assertFalse((self.directory/field).exists())

    def test_observed_scalar_and_body_pose_must_match_complete_qpos(self):
        for mutation in ("finger", "block", "rotation", "quaternion"):
            with self.subTest(mutation=mutation):
                source = deepcopy(self.result)
                frame = source["samples"][1]
                if mutation == "finger": frame["scalar_joint_positions"][FINGER_JOINTS[1]] += .001
                elif mutation == "block": frame["block_position_m"][0] += .001
                elif mutation == "rotation": frame["relative_rotation_matrix"][0][0] = 42.
                else: frame["qpos"][-1] = 2.
                (self.source/"result.json").write_text(json.dumps(source))
                with self.assertRaises(ValueError): self.export(mutation)
                self.assertFalse((self.directory/mutation).exists())

    def test_model_manifest_run_and_owner_digests_fail_closed(self):
        for mutation in ("model", "manifest", "run", "expected"):
            with self.subTest(mutation=mutation):
                self.write_source()
                if mutation == "model": (self.source/"fr5_grasp.xml").write_bytes(self.xml+b"\n")
                elif mutation == "manifest":
                    changed = dict(self.manifest, mjcf_sha256="0"*64)
                    (self.source/"manifest.json").write_text(json.dumps(changed))
                elif mutation == "run":
                    changed = dict(self.result, manifest_sha256="0"*64)
                    (self.source/"result.json").write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError, "digest|receipt"):
                    self.export(mutation, **({"expected_result_sha256":"0"*64} if mutation == "expected" else {}))

    def test_engine_and_mixed_unit_state_contract_are_explicit(self):
        for mutation in ("engine", "unit"):
            with self.subTest(mutation=mutation):
                changed = deepcopy(self.result)
                if mutation == "engine": changed["engine"] = "MuJoCo 0.0.0"
                else: changed["qpos_state_contract"]["scalar_joint_units"][FINGER_JOINTS[0]] = "rad"
                (self.source/"result.json").write_text(json.dumps(changed))
                with self.assertRaisesRegex(ValueError, "engine|qpos"):
                    self.export(mutation)

    def test_actual_arm_slide_or_finger_hinge_cannot_be_relabelled_as_expected_units(self):
        for name, kind in ((ARM_JOINTS[0], "slide"), (FINGER_JOINTS[0], "hinge")):
            with self.subTest(name=name, kind=kind):
                if name in ARM_JOINTS:
                    xml = self.xml.replace(f'<joint name="{name}" '.encode(),
                        f'<joint name="{name}" type="{kind}" '.encode(), 1)
                else:
                    xml = self.xml.replace(f'<joint name="{name}" type="slide"'.encode(),
                        f'<joint name="{name}" type="{kind}"'.encode(), 1)
                model = self.mj.MjModel.from_xml_string(xml.decode())
                data = self.mj.MjData(model)
                self.mj.mj_forward(model, data)
                with self.assertRaisesRegex(ValueError, "joint type"):
                    ObservedModelDisplay(model, data, model_sha256=digest(xml), source_sha256="a"*64)
                manifest = deepcopy(self.manifest)
                # Even a consistently signed declaration cannot make a slide
                # an arm angle, or a hinge a finger displacement in metres.
                next(row for row in manifest["qpos_state_contract"]["qpos_layout"]
                     if row["name"] == name)["type"] = kind
                manifest["mjcf_sha256"] = digest(xml)
                manifest["manifest_sha256"] = manifest_digest(manifest)
                with self.assertRaisesRegex(ValueError, "joint type"):
                    LiveGraspDisplay(xml, manifest)
                result = deepcopy(self.result)
                result["manifest_sha256"] = manifest["manifest_sha256"]
                result["qpos_state_contract"] = deepcopy(manifest["qpos_state_contract"])
                (self.source/"fr5_grasp.xml").write_bytes(xml)
                (self.source/"manifest.json").write_text(json.dumps(manifest))
                (self.source/"result.json").write_text(json.dumps(result))
                with self.assertRaisesRegex(ValueError, "joint type"):
                    self.export("wrong-type-"+kind)
                self.assertFalse((self.directory/("wrong-type-"+kind)).exists())

    def test_matching_source_declarations_cannot_redefine_actual_layout_type_or_units(self):
        for field, value in (("qpos_address", 1), ("type", "slide"), ("unit", "m")):
            with self.subTest(field=field):
                manifest = deepcopy(self.manifest)
                contract = manifest["qpos_state_contract"]
                if field == "unit": contract["scalar_joint_units"][ARM_JOINTS[0]] = value
                else: contract["qpos_layout"][0][field] = value
                manifest["manifest_sha256"] = manifest_digest(manifest)
                result = deepcopy(self.result)
                result["manifest_sha256"] = manifest["manifest_sha256"]
                result["qpos_state_contract"] = deepcopy(contract)
                (self.source/"manifest.json").write_text(json.dumps(manifest))
                (self.source/"result.json").write_text(json.dumps(result))
                with self.assertRaisesRegex(ValueError, "qpos contract"):
                    LiveGraspDisplay(self.xml, manifest)
                with self.assertRaisesRegex(ValueError, "qpos contract"):
                    self.export("wrong-contract-"+field)
                self.assertFalse((self.directory/("wrong-contract-"+field)).exists())

    def test_limits_new_directory_and_source_time_are_enforced(self):
        for field in ("backwards", "same_time_changed", "limit"):
            with self.subTest(field=field):
                changed = deepcopy(self.result)
                if field == "backwards": changed["samples"][1]["simulation_sec"] = .01
                elif field == "same_time_changed": changed["samples"][1]["simulation_sec"] = .02
                (self.source/"result.json").write_text(json.dumps(changed))
                with self.assertRaises(ValueError):
                    self.export(field, **({"limits":SceneLimits(max_geoms=1)} if field == "limit" else {}))
        self.write_source(); self.export()
        before = (self.directory/"display/recording.json").read_bytes()
        with self.assertRaisesRegex(ValueError, "new directory"): self.export()
        self.assertEqual(before, (self.directory/"display/recording.json").read_bytes())
        with self.assertRaisesRegex(ValueError, "frozen"):
            export_recording(self.source, self.source/"display")

    def test_external_mjcf_reads_are_rejected_before_compiler_access(self):
        self.xml = self.xml.replace(b"<worldbody>", b'<include file="private.xml"/><worldbody>')
        self.manifest["mjcf_sha256"] = digest(self.xml)
        self.manifest["manifest_sha256"] = manifest_digest(self.manifest)
        self.result["manifest_sha256"] = self.manifest["manifest_sha256"]
        self.write_source()
        with patch.object(self.mj.MjModel, "from_xml_string", side_effect=AssertionError("Compiler must not read include")):
            with self.assertRaisesRegex(ValueError, "section|asset"):
                self.export()

    def test_failed_outcome_stays_failed_and_recording_has_only_readonly_controls(self):
        self.result.update(outcome="failed", error=dict(type="Slip", message="observed slip"))
        self.write_source(); receipt = self.export()
        self.assertEqual(receipt["source_outcome"], "failed")
        self.assertEqual(receipt["source_error"]["type"], "Slip")
        recording = Recording.load(self.directory/"display/recording.json")
        self.assertIn("failed", recording.label)
        now = [0.]
        host = RecordedSceneHost(recording, clock=lambda:now[0])
        with self.assertRaises(ValueError): host.control("execute")
        old_binding = host.snapshot()["binding_id"]
        host.control("seek", .01)
        self.assertNotEqual(old_binding, host.snapshot()["binding_id"])
        now[0] = 1.
        pose = host.pose_envelope(host.snapshot()["binding_id"])
        self.assertEqual(pose["source_state_age_sec"], 1.)
        self.assertEqual(set(pose["source"]["joint_positions_rad"]), set(ARM_JOINTS))
        host.close()

    def test_existing_scalar_scene_producer_still_rejects_freejoint(self):
        with self.assertRaisesRegex(ValueError, "Every model joint|scalar"):
            CompiledScene(self.source/"fr5_grasp.xml", list(ARM_JOINTS+FINGER_JOINTS),
                          dict(zip(ARM_JOINTS+FINGER_JOINTS, [0.]*8)))

    def test_live_wrapper_requires_full_observations_and_never_recompiles_or_steps_per_pose(self):
        producer = LiveGraspDisplay(self.xml, self.manifest)
        manifest = producer.manifest
        self.assertEqual(json.loads(producer.manifest_raw), manifest)
        manifest["geoms"].clear()
        self.assertTrue(producer.manifest["geoms"])
        with patch.object(self.mj.MjModel, "from_xml_string", side_effect=AssertionError("No per-frame compile")), \
             patch.object(self.mj, "mj_step", side_effect=AssertionError("No display physics")):
            pose = producer.pose(self.result["samples"][1])
        data = self.mj.MjData(self.model); data.qpos[:] = self.result["samples"][1]["qpos"]
        self.mj.mj_forward(self.model, data)
        for geom in pose["geoms"]:
            self.np.testing.assert_allclose(geom["position"], data.geom_xpos[geom["id"]], atol=1e-12)
            self.np.testing.assert_allclose(geom["rotation"], data.geom_xmat[geom["id"]], atol=1e-12)
        incomplete = deepcopy(self.result["samples"][0]); incomplete.pop("qpos")
        with self.assertRaisesRegex(ValueError, "qpos"): producer.pose(incomplete)

    def test_existing_model_display_reads_only_and_refuses_foreign_data(self):
        data = self.mj.MjData(self.model); data.qpos[:] = self.result["samples"][0]["qpos"]
        self.mj.mj_forward(self.model, data)
        before = data.qpos.copy()
        producer = ObservedModelDisplay(self.model, data,
            model_sha256=digest(self.xml), source_sha256="a"*64)
        with patch.object(self.mj, "mj_forward", side_effect=AssertionError("Producer already forwarded")), \
             patch.object(self.mj, "mj_step", side_effect=AssertionError("No display physics")):
            pose = producer.pose(data)
        self.np.testing.assert_array_equal(data.qpos, before)
        self.assertEqual(data.time, 0.)
        self.assertEqual(pose["model_sha256"], digest(self.xml))
        other = self.mj.MjModel.from_xml_string(self.xml.decode())
        with self.assertRaisesRegex(ValueError, "foreign"):
            producer.pose(self.mj.MjData(other))

    def test_frozen_actual_c_observations_reconstruct_without_any_physics_step(self):
        source = ROOT/"runtime/w07-fr5/prototype-20261028-c"
        if not (source/"result.json").is_file():
            self.skipTest("Frozen local FR5 prototype c is not present")
        before = {name: (source/name).read_bytes() for name in ("fr5_grasp.xml", "manifest.json", "result.json")}
        raw = before["result.json"]
        result = json.loads(raw)
        if result["engine"] != "MuJoCo "+self.mj.__version__:
            self.skipTest("Frozen local result uses a different MuJoCo version")
        with patch.object(self.mj, "mj_step", side_effect=AssertionError("Replay must not integrate")):
            receipt = export_recording(source, self.directory/"actual-c", expected_result_sha256=digest(raw))
        self.assertEqual(receipt["frames"], len(result["samples"]))
        self.assertEqual(receipt["physics_steps_during_export"], 0)
        self.assertLessEqual(max(receipt["reconstructed_pose_max_absolute_errors"].values()), 1e-9)
        recording = Recording.load(self.directory/"actual-c/recording.json")
        baseline_path = ROOT/"runtime/w07-fr5/display-c-20261028-v2/recording.json"
        if baseline_path.is_file():
            baseline_raw = baseline_path.read_bytes()
            baseline = Recording.load(baseline_path)
            # Adapter provenance changes on a source edit; geometry bytes,
            # physical observations and their source timestamps must not.
            self.assertEqual(recording.assets, baseline.assets)
            for field in ("geoms", "assets", "joint_names", "home_rad", "world_frame", "units", "default_visible_groups"):
                self.assertEqual(recording.manifest[field], baseline.manifest[field])
            self.assertEqual(len(recording.frames), len(baseline.frames))
            for current, previous in zip(recording.frames, baseline.frames):
                self.assertEqual(current["time_sec"], previous["time_sec"])
                self.assertEqual(current["positions_rad"], previous["positions_rad"])
                self.assertEqual(current["pose"]["geoms"], previous["pose"]["geoms"])
            self.assertEqual(baseline_path.read_bytes(), baseline_raw)
        model = self.mj.MjModel.from_xml_path(str(source/"fr5_grasp.xml"))
        for sample_index in (0, len(result["samples"])//2, len(result["samples"])-1):
            sample = result["samples"][sample_index]
            data = self.mj.MjData(model); data.qpos[:] = sample["qpos"]
            self.mj.mj_forward(model, data)
            for geom in recording.frames[sample_index]["pose"]["geoms"]:
                self.np.testing.assert_allclose(geom["position"], data.geom_xpos[geom["id"]], atol=1e-12)
                self.np.testing.assert_allclose(geom["rotation"], data.geom_xmat[geom["id"]], atol=1e-12)
        self.assertEqual({name: (source/name).read_bytes() for name in before}, before)
