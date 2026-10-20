"""Shared robot physics, FK, same-model reuse and staged publication regressions."""

from copy import deepcopy
from dataclasses import dataclass, replace
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from mujoco_ros2_manipulation.workcells.config import Workcell, Instance, load_workcell
from mujoco_ros2_manipulation.workcells.prepare import compose, prepare
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from tests.helpers import urdf_fk


@dataclass(frozen=True)
class TemporaryWorkcell(Workcell):
    directory: Path

    @property
    def output(self):
        return self.directory


class WorkcellModelTests(unittest.TestCase):
    def setUp(self):
        self.cell = load_workcell()
        if not (self.cell.output / "source.json").exists():
            self.skipTest("Generated workcell is required")
        check_workcell(self.cell)

    def test_all_arm_frames_match_independent_fk_in_three_poses(self):
        model = mujoco.MjModel.from_xml_path(str(self.cell.output / "cell.xml"))
        state = mujoco.MjData(model)
        robot = ET.parse(self.cell.output / "cell.urdf").getroot()
        self.assertEqual((model.nv, model.nu), (12, 12))
        self.assertEqual(len(robot.findall("ros2_control")), 1)
        for values in [
            self.cell.home,
            [
                v
                for i in self.cell.instances
                for v in i.profile.data["verification"]["direct_target_rad"]
            ],
            [v for i in self.cell.instances for v in i.plan_target],
        ]:
            positions = dict(zip(self.cell.joints, values))
            for name, value in positions.items():
                state.joint(name).qpos[0] = value
            mujoco.mj_forward(model, state)
            expected = urdf_fk(robot, positions)
            for inst in self.cell.instances:
                for name in [
                    inst.prefix + "world",
                    inst.prefix + inst.profile.ros["base_frame"],
                    inst.prefix + inst.profile.ros["tip_frame"],
                ]:
                    np.testing.assert_allclose(
                        state.body(name).xpos, expected[name][:3, 3], atol=1e-6
                    )
                    np.testing.assert_allclose(
                        state.body(name).xmat.reshape(3, 3),
                        expected[name][:3, :3],
                        atol=1e-6,
                    )
            for obj in self.cell.data["objects"]:
                name = "workcell_" + obj["name"]
                np.testing.assert_allclose(
                    state.body(name).xpos, expected[name][:3, 3], atol=1e-6
                )

    def test_tilted_instance_mounts_preserve_pose_conventions(self):
        data = deepcopy(self.cell.data)
        data["robots"][0].update(xyz=[-1.5, 0.4, 1.0], rpy=[0.1, -0.2, 0.3])
        data["robots"][1].update(xyz=[1.5, -0.4, 1.0], rpy=[-0.1, 0.2, -0.3])
        instances = tuple(
            Instance(entry, old.profile)
            for entry, old in zip(data["robots"], self.cell.instances)
        )
        cell = replace(self.cell, data=data, instances=instances)
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            compose(cell, directory)
            model = mujoco.MjModel.from_xml_path(str(directory / "cell.xml"))
            state = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, state, 0)
            mujoco.mj_forward(model, state)
            expected = urdf_fk(
                ET.parse(directory / "cell.urdf").getroot(),
                dict(zip(cell.joints, cell.home)),
            )
            for inst in instances:
                name = inst.prefix + inst.profile.ros["tip_frame"]
                np.testing.assert_allclose(
                    state.body(name).xpos, expected[name][:3, 3], atol=1e-6
                )
                np.testing.assert_allclose(
                    state.body(name).xmat.reshape(3, 3),
                    expected[name][:3, :3],
                    atol=1e-6,
                )

    def test_two_fr5_instances_keep_distinct_actuators_and_meshes(self):
        data = deepcopy(self.cell.data)
        data["robots"][1]["model"] = "fr5"
        instances = (
            self.cell.instances[0],
            Instance(data["robots"][1], self.cell.instances[0].profile),
        )
        cell = Workcell(self.cell.root, data, instances)
        with tempfile.TemporaryDirectory() as tmp:
            compose(cell, Path(tmp))
            model = mujoco.MjModel.from_xml_path(str(Path(tmp) / "cell.xml"))
            for inst in instances:
                for name in inst.joints:
                    actuator = model.actuator(name)
                    self.assertEqual(
                        model.actuator_trnid[actuator.id, 0], model.joint(name).id
                    )
                self.assertGreaterEqual(model.mesh(inst.prefix + "base_link").id, 0)
            self.assertNotEqual(
                model.mesh("left_base_link").id, model.mesh("right_base_link").id
            )

    def test_output_edits_and_staged_failures_do_not_hide_stale_builds(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "published"
            shutil.copytree(self.cell.output, directory)
            cell = TemporaryWorkcell(
                self.cell.root, self.cell.data, self.cell.instances, directory
            )
            check_workcell(cell)
            changed = deepcopy(cell.data)
            changed["physics"]["timestep"] = 0.003
            with self.assertRaisesRegex(ValueError, "inputs changed"):
                check_workcell(replace(cell, data=changed))
            path = directory / "controllers.yaml"
            before = path.read_bytes()
            path.write_bytes(before + b"changed")
            with self.assertRaisesRegex(ValueError, "Changed output"):
                check_workcell(cell)
            path.write_bytes(before)
            saved = {p.name: p.read_bytes() for p in directory.iterdir()}
            with patch(
                "mujoco_ros2_manipulation.workcells.prepare.compose",
                side_effect=ValueError("injected build failure"),
            ):
                with self.assertRaisesRegex(ValueError, "injected"):
                    prepare(cell)
            self.assertEqual(
                saved, {p.name: p.read_bytes() for p in directory.iterdir()}
            )
            check_workcell(cell)
