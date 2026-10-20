"""Verify clear initial geometry and surface contact after a substantial approach."""
from pathlib import Path
import tempfile
import unittest
import mujoco
import numpy as np
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.contact_demo import (
    contact_demo_model,
    OBSTACLE_BODY,
)


class VisibleApproachGeometryTests(unittest.TestCase):
    def test_fixed_obstacle_is_initially_clear_and_contact_requires_arm_motion(self):
        cell = load_workcell()
        if not (cell.output / "cell.xml").is_file():
            self.skipTest("Shared workcell not generated")
        (ROOT / "runtime/validation/runs").mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime/validation/runs") as tmp:
            model = mujoco.MjModel.from_xml_path(
                str(contact_demo_model(cell, Path(tmp)))
            )
            state = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, state, 0)
            home = state.qpos.copy()
            obstacle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, OBSTACLE_BODY)
            self.assertEqual(model.nmocap, 0)
            self.assertEqual(model.body_dofnum[obstacle], 0)
            first_contact = None
            for angle in np.linspace(0, 1.2, 121):
                state.qpos[:] = home
                state.qpos[0] += angle
                state.qpos[6] -= angle
                mujoco.mj_forward(model, state)
                np.testing.assert_allclose(
                    state.xpos[obstacle], [-0.45, 0.72, 0.5], atol=1e-12
                )
                if state.ncon:
                    first_contact = angle
                    for contact in state.contact:
                        bodies = {int(model.geom_bodyid[g]) for g in contact.geom}
                        self.assertIn(obstacle, bodies)
                        other = (bodies - {obstacle}).pop()
                        self.assertTrue(
                            mujoco.mj_id2name(
                                model, mujoco.mjtObj.mjOBJ_BODY, other
                            ).startswith("left_wrist")
                        )
                        self.assertGreater(contact.dist, -0.01)
                    break
            self.assertIsNotNone(first_contact)
            self.assertGreater(first_contact, 0.7)
            self.assertLess(first_contact, 1.1)
