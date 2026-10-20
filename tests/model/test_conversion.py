"""Regression checks for model selection, stale outputs and joint name mapping."""

from copy import deepcopy
from dataclasses import dataclass
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from mujoco_ros2_manipulation.profiles import (
    ROOT, RobotProfile, check_generated_model, controller_config, load_profile,
    list_profiles, moveit_controller_config, validate_profile,
)


from tests.helpers import TemporaryOutputProfile, urdf_fk

class ConversionTests(unittest.TestCase):
    def test_tip_frames_preserve_urdf_kinematics(self):
        """Compare independent URDF forward kinematics with compiled MJCF bodies."""
        import mujoco

        for profile in list_profiles():
            if not profile.data['enabled']:
                continue
            with self.subTest(robot=profile.id):
                if not profile.scene_path.exists() or not profile.urdf_path.exists():
                    self.skipTest('generated models and manufacturer meshes are required')
                check_generated_model(profile)
                robot = ET.parse(profile.urdf_path).getroot()
                model = mujoco.MjModel.from_xml_path(str(profile.scene_path))
                state = mujoco.MjData(model)
                poses = [profile.home, profile.data['verification']['direct_target_rad'],
                         profile.data['verification']['moveit_target_rad']]
                for pose in poses:
                    coordinates = dict(zip(profile.joints, pose))
                    for name, value in coordinates.items():
                        state.joint(name).qpos[0] = value
                    mujoco.mj_forward(model, state)
                    expected = urdf_fk(robot, coordinates)
                    frames = [profile.ros['tip_frame']]
                    if profile.id == 'ur5e':
                        frames += ['flange', 'wrist_3_link']
                    for name in frames:
                        body = state.body(name)
                        np.testing.assert_allclose(body.xpos, expected[name][:3, 3], atol=1e-6, rtol=0)
                        np.testing.assert_allclose(body.xmat.reshape(3, 3), expected[name][:3, :3],
                                                   atol=1e-6, rtol=0)

    @unittest.skipUnless((ROOT / 'external' / 'frcobot_ros2' / '.git').exists(),
                         'manufacturer URDF / STL checkout is required')
    def test_reordered_ros_joints_preserve_mujoco_home_and_actuators(self):
        import mujoco
        from mujoco_ros2_manipulation.model_build.prepare import prepare

        data = deepcopy(load_profile('fr5').data)
        for values in [data['joints'], data['home_rad'], data['servo']['kp'], data['servo']['kv'],
                       data['verification']['direct_target_rad'],
                       data['verification']['moveit_target_rad']]:
            values.reverse()
        validate_profile(data)
        with tempfile.TemporaryDirectory() as temporary:
            profile = TemporaryOutputProfile(ROOT, data, Path(temporary))
            # Keep test outputs separate from the executable robot's generated controllers.
            with patch('mujoco_ros2_manipulation.model_build.prepare.write_controller_config'), patch('builtins.print'):
                prepare(profile)
            model = mujoco.MjModel.from_xml_path(str(profile.scene_path))
            state = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, state, 0)
            for name, initial, kp in zip(profile.joints, profile.home, data['servo']['kp']):
                self.assertAlmostEqual(state.joint(name).qpos[0], initial)
                actuator_id = int(model.actuator(name).id)
                self.assertAlmostEqual(state.ctrl[actuator_id], initial)
                self.assertAlmostEqual(model.actuator_gainprm[actuator_id, 0], kp)
                self.assertEqual(model.actuator_trnid[actuator_id, 0], model.joint(name).id)


