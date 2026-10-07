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

from robot_config import (
    ROOT, RobotProfile, check_generated_model, controller_config, load_profile,
    list_profiles, moveit_controller_config, validate_profile,
)


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile('fr5')

    def test_pending_models_require_further_configuration(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            directory = root / 'config' / 'robots'
            directory.mkdir(parents=True)
            data = {'schema_version': 1, 'id': 'future_robot', 'label': 'Future robot',
                    'enabled': False, 'status': 'planned', 'pending_reason': 'Not configured'}
            (directory / 'future_robot.yaml').write_text(yaml.safe_dump(data))
            pending = load_profile('future_robot', require_enabled=False, root=root)
            self.assertFalse(pending.data['enabled'])
            with self.assertRaisesRegex(ValueError, 'Not configured'):
                load_profile('future_robot', root=root)

    def test_unknown_model_has_clear_error(self):
        with self.assertRaisesRegex(ValueError, 'Unknown robot'):
            load_profile('missing_robot')

    def test_malformed_joint_configuration_is_rejected(self):
        for change in ['short_home', 'duplicate_joint', 'nonfinite_gain']:
            with self.subTest(change=change):
                data = deepcopy(self.profile.data)
                if change == 'short_home':
                    data['home_rad'].pop()
                elif change == 'duplicate_joint':
                    data['joints'][1] = data['joints'][0]
                else:
                    data['servo']['kp'][0] = float('nan')
                with self.assertRaises(ValueError):
                    validate_profile(data)

    def test_controllers_use_new_names_and_joint_count(self):
        data = deepcopy(self.profile.data)
        data['joints'] = [f'axis_{index}' for index in range(1, 8)]
        data['ros']['trajectory_controller'] = 'other_arm_controller'
        data['ros']['state_controller'] = 'other_state_broadcaster'
        for values in [data['home_rad'], data['servo']['kp'], data['servo']['kv'],
                       data['verification']['direct_target_rad'],
                       data['verification']['moveit_target_rad']]:
            values.append(values[-1])
        validate_profile(data)
        generated = controller_config(RobotProfile(ROOT, data))
        manager = generated['controller_manager']['ros__parameters']
        self.assertIn('other_arm_controller', manager)
        self.assertIn('other_state_broadcaster', manager)
        self.assertNotIn('fairino5_controller', manager)
        parameters = generated['other_arm_controller']['ros__parameters']
        self.assertEqual(parameters['joints'], data['joints'])
        self.assertTrue(all(joint in parameters['constraints'] for joint in data['joints']))
        moveit = moveit_controller_config(RobotProfile(ROOT, data))
        self.assertEqual(moveit['moveit_simple_controller_manager']['other_arm_controller']['joints'],
                         parameters['joints'])

    def test_changed_profile_requires_model_regeneration(self):
        with tempfile.TemporaryDirectory() as temporary:
            profile = RobotProfile(Path(temporary), deepcopy(self.profile.data))
            profile.output.mkdir(parents=True)
            profile.scene_path.touch()
            profile.urdf_path.touch()
            (profile.output / 'source.json').write_text(json.dumps({
                'robot_profile_sha256': profile.fingerprint}))
            check_generated_model(profile)
            profile.data['servo']['kp'][0] += 1
            with self.assertRaisesRegex(ValueError, 'profile changed'):
                check_generated_model(profile)

    def test_profile_cannot_use_paths_outside_project(self):
        for path in ['/etc/passwd', '../outside-model.xml']:
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.profile.path(path)


@dataclass(frozen=True)
class TemporaryOutputProfile(RobotProfile):
    output_directory: Path

    @property
    def output(self):
        return self.output_directory


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
        from prepare_robot import prepare

        data = deepcopy(load_profile('fr5').data)
        for values in [data['joints'], data['home_rad'], data['servo']['kp'], data['servo']['kv'],
                       data['verification']['direct_target_rad'],
                       data['verification']['moveit_target_rad']]:
            values.reverse()
        validate_profile(data)
        with tempfile.TemporaryDirectory() as temporary:
            profile = TemporaryOutputProfile(ROOT, data, Path(temporary))
            # Keep test outputs separate from the executable robot's generated controllers.
            with patch('prepare_robot.write_controller_config'), patch('builtins.print'):
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


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def urdf_fk(robot, coordinates):
    remaining = list(robot.findall('joint'))
    children = {joint.find('child').get('link') for joint in remaining}
    root, = {link.get('name') for link in robot.findall('link')} - children
    transforms = {root: np.eye(4)}
    while remaining:
        progressed = False
        for joint in list(remaining):
            parent = joint.find('parent').get('link')
            if parent not in transforms:
                continue
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = (axis_rotation([0, 0, 1], rpy[2]) @
                                 axis_rotation([0, 1, 0], rpy[1]) @
                                 axis_rotation([1, 0, 0], rpy[0]))
            if joint.get('type') != 'fixed':
                axis = joint.find('axis')
                vector = [float(v) for v in axis.get('xyz', '1 0 0').split()]
                transform[:3, :3] = transform[:3, :3] @ axis_rotation(vector, coordinates[joint.get('name')])
            transforms[joint.find('child').get('link')] = transforms[parent] @ transform
            remaining.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError('URDF joint tree is disconnected or cyclic')
    return transforms


if __name__ == '__main__':
    unittest.main()
