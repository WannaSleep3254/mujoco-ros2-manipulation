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
            # This test covers the profile gate; complete manifests are tested separately.
            with patch('mujoco_ros2_manipulation.profiles.check_model_integrity') as integrity:
                check_generated_model(profile)
                integrity.assert_called_once()
            profile.data['servo']['kp'][0] += 1
            with self.assertRaisesRegex(ValueError, 'profile changed'):
                check_generated_model(profile)

    def test_profile_cannot_use_paths_outside_project(self):
        for path in ['/etc/passwd', '../outside-model.xml']:
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.profile.path(path)


