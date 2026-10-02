"""Regression checks for model selection, stale outputs and joint name mapping."""

from copy import deepcopy
from dataclasses import dataclass
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from robot_config import (
    ROOT, RobotProfile, check_generated_model, controller_config, load_profile,
    validate_profile,
)


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = load_profile('fr5')

    def test_pending_models_require_further_configuration(self):
        for robot_id in ['fr10', 'ur5e']:
            with self.subTest(robot_id=robot_id):
                pending = load_profile(robot_id, require_enabled=False)
                self.assertFalse(pending.data['enabled'])
                with self.assertRaisesRegex(ValueError, pending.data['pending_reason']):
                    load_profile(robot_id)

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


if __name__ == '__main__':
    unittest.main()
