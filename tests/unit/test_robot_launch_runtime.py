"""Every single-robot launch checks the isolated runtime before describing nodes."""
import ast
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET


def launch_function(namespace):
    path = Path(__file__).resolve().parents[2]/'src/mujoco_ros2_manipulation/ros_launch.py'
    tree = ast.parse(path.read_text())
    function = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'launch_nodes')
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['launch_nodes']


class RobotLaunchRuntimeTests(unittest.TestCase):
    def fixture(self, root, events):
        model = root/'robot.urdf'
        model.write_text('<robot><link name="world"/><ros2_control><hardware>'
                         '<param name="mujoco_model"/><param name="headless"/>'
                         '</hardware></ros2_control></robot>')
        guard = root/'runtime/native/moveit_plugin_lifetime.so'
        guard.parent.mkdir(parents=True)
        guard.touch()
        profile = NS(root=root, urdf_path=model, scene_path=root/'robot.xml',
                     ros=dict(state_controller='joint_state_broadcaster', trajectory_controller='arm_controller',
                              world_frame='world', base_frame='base'))
        factory = Mock(side_effect=lambda **kwargs: events.append('node') or NS(**kwargs))
        namespace = dict(ROOT=root, ET=ET, load_profile=Mock(return_value=profile),
            check_generated_model=Mock(side_effect=lambda _:events.append('model')),
            check_action_clock_runtime=Mock(side_effect=lambda _:events.append('action_clock')),
            LaunchConfiguration=lambda name:NS(perform=lambda context:context[name]),
            write_controller_config=lambda _:root/'controllers.yaml', Node=factory,
            RegisterEventHandler=lambda event:event, OnProcessExit=lambda **kwargs:NS(**kwargs),
            stop_when_core_exits=lambda *args:None)
        return launch_function(namespace), namespace, factory

    def test_missing_or_shadowed_action_overlay_blocks_all_supported_robot_launches(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            launch, namespace, factory = self.fixture(root, [])
            namespace['check_action_clock_runtime'].side_effect = ValueError('isolated action runtime rejected')
            for robot in ('fr3', 'fr5', 'fr10', 'ur5e'):
                for moveit in ('true', 'false'):
                    with self.subTest(robot=robot, moveit=moveit):
                        with self.assertRaisesRegex(ValueError, 'isolated action runtime rejected'):
                            launch(dict(robot=robot, scene='default', gui='false', moveit=moveit, rviz='false'))
                        namespace['check_action_clock_runtime'].assert_called_with(root)
                        factory.assert_not_called()

    def test_valid_overlay_preserves_existing_headless_node_descriptions(self):
        with tempfile.TemporaryDirectory() as temporary:
            events = []
            launch, namespace, factory = self.fixture(Path(temporary), events)
            nodes = launch(dict(robot='fr5', scene='default', gui='false', moveit='false', rviz='false'))
            self.assertEqual(events[:2], ['model', 'action_clock'])
            self.assertEqual(factory.call_count, 4)
            self.assertEqual(len(nodes), 5)
            self.assertEqual(factory.call_args_list[0].kwargs['executable'], 'ros2_control_node')
            namespace['check_generated_model'].assert_called_once()
