"""Launch descriptions use one selected artifact for physics, state and MoveIt."""
import ast
from pathlib import Path
import os
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET
import yaml


def nodes(namespace):
    path = Path(__file__).resolve().parents[2]/'src/mujoco_ros2_manipulation/workcells/launch.py'
    tree = ast.parse(path.read_text(), filename=str(path))
    body = [item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == 'nodes']
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(path), 'exec'), namespace)
    return namespace['nodes']


class EnvironmentExecutionLaunchTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.base = NS(id='base', root=self.root, output=self.root/'base', instances=[])
        self.base.output.mkdir()
        output = self.root/'environment'; output.mkdir()
        self.target = NS(output=output, validate=Mock())
        robot = '<robot name="environment"><link name="world"/><ros2_control><hardware><plugin>mujoco_ros2_control/MujocoSystemInterface</plugin><param name="headless">false</param><param name="mujoco_model">'+str(output/'cell.xml')+'</param></hardware></ros2_control></robot>'
        semantics = '<robot name="environment"/>'
        (output/'cell.urdf').write_text(robot)
        (output/'moveit.yaml').write_text(yaml.safe_dump(dict(robot_description=robot, robot_description_semantic=semantics)))
        (output/'controllers.yaml').write_text('target: environment\n')
        (output/'moveit.rviz').write_text('target rviz\n')
        guard = self.root/'runtime/native/moveit_plugin_lifetime.so'
        guard.parent.mkdir(parents=True); guard.touch()
        self.factory = Mock(side_effect=lambda **kwargs: NS(**kwargs))
        self.load = Mock(return_value=self.target)
        namespace = dict(ET=ET, os=os, sys=sys, Path=Path, yaml=yaml,
            LaunchConfiguration=lambda name:NS(perform=lambda context:context[name]),
            load_workcell=Mock(return_value=self.base), check_workcell=Mock(),
            check_contact_runtime=Mock(), load_execution_target=self.load,
            Node=self.factory, ExecuteProcess=self.factory,
            RegisterEventHandler=lambda event:event, OnProcessExit=lambda **kwargs:NS(**kwargs),
            stop_when_core_exits=lambda *a:None)
        self.launch = nodes(namespace)
        self.context = dict(workcell='base', execution_environment=str(output),
            execution_manifest_sha256='f'*64, gui='false', rviz='true', contact_test='false',
            contact_demo='false', contact_demo_control='false', clock_test='off', planning_case='none')

    def test_simulation_state_publisher_moveit_and_rviz_share_target_outputs(self):
        self.launch(self.context)
        self.load.assert_called_once_with(self.base, str(self.target.output), 'f'*64)
        described = [call.kwargs for call in self.factory.call_args_list if 'package' in call.kwargs]
        by_package = {item['package']:item for item in described}
        simulation = by_package['mujoco_ros2_control']['parameters']
        state = by_package['robot_state_publisher']['parameters']
        moveit = by_package['moveit_ros_move_group']['parameters'][0]
        self.assertEqual(simulation[0], state[0])
        self.assertEqual(moveit['robot_description'], simulation[0]['robot_description'])
        root = ET.fromstring(moveit['robot_description'])
        self.assertEqual(root.find("ros2_control/hardware/param[@name='mujoco_model']").text,
                         str(self.target.output/'cell.xml'))
        self.assertEqual(root.find("ros2_control/hardware/param[@name='headless']").text, 'true')
        self.assertEqual(ET.fromstring(moveit['robot_description_semantic']).get('name'), 'environment')
        self.assertEqual(simulation[1], str(self.target.output/'controllers.yaml'))
        self.assertEqual(by_package['rviz2']['arguments'], ['-d', str(self.target.output/'moveit.rviz')])
        self.target.validate.assert_called_once()

    def test_bad_manifest_rejects_before_any_ros_process_description(self):
        self.load.side_effect = ValueError('Overlay manifest SHA mismatch')
        with self.assertRaisesRegex(ValueError, 'manifest SHA mismatch'):
            self.launch(self.context)
        self.factory.assert_not_called()

    def test_fault_or_contact_fixtures_cannot_silently_replace_the_selected_model(self):
        for key, value in (('contact_test', 'true'), ('contact_demo', 'true'), ('clock_test', 'missing')):
            with self.subTest(key=key):
                self.factory.reset_mock()
                with self.assertRaisesRegex(ValueError, 'cannot be combined'):
                    self.launch(dict(self.context, **{key:value}))
                self.factory.assert_not_called()
