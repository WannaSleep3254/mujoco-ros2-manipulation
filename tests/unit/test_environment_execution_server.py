"""Automatic/reviewed production request paths refuse changed target identities."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET
import hashlib

from mujoco_ros2_manipulation.workcells.motion_request import command
from tests.unit import test_motion_review_server as review_tests


class EnvironmentExecutionServerTests(unittest.TestCase):
    def setUp(self):
        self.fixture = review_tests.ReviewServerTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.node = self.fixture.node
        self.description = dict(artifact_kind='environment-execution-target-v1', manifest_sha256='f'*64)
        self.node.execution_target = NS(validate=Mock(return_value=deepcopy(self.description)),
            description=Mock(return_value=deepcopy(self.description)))

    def run_command(self):
        payload = command(dict(left=.1, right=.1))
        self.fixture.send(payload, 'command')
        self.node.process_next()
        return json.loads((self.node.directory/(payload['request_id']+'.json')).read_text())

    def test_automatic_execution_checks_runtime_before_planning_and_immediately_before_dispatch(self):
        value = self.run_command()
        self.assertEqual(value['status'], 'succeeded', value['error'])
        self.assertEqual(value['dispatched_goals'], 2)
        self.assertEqual(value['execution_target'], self.description)
        self.assertEqual(value['execution_target_identity'], self.fixture.identity)
        self.assertEqual(self.node._review_runtime_identity.call_count, 2)
        self.assertEqual(self.node.execution_target.validate.call_count, 2)
        self.node.dispatch.assert_called_once()

    def test_changed_file_at_initial_gate_blocks_and_sends_zero_goals(self):
        self.node.execution_target.validate.side_effect = ValueError('output changed')
        value = self.run_command()
        self.assertEqual(value['status'], 'failed')
        self.assertEqual(value['failure_kind'], 'execution_environment_changed')
        self.assertEqual(value['dispatched_goals'], 0)
        self.assertTrue(self.node.blocked)
        self.node.coordinated_path.assert_not_called()
        self.node.dispatch.assert_not_called()

    def test_changed_file_after_planning_is_refused_at_last_gate_without_dispatch(self):
        def plan(*_):
            self.node.execution_target.validate.side_effect = ValueError('late file change')
            return self.fixture.curve
        self.node.coordinated_path.side_effect = plan
        value = self.run_command()
        self.assertEqual(value['status'], 'failed')
        self.assertEqual(value['failure_kind'], 'execution_environment_changed')
        self.assertEqual(value['dispatched_goals'], 0)
        self.node.dispatch.assert_not_called()
        self.assertTrue(self.node.blocked)

    def test_runtime_model_or_scene_change_after_planning_sends_zero_goals(self):
        changed = deepcopy(self.fixture.identity)
        changed['planning_scene_sha256'] = 'c'*64
        self.node._review_runtime_identity.side_effect = [deepcopy(self.fixture.identity), changed]
        value = self.run_command()
        self.assertEqual(value['failure_kind'], 'execution_environment_changed')
        self.assertEqual(value['dispatched_goals'], 0)
        self.node.dispatch.assert_not_called()

    def test_new_request_cannot_adopt_a_scene_that_changed_after_startup(self):
        startup = deepcopy(self.fixture.identity)
        changed = deepcopy(startup)
        changed['planning_scene_sha256'] = 'e'*64
        self.node.execution_runtime_identity = startup
        self.node._review_runtime_identity.return_value = changed
        value = self.run_command()
        self.assertEqual(value['failure_kind'], 'execution_environment_changed')
        self.assertEqual(value['dispatched_goals'], 0)
        self.node.coordinated_path.assert_not_called()
        self.node.dispatch.assert_not_called()
        self.assertTrue(self.node.blocked)

    def test_reviewed_plan_cannot_dispatch_after_target_changes(self):
        _, _, approval = self.fixture.make_review()
        self.node.execution_target.validate.side_effect = ValueError('target changed after review')
        self.fixture.send(approval, 'approve')
        self.node.process_next()
        value = json.loads((self.node.directory/(approval['request_id']+'.json')).read_text())
        self.assertEqual(value['status'], 'failed')
        self.assertEqual(value['dispatched_goals'], 0)
        self.node.dispatch.assert_not_called()
        self.assertTrue(self.node.blocked)

    def test_real_runtime_identity_rejects_wrong_simulator_or_moveit_environment_before_send(self):
        output = self.node.directory/'environment'; output.mkdir()
        robot = '<robot name="installed"><link name="world"/><ros2_control><hardware><param name="headless">false</param><param name="mujoco_model">'+str(output/'cell.xml')+'</param></hardware></ros2_control></robot>'
        semantics = '<robot name="installed"/>'
        (output/'cell.urdf').write_text(robot); (output/'cell.srdf').write_text(semantics)
        self.node.cell.output = self.node.directory/'base'
        self.node.execution_target.output = output
        self.node.get_publishers_info_by_topic = Mock(return_value=[object()])
        self.node.count_publishers = Mock(return_value=1)
        self.node.until = lambda predicate, *args: self.assertTrue(predicate())
        for key in ('hardware', 'moveit', 'scene'):
            setattr(self.node, 'review_'+key+'_client', NS(service_is_ready=lambda:True,
                call_async=lambda request, key=key:key))
        scene = NS(world={}, robot_state=NS(attached_collision_objects=[]), allowed_collision_matrix={},
            link_padding=[], link_scale=[], fixed_frame_transforms=[])
        values = lambda strings:[NS(string_value=value, bool_value=True) for value in strings]
        real_method = review_tests.Server._review_runtime_identity.__get__(self.node)
        self.node._review_runtime_identity = real_method
        namespace = real_method.__func__.__globals__
        stubs = dict(ET=ET, hashlib=hashlib, GetParameters=NS(Request=lambda **kw:NS(**kw)),
            PlanningSceneComponents=NS(WORLD_OBJECT_GEOMETRY=1, ROBOT_STATE_ATTACHED_OBJECTS=2,
                ALLOWED_COLLISION_MATRIX=4, LINK_PADDING_AND_SCALING=8, TRANSFORMS=16),
            GetPlanningScene=NS(Request=lambda:NS(components=NS())), message_to_ordereddict=lambda value:value)
        for mismatch in ('simulator', 'moveit'):
            with self.subTest(mismatch=mismatch), patch.dict(namespace, stubs):
                self.node.blocked = False
                observed_robot = robot.replace(str(output/'cell.xml'), str(self.node.cell.output/'cell.xml')) if mismatch == 'simulator' else robot
                answers = dict(hardware=NS(values=values([observed_robot, ''])),
                    moveit=NS(values=values([robot, semantics.replace('installed', 'other')] if mismatch == 'moveit' else [robot, semantics])),
                    scene=NS(scene=scene))
                self.node.result = lambda future,*args:answers[future]
                value = self.run_command()
                self.assertEqual(value['failure_kind'], 'execution_environment_changed')
                self.assertEqual(value['dispatched_goals'], 0)
                self.node.dispatch.assert_not_called()
                self.assertTrue(self.node.blocked)
