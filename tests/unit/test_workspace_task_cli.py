"""Reject ambiguous workspace ownership before any ROS startup or worker launch."""
from argparse import Namespace
import unittest
from pathlib import Path

from mujoco_ros2_manipulation.operator_console.__main__ import validate_workspace_options


class WorkspaceTaskCliTests(unittest.TestCase):
    def options(self, **values):
        options = dict(ros_grasp_environment=None, ros_grasp_manifest_sha256=None,
                       ros_grasp_domain=None, domain=218, no_render=False,
                       execution_environment=None, ros_grasp_viewer=None, saved_recording=[])
        options.update(values)
        return Namespace(**options)

    def test_normal_console_remains_optional(self):
        validate_workspace_options(self.options())

    def test_complete_distinct_task_registration(self):
        validate_workspace_options(self.options(ros_grasp_environment=Path('target'),
            ros_grasp_manifest_sha256='a'*64, ros_grasp_domain=219))

    def test_partial_registration_never_falls_back_to_default_domain(self):
        for options in (dict(ros_grasp_environment=Path('target')),
                        dict(ros_grasp_manifest_sha256='a'*64, ros_grasp_domain=219)):
            with self.subTest(options=options), self.assertRaises(ValueError):
                validate_workspace_options(self.options(**options))

    def test_conflicting_execution_and_display_modes_are_rejected(self):
        base = dict(ros_grasp_environment=Path('target'),
                    ros_grasp_manifest_sha256='a'*64, ros_grasp_domain=219)
        for incompatible in (dict(ros_grasp_domain=218), dict(no_render=True),
                             dict(ros_grasp_domain=233), dict(execution_environment=Path('overlay')),
                             dict(ros_grasp_viewer='http://127.0.0.1:8930/')):
            with self.subTest(incompatible=incompatible), self.assertRaises(ValueError):
                validate_workspace_options(self.options(**(base|incompatible)))

    def test_recording_registration_is_bounded_before_loading(self):
        with self.assertRaises(ValueError):
            validate_workspace_options(self.options(saved_recording=[Path('recording')]*9))
