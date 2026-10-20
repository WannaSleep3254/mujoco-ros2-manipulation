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


@dataclass(frozen=True)
class TemporaryOutputProfile(RobotProfile):
    output_directory: Path

    @property
    def output(self):
        return self.output_directory


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
