"""Load model profiles and derive controller configuration without ROS imports."""

from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re

import yaml


ROOT = Path(__file__).resolve().parent


@dataclass(frozen=True)
class RobotProfile:
    root: Path
    data: dict

    @property
    def id(self):
        return self.data['id']

    @property
    def label(self):
        return self.data['label']

    @property
    def joints(self):
        return self.data['joints']

    @property
    def home(self):
        return self.data['home_rad']

    @property
    def ros(self):
        return self.data['ros']

    @property
    def output(self):
        return self.root / 'models' / self.id

    @property
    def scene_path(self):
        return self.output / f'{self.id}.xml'

    @property
    def urdf_path(self):
        return self.output / f'{self.id}.ros2_control.urdf'

    @property
    def fingerprint(self):
        encoded = json.dumps(self.data, sort_keys=True, separators=(',', ':')).encode()
        return hashlib.sha256(encoded).hexdigest()

    def source_path(self, key):
        return self.path(self.data['source'][key])

    def path(self, value):
        candidate = Path(value)
        if candidate.is_absolute():
            raise ValueError(f'{self.id}: use a repository-relative path: {value}')
        resolved = (self.root / candidate).resolve()
        if not resolved.is_relative_to(self.root.resolve()):
            raise ValueError(f'{self.id}: path escapes the project: {value}')
        return resolved


def _number(value, name, *, minimum=None, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name}: expected a finite number')
    if (positive and value <= 0) or (minimum is not None and value < minimum):
        raise ValueError(f'{name}: invalid value {value}')


def validate_profile(data):
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('Expected a robot profile with schema_version: 1')
    for name in ['id', 'label', 'status']:
        if not isinstance(data.get(name), str) or not data[name].strip():
            raise ValueError(f'Profile requires {name}')
    if not re.fullmatch(r'[a-z][a-z0-9_]*', data['id']):
        raise ValueError('Invalid robot id')
    if not isinstance(data.get('enabled'), bool):
        raise ValueError('Profile requires enabled: true or false')
    if not data['enabled']:
        if not data.get('pending_reason'):
            raise ValueError('A pending profile requires pending_reason')
        return
    try:
        joints = data['joints']
        if (not isinstance(joints, list) or not joints or
                any(not isinstance(name, str) or not name for name in joints) or
                len(joints) != len(set(joints))):
            raise ValueError('joints must be a nonempty list of unique names')
        vectors = {'home_rad': data['home_rad'], 'servo.kp': data['servo']['kp'],
                   'servo.kv': data['servo']['kv'],
                   'verification.direct_target_rad': data['verification']['direct_target_rad'],
                   'verification.moveit_target_rad': data['verification']['moveit_target_rad']}
        for name, values in vectors.items():
            if not isinstance(values, list) or len(values) != len(joints):
                raise ValueError(f'{name}: expected {len(joints)} joint values')
            for value in values:
                _number(value, name, minimum=0 if name.startswith('servo.') else None)
        for name in ['joint_damping', 'joint_armature']:
            _number(data['servo'][name], 'servo.' + name, minimum=0)
        for name in ['timestep', 'sim_speed_factor']:
            _number(data['physics'][name], 'physics.' + name, positive=True)
        if data['physics']['integrator'] not in ['Euler', 'RK4', 'implicit', 'implicitfast']:
            raise ValueError('Unsupported physics integrator')
        if data['source']['format'] not in ['urdf', 'xacro']:
            raise ValueError('The current converter supports URDF and Xacro sources')
        if data['source'].get('mujoco_visuals', 'original') not in ['original', 'collision', 'dae']:
            raise ValueError('source.mujoco_visuals must be original, collision or dae')
        for name in ['robot_name', 'system_name', 'moveit_package', 'planning_group',
                     'trajectory_controller', 'state_controller', 'world_frame',
                     'base_frame', 'tip_frame']:
            if not isinstance(data['ros'][name], str) or not data['ros'][name]:
                raise ValueError('ros.' + name + ' must be nonempty')
        frames = data['ros']['tf_child_frames']
        if not isinstance(frames, list) or not frames or any(not isinstance(f, str) for f in frames):
            raise ValueError('ros.tf_child_frames must list expected robot frames')
        for name in ['trajectory_duration_sec', 'tracking_tolerance_rad',
                     'goal_constraint_tolerance_rad']:
            _number(data['verification'][name], 'verification.' + name, positive=True)
        count = data['verification']['settle_messages']
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError('verification.settle_messages must be a positive integer')
        for name in ['velocity_scaling', 'acceleration_scaling']:
            value = data['verification'][name]
            _number(value, 'verification.' + name, positive=True)
            if value > 1:
                raise ValueError('verification.' + name + ' must not exceed 1')
    except KeyError as error:
        raise ValueError(f'Missing profile field: {error.args[0]}') from error


def load_profile(robot_id='fr5', *, require_enabled=True, root=ROOT):
    if not re.fullmatch(r'[a-z][a-z0-9_]*', robot_id):
        raise ValueError(f'Invalid robot id: {robot_id}')
    path = Path(root) / 'config' / 'robots' / f'{robot_id}.yaml'
    if not path.is_file():
        raise ValueError(f'Unknown robot {robot_id}; see config/robots')
    data = yaml.safe_load(path.read_text())
    validate_profile(data)
    if data['id'] != robot_id:
        raise ValueError(f'Profile id must match filename: {path.name}')
    if require_enabled and not data['enabled']:
        raise ValueError(f'{data["label"]}: {data["pending_reason"]}')
    return RobotProfile(Path(root).resolve(), data)


def list_profiles():
    return [load_profile(path.stem, require_enabled=False)
            for path in sorted((ROOT / 'config' / 'robots').glob('*.yaml'))]


def controller_config(profile):
    defaults = yaml.safe_load((profile.root / 'config' / 'controller_defaults.yaml').read_text())
    parameters = deepcopy(defaults['trajectory_controller'])
    tolerances = parameters.pop('joint_tolerances')
    parameters['joints'] = list(profile.joints)
    parameters['constraints'].update({joint: dict(tolerances) for joint in profile.joints})
    ros = profile.ros
    return {
        'controller_manager': {'ros__parameters': {
            'update_rate': defaults['update_rate'],
            ros['state_controller']: {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            ros['trajectory_controller']: {'type': 'joint_trajectory_controller/JointTrajectoryController'}}},
        ros['trajectory_controller']: {'ros__parameters': parameters},
    }


def write_controller_config(profile):
    output = profile.root / 'runtime' / 'generated' / profile.id / 'controllers.yaml'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(controller_config(profile), sort_keys=False))
    return output


def moveit_controller_config(profile):
    name = profile.ros['trajectory_controller']
    return {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': {
            'controller_names': [name],
            name: {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                   'default': True, 'joints': list(profile.joints)},
        },
    }


def write_moveit_controller_config(profile):
    output = profile.root / 'runtime' / 'generated' / profile.id / 'moveit_controllers.yaml'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(moveit_controller_config(profile), sort_keys=False))
    return output


def check_generated_model(profile):
    for path in [profile.scene_path, profile.urdf_path, profile.output / 'source.json']:
        if not path.is_file():
            raise ValueError(f'Missing {path.name}; run .venv/bin/python prepare_robot.py --robot {profile.id}')
    metadata = json.loads((profile.output / 'source.json').read_text())
    if metadata.get('robot_profile_sha256') != profile.fingerprint:
        raise ValueError(f'{profile.id} profile changed; run .venv/bin/python prepare_robot.py --robot {profile.id}')
