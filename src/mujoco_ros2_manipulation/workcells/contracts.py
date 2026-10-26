"""Describe one validated workcell without importing a transport or simulation engine.

This is a description of the current build, not a plugin loader or a live-model
validator. Call check_workcell(cell) first; this function never reads assets.
"""
from copy import deepcopy
import json
import math
from pathlib import PurePosixPath
import re


_ASSET_ROLES = {
    "cell.xml": "mujoco_scene",
    "cell.urdf": "ros_robot_and_collision_description",
    "cell.srdf": "moveit_semantics",
    "controllers.yaml": "ros2_control_configuration",
    "moveit.yaml": "moveit_configuration",
    "moveit.rviz": "rviz_configuration",
}


def _hashes(values, name):
    if not isinstance(values, dict) or not values:
        raise ValueError(name + " must contain asset SHA-256 values")
    for path, digest in values.items():
        if (not isinstance(path, str) or not path
                or PurePosixPath(path).is_absolute() or ".." in PurePosixPath(path).parts
                or str(PurePosixPath(path)) != path):
            raise ValueError(name + " requires normalized repository-relative paths")
        if not isinstance(digest, str) or not re.fullmatch("[0-9a-f]{64}", digest):
            raise ValueError(name + " requires lowercase SHA-256 values")
    return deepcopy(values)


def describe_workcell(cell, validated_manifest):
    """Return a detached JSON-safe v1 description of a checked Workcell.

    The caller supplies the result of check_workcell(cell). Fingerprint, joint
    ownership, home and limits must still match that cell. Files are not opened
    or rehashed here. No path in the result depends on the checkout location.
    """
    manifest = validated_manifest
    names, home = list(cell.joints), list(cell.home)
    if (not isinstance(manifest, dict) or type(manifest.get("schema_version")) is not int
            or manifest["schema_version"] != 1
            or manifest.get("workcell_sha256") != cell.fingerprint
            or manifest.get("joints") != names or manifest.get("home_rad") != home):
        raise ValueError("Validated manifest does not describe this workcell")
    if (not names or len(names) != len(set(names)) or len(home) != len(names)
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in home)):
        raise ValueError("Workcell requires unique named joints and finite home values")
    limits = manifest.get("joint_limits")
    if not isinstance(limits, dict) or set(limits) != set(names):
        raise ValueError("Joint limits must cover every workcell joint exactly once")
    for name, value in zip(names, home):
        bound = limits[name]
        if (not isinstance(bound, dict) or set(bound) != {"lower", "upper", "effort", "velocity"}
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in bound.values())
                or not bound["lower"] <= value <= bound["upper"]
                or bound["effort"] <= 0 or bound["velocity"] <= 0):
            raise ValueError("Invalid rotational joint limits or home: " + name)
    inputs = _hashes(manifest.get("inputs"), "inputs")
    outputs = _hashes(manifest.get("outputs"), "outputs")
    if set(outputs) != set(_ASSET_ROLES):
        raise ValueError("Expected the six current workcell build outputs")
    directory = "runtime/generated/workcells/" + cell.id
    robots = []
    owned = []
    for instance in cell.instances:
        owned.extend(instance.joints)
        robots.append(dict(
            instance=instance.id, model=instance.profile.id, prefix=instance.prefix,
            joint_names=list(instance.joints),
            joint_map=dict(zip(instance.profile.joints, instance.joints)),
            base_frame=instance.prefix + instance.profile.ros["base_frame"],
            tip_frame=instance.prefix + instance.profile.ros["tip_frame"],
            installation_pose=dict(frame="world", xyz_m=instance.data["xyz"],
                                   rpy_rad=instance.data["rpy"]),
            current_ros_adapter=dict(planning_group=instance.group,
                                     trajectory_controller=instance.controller),
        ))
    if owned != names:
        raise ValueError("Instance joint ownership does not match the complete workcell state")
    result = dict(
        schema_version=1, kind="workcell_description", workcell_id=cell.id, label=cell.label,
        identity=dict(workcell_sha256=cell.fingerprint,
                      profile_sha256={i.id: i.profile.fingerprint for i in cell.instances},
                      inputs_sha256=inputs, outputs_sha256=outputs),
        assets=dict(directory=directory, outputs=[
            dict(path=directory + "/" + name, sha256=outputs[name], role=role)
            for name, role in _ASSET_ROLES.items()
        ]),
        units=dict(length="m", joint_position="rad", joint_velocity="rad/s",
                   joint_acceleration="rad/s^2", joint_effort="N*m", time="s",
                   orientation="URDF fixed-axis roll-pitch-yaw in rad",
                   box_size="full side lengths in m", cylinder_size="radius and full height in m"),
        world_frame="world", joint_names=names, joint_limits=limits,
        home_rad=dict(zip(names, home)), robots=robots,
        environment=dict(frame="world", objects=cell.data["objects"]),
        current_adapters=dict(
            simulation="MuJoCo via mujoco_ros2_control/MujocoSystemInterface",
            transport="ROS 2", planning="MoveIt 2 / OMPL",
            execution="FollowJointTrajectory / ros2_control spline interpolation",
            visualization="ROS joint-state rendering with MuJoCo; RViz",
        ),
        scope=dict(joints="rotational", environment="fixed box/cylinder primitives",
                   robot_instances=2, command="complete named joint targets",
                   runtime_plugins=False),
    )
    # This also rejects nonfinite scene/pose values and non-JSON custom values.
    try:
        return json.loads(json.dumps(result, allow_nan=False))
    except (ValueError, TypeError) as error:
        raise ValueError("Workcell description must contain finite JSON values") from error
