"""Compose two verified robots into one 12-joint MuJoCo / ROS / MoveIt world."""

import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from mujoco_ros2_manipulation.workcells.config import load_workcell, controllers
from mujoco_ros2_manipulation.workcells.integrity import (
    snapshot,
    check_workcell,
    OUTPUTS,
)
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.scenes import apply_scene, numbers


def quaternion(rpy):
    r, p, y = [v / 2 for v in rpy]
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def prefixed_urdf(robot, prefix, excluded_objects=("workcell_floor",)):
    """Remove the default world geometry and hardware, prefix every URDF symbol."""
    robot = deepcopy(robot)
    for element in list(robot):
        child = element.find("child")
        excluded = element.get("name") in excluded_objects or (
            element.tag == "joint"
            and child is not None
            and child.get("link") in excluded_objects
        )
        if element.tag in ("ros2_control", "mujoco") or excluded:
            robot.remove(element)
    # All symbols including the source world receive a prefix; common world is added outside.
    for element in robot.iter():
        if element.get("name"):
            element.set("name", prefix + element.get("name"))
        if element.tag in ("parent", "child") and element.get("link"):
            element.set("link", prefix + element.get("link"))
    return robot


def prefixed_mjcf(source, prefix, directory):
    source = deepcopy(source)
    reference = {
        "mesh",
        "material",
        "texture",
        "joint",
        "joint1",
        "joint2",
        "body",
        "body1",
        "body2",
        "site",
        "site1",
        "site2",
        "target",
    }
    for element in source.iter():
        for key, value in list(element.attrib.items()):
            if key == "name" or key in reference:
                element.set(key, prefix + value)
            elif key == "file":
                element.set(key, str((directory / value).resolve()))
    return source


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def moveit_parameters(cell, urdf, srdf, limits):
    groups = {
        i.group: {
            "planner_configs": ["RRTConnectkConfigDefault"],
            "longest_valid_segment_fraction": 0.005,
        }
        for i in cell.instances
    }
    groups["both_arms"] = {
        "planner_configs": ["RRTConnectkConfigDefault"],
        "longest_valid_segment_fraction": 0.005,
    }
    return {
        "robot_description": urdf,
        "robot_description_semantic": srdf,
        "robot_description_kinematics": {
            i.group: {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_timeout": 0.05,
                "kinematics_solver_search_resolution": 0.005,
            }
            for i in cell.instances
        },
        "robot_description_planning": {
            "joint_limits": {
                name: {
                    "has_velocity_limits": True,
                    "max_velocity": float(values["velocity"]),
                    "has_acceleration_limits": True,
                    "max_acceleration": 1.0,
                }
                for name, values in limits.items()
            }
        },
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
            "planner_configs": {
                "RRTConnectkConfigDefault": {
                    "type": "geometric::RRTConnect",
                    "range": 0.0,
                }
            },
            **groups,
        },
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": [i.controller for i in cell.instances],
            **{
                i.controller: {
                    "type": "FollowJointTrajectory",
                    "action_ns": "follow_joint_trajectory",
                    "default": True,
                    "joints": i.joints,
                }
                for i in cell.instances
            },
        },
        "moveit_manage_controllers": True,
        "use_sim_time": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


def compose(cell, directory):
    import mujoco
    import numpy as np

    before = snapshot(cell)
    robot = ET.Element("robot", {"name": cell.id})
    ET.SubElement(robot, "link", {"name": "world"})
    semantic = ET.Element("robot", {"name": cell.id})
    mj = ET.Element("mujoco", {"model": cell.id})
    ET.SubElement(mj, "compiler", {"angle": "radian", "meshdir": "."})
    ET.SubElement(
        mj,
        "option",
        {
            "timestep": str(cell.data["physics"]["timestep"]),
            "integrator": cell.data["physics"]["integrator"],
        },
    )
    assets = ET.SubElement(mj, "asset")
    world = ET.SubElement(mj, "worldbody")
    contact = ET.SubElement(mj, "contact")
    actuator = ET.SubElement(mj, "actuator")
    limits = {}
    for inst in cell.instances:
        base_scene = inst.profile.scene_data
        if any(
            value != 0
            for key in ("xyz", "rpy")
            for value in base_scene["robot_pose"][key]
        ):
            raise ValueError(
                "Workcell templates require a neutral default robot_pose; use workcell instance poses"
            )
        excluded_objects = {"workcell_" + obj["name"] for obj in base_scene["objects"]}
        original = ET.parse(inst.profile.urdf_path).getroot()
        cloned = prefixed_urdf(original, inst.prefix, excluded_objects)
        for child in cloned:
            robot.append(child)
        placement = ET.SubElement(
            robot, "joint", {"name": inst.prefix + "placement", "type": "fixed"}
        )
        ET.SubElement(placement, "parent", {"link": "world"})
        ET.SubElement(placement, "child", {"link": inst.prefix + "world"})
        ET.SubElement(
            placement,
            "origin",
            {"xyz": numbers(inst.data["xyz"]), "rpy": numbers(inst.data["rpy"])},
        )
        group = ET.SubElement(semantic, "group", {"name": inst.group})
        ET.SubElement(
            group,
            "chain",
            {
                "base_link": inst.prefix + inst.profile.ros["base_frame"],
                "tip_link": inst.prefix + inst.profile.ros["tip_frame"],
            },
        )
        for pair in (
            ET.parse(inst.profile.source_path("srdf"))
            .getroot()
            .findall("disable_collisions")
        ):
            attrs = dict(pair.attrib)
            attrs["link1"] = inst.prefix + attrs["link1"]
            attrs["link2"] = inst.prefix + attrs["link2"]
            ET.SubElement(semantic, "disable_collisions", attrs)
        source = prefixed_mjcf(
            ET.parse(inst.profile.scene_path).getroot(),
            inst.prefix,
            inst.profile.output,
        )
        assets.extend(list(source.find("asset")))
        mount = ET.SubElement(
            world,
            "body",
            {
                "name": inst.prefix + "world",
                "pos": numbers(inst.data["xyz"]),
                "quat": numbers(quaternion(inst.data["rpy"])),
            },
        )
        for child in source.find("worldbody"):
            if child.tag == "body" and child.get("name") not in {
                inst.prefix + name for name in excluded_objects
            }:
                mount.append(child)
        for tag, target in [("contact", contact), ("actuator", actuator)]:
            target.extend(list(source.find(tag)))
        metadata = json.loads((inst.profile.output / "source.json").read_text())
        limits.update(
            {
                inst.prefix + name: value
                for name, value in metadata["joint_limits"].items()
            }
        )
        for name, value in zip(inst.joints, inst.plan_target):
            if not limits[name]["lower"] <= value <= limits[name]["upper"]:
                raise ValueError("Workcell target exceeds joint limits")
    both = ET.SubElement(semantic, "group", {"name": "both_arms"})
    for inst in cell.instances:
        ET.SubElement(both, "group", {"name": inst.group})
    home = ET.SubElement(
        semantic, "group_state", {"name": "home", "group": "both_arms"}
    )
    for name, value in zip(cell.joints, cell.home):
        ET.SubElement(home, "joint", {"name": name, "value": str(value)})
    environment = ET.fromstring(
        '<robot name="environment"><link name="anchor"/></robot>'
    )
    apply_scene(
        environment,
        {
            "robot_pose": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]},
            "objects": cell.data["objects"],
        },
        "world",
    )
    for child in environment:
        if (
            child.get("name", "").startswith("workcell_")
            and child.get("name") != "workcell_mount"
        ):
            robot.append(child)
    for obj in cell.data["objects"]:
        body = ET.SubElement(
            world,
            "body",
            {
                "name": "workcell_" + obj["name"],
                "pos": numbers(obj["xyz"]),
                "quat": numbers(quaternion(obj["rpy"])),
            },
        )
        size = (
            [v / 2 for v in obj["size"]]
            if obj["shape"] == "box"
            else [obj["size"][0], obj["size"][1] / 2]
        )
        ET.SubElement(
            body,
            "geom",
            {"type": obj["shape"], "size": numbers(size), "rgba": numbers(obj["rgba"])},
        )
    ET.SubElement(
        world,
        "light",
        {
            "pos": "1 -1 3",
            "dir": "-1 1 -2",
            "directional": "true",
            "ambient": "0.3 0.3 0.3",
        },
    )
    model = mujoco.MjModel.from_xml_string(ET.tostring(mj, encoding="unicode"))
    qpos = model.qpos0.copy()
    for name, value in zip(cell.joints, cell.home):
        qpos[model.joint(name).qposadr[0]] = value
    ET.SubElement(
        ET.SubElement(mj, "keyframe"),
        "key",
        {"name": "home", "qpos": numbers(qpos), "ctrl": numbers(cell.home)},
    )
    write_xml(mj, directory / "cell.xml")
    hardware = ET.SubElement(
        ET.SubElement(
            robot, "ros2_control", {"name": "WorkcellMuJoCo", "type": "system"}
        ),
        "hardware",
    )
    ET.SubElement(hardware, "plugin").text = "mujoco_ros2_control/MujocoSystemInterface"
    for name, value in {
        "mujoco_model": str(cell.output / "cell.xml"),
        "initial_keyframe": "home",
        "headless": "false",
        "sim_speed_factor": str(cell.data["physics"]["sim_speed_factor"]),
    }.items():
        ET.SubElement(hardware, "param", {"name": name}).text = value
    control = robot.find("ros2_control")
    for name, value in zip(cell.joints, cell.home):
        joint = ET.SubElement(control, "joint", {"name": name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        state = ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(state, "param", {"name": "initial_value"}).text = str(value)
        for interface in ("velocity", "effort"):
            ET.SubElement(joint, "state_interface", {"name": interface})
    write_xml(robot, directory / "cell.urdf")
    write_xml(semantic, directory / "cell.srdf")
    (directory / "controllers.yaml").write_text(
        yaml.safe_dump(controllers(cell), sort_keys=False)
    )
    parameters = moveit_parameters(
        cell,
        ET.tostring(robot, encoding="unicode"),
        ET.tostring(semantic, encoding="unicode"),
        limits,
    )
    (directory / "moveit.yaml").write_text(yaml.safe_dump(parameters, sort_keys=False))
    rviz = (
        (cell.root / "packages/ur5e_moveit_config/config/moveit.rviz")
        .read_text()
        .replace("ur5e_manipulator", "both_arms")
        .replace("Target Frame: base_link", "Target Frame: world")
        .replace("Distance: 2.0", "Distance: 3.5")
    )
    (directory / "moveit.rviz").write_text(rviz)
    compiled = mujoco.MjModel.from_xml_path(str(directory / "cell.xml"))
    state = mujoco.MjData(compiled)
    mujoco.mj_resetDataKeyframe(compiled, state, 0)
    for _ in range(2000):
        mujoco.mj_step(compiled, state)
    actual = np.array([state.joint(name).qpos[0] for name in cell.joints])
    if not np.isfinite(actual).all() or np.max(np.abs(actual - cell.home)) > 0.01:
        raise ValueError("Workcell home failed to settle")
    contacts = [
        (
            compiled.body(compiled.geom_bodyid[c.geom1]).name,
            compiled.body(compiled.geom_bodyid[c.geom2]).name,
        )
        for c in state.contact
    ]
    if contacts:
        raise ValueError(f"Unexpected initial contacts: {contacts}")
    if snapshot(cell) != before:
        raise ValueError("Inputs changed during composition")
    return {
        "schema_version": 1,
        "workcell_sha256": cell.fingerprint,
        "joints": cell.joints,
        "home_rad": cell.home,
        "joint_limits": limits,
        "contacts_at_home": contacts,
        "home_error_rad": (actual - cell.home).tolist(),
        "inputs": before,
        "outputs": {name: file_sha256(directory / name) for name in OUTPUTS},
    }


def prepare(cell):
    cell.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix="." + cell.id + "-", dir=cell.output.parent
    ) as tmp:
        staging = Path(tmp)
        report = compose(cell, staging)
        (staging / "source.json").write_text(json.dumps(report, indent=2) + "\n")
        cell.output.mkdir(parents=True, exist_ok=True)
        for name in (*OUTPUTS, "source.json"):
            os.replace(staging / name, cell.output / name)
    check_workcell(cell)
    print(
        json.dumps(
            {
                "workcell": cell.id,
                "joints": len(cell.joints),
                "output": str(cell.output),
                "home_contacts": report["contacts_at_home"],
            }
        )
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell", default="fr5_ur5e")
    args = parser.parse_args()
    try:
        prepare(load_workcell(args.workcell))
    except (ValueError, OSError) as error:
        parser.exit(1, f"Workcell generation failed: {error}\n")
    return 0
