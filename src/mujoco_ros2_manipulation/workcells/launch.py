"""One physics instance, one full robot state, two arm controllers and MoveIt."""

import xml.etree.ElementTree as ET
import os
import sys
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mujoco_ros2_manipulation.ros_launch import stop_when_core_exits
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.contact_policy import (
    check_contact_runtime,
    contact_test_model,
)


def nodes(context):
    cell = load_workcell(LaunchConfiguration("workcell").perform(context))
    check_workcell(cell)
    check_contact_runtime(cell.root)
    contact_test = (
        LaunchConfiguration("contact_test").perform(context).lower() == "true"
    )
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    robot = ET.parse(cell.output / "cell.urdf").getroot()
    robot.find("ros2_control/hardware/param[@name='headless']").text = str(
        not gui
    ).lower()
    if contact_test:
        path = Path(os.environ["MANIPULATION_DIAGNOSTICS_PATH"])
        fixture = contact_test_model(cell, path.parent)
        robot.find("ros2_control/hardware/param[@name='mujoco_model']").text = str(
            fixture
        )
    description = {"robot_description": ET.tostring(robot, encoding="unicode")}
    simulation = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        additional_env={
            "MANIPULATION_CONTACT_STOP": "1",
            "MANIPULATION_ENABLE_CONTACT_TEST": "1" if contact_test else "0",
        },
        output="screen",
        parameters=[
            description,
            str(cell.output / "controllers.yaml"),
            {"use_sim_time": True},
        ],
    )
    result = [
        simulation,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[description, {"use_sim_time": True}],
        ),
    ]
    for name in ["joint_state_broadcaster"] + [i.controller for i in cell.instances]:
        result.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    name,
                    "-c",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "30",
                ],
                output="screen",
            )
        )
    config = yaml.safe_load((cell.output / "moveit.yaml").read_text())
    config.update(description)
    guard = cell.root / "runtime/native/moveit_plugin_lifetime.so"
    if not guard.is_file():
        raise RuntimeError("Build runtime before launching the workcell")
    movegroup = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env={"LD_PRELOAD": str(guard)},
        parameters=[config],
    )
    result.append(movegroup)
    contact_guard = ExecuteProcess(
        cmd=[
            sys.executable,
            "-m",
            "mujoco_ros2_manipulation.workcells.contact_guard",
            "--controllers",
            *[i.controller for i in cell.instances],
        ],
        output="screen",
    )
    result.append(contact_guard)
    core = [simulation, movegroup, contact_guard]
    if rviz:
        window = Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            additional_env={"LD_PRELOAD": str(guard)},
            arguments=["-d", str(cell.output / "moveit.rviz")],
            parameters=[config],
        )
        result.append(window)
        core.append(window)
    return [
        RegisterEventHandler(
            OnProcessExit(target_action=node, on_exit=stop_when_core_exits)
        )
        for node in core
    ] + result


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("workcell", default_value="fr5_ur5e"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("contact_test", default_value="false"),
            OpaqueFunction(function=nodes),
        ]
    )
