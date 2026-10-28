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
from mujoco_ros2_manipulation.workcells.execution_target import load_execution_target
from mujoco_ros2_manipulation.workcells.contact_demo import (
    contact_demo_model,
    add_demo_urdf,
)
from mujoco_ros2_manipulation.workcells.contact_policy import (
    check_contact_runtime,
    contact_test_model,
)


def nodes(context):
    cell = load_workcell(LaunchConfiguration("workcell").perform(context))
    check_workcell(cell)
    target = load_execution_target(cell,
        LaunchConfiguration("execution_environment").perform(context) or None,
        LaunchConfiguration("execution_manifest_sha256").perform(context) or None)
    output = cell.output if target is None else target.output
    check_contact_runtime(cell.root)
    contact_test = (
        LaunchConfiguration("contact_test").perform(context).lower() == "true"
    )
    contact_demo = (
        LaunchConfiguration("contact_demo").perform(context).lower() == "true"
    )
    control = (
        LaunchConfiguration("contact_demo_control").perform(context).lower() == "true"
    )
    if control and not contact_demo:
        raise ValueError("Control requires contact_demo")
    if contact_test and contact_demo:
        raise ValueError("Select either injected contact test or fixed-obstacle demo")
    clock_test = LaunchConfiguration("clock_test").perform(context)
    if clock_test not in ("off", "missing", "repeated"):
        raise ValueError("clock_test must be off, missing or repeated")
    if target is not None and (contact_test or contact_demo or clock_test != "off"):
        raise ValueError("Execution environments cannot be combined with model/fault-injection fixtures")
    if clock_test != "off":
        if contact_test or contact_demo:
            raise ValueError("Clock injection cannot be combined with contact fixtures")
        directory = Path(os.environ["MANIPULATION_DIAGNOSTICS_PATH"]).resolve().parent
        if not directory.is_relative_to(cell.root / "runtime/validation/runs"):
            raise ValueError(
                "Clock injection requires an isolated verification directory"
            )
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"
    robot = ET.parse(output / "cell.urdf").getroot()
    robot.find("ros2_control/hardware/param[@name='headless']").text = str(
        not gui
    ).lower()
    if contact_test or contact_demo:
        path = Path(os.environ["MANIPULATION_DIAGNOSTICS_PATH"])
        fixture = (
            contact_demo_model(cell, path.parent, obstacle=not control)
            if contact_demo
            else contact_test_model(cell, path.parent)
        )
        if contact_demo and not control:
            add_demo_urdf(robot)
        robot.find("ros2_control/hardware/param[@name='mujoco_model']").text = str(
            fixture
        )
    description = {"robot_description": ET.tostring(robot, encoding="unicode")}
    guard = cell.root / "runtime/native/moveit_plugin_lifetime.so"
    if not guard.is_file():
        raise RuntimeError("Build runtime before launching the workcell")
    simulation = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        additional_env={
            "LD_PRELOAD": str(guard),
            "MANIPULATION_PIN_CONTROL_PLUGINS": "1",
            "MANIPULATION_CONTACT_STOP": "1",
            "MANIPULATION_CLOCK_TEST_MODE": clock_test,
            "MANIPULATION_ENABLE_CONTACT_TEST": "1" if contact_test else "0",
        },
        output="screen",
        parameters=[
            description,
            str(output / "controllers.yaml"),
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
    config = yaml.safe_load((output / "moveit.yaml").read_text())
    planning_case = LaunchConfiguration("planning_case").perform(context)
    if planning_case != "none":
        from mujoco_ros2_manipulation.workcells.interference import load_case

        _, scenario = load_case(cell, planning_case)
        settings = scenario["planning"]
        for key in ("path_tolerance", "resample_dt"):
            config["ompl"][key] = settings[key]
        for group in ["both_arms"] + [i.group for i in cell.instances]:
            config["ompl"][group]["longest_valid_segment_fraction"] = settings[
                "longest_valid_segment_fraction"
            ]

    config.update(description)
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
            arguments=["-d", str(output / "moveit.rviz")],
            parameters=[config],
        )
        result.append(window)
        core.append(window)
    if target is not None:
        target.validate()
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
            DeclareLaunchArgument("execution_environment", default_value=""),
            DeclareLaunchArgument("execution_manifest_sha256", default_value=""),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("contact_test", default_value="false"),
            DeclareLaunchArgument("clock_test", default_value="off"),
            DeclareLaunchArgument("planning_case", default_value="none"),
            DeclareLaunchArgument("contact_demo", default_value="false"),
            DeclareLaunchArgument("contact_demo_control", default_value="false"),
            OpaqueFunction(function=nodes),
        ]
    )
