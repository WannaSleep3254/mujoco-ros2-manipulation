"""FAIRINO FR5 V6: MuJoCo physics + ros2_control + optional MoveIt / RViz."""

from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


ROOT = Path(__file__).resolve().parent


def stop_when_core_exits(event, context):
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(reason=f'FR5 core process exited ({event.returncode})'))]


def launch_nodes(context):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    moveit = LaunchConfiguration('moveit').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    model_path = ROOT / 'models' / 'fr5' / 'fr5.ros2_control.urdf'
    robot = ET.parse(model_path).getroot()
    robot.find("./ros2_control/hardware/param[@name='headless']").text = str(not gui).lower()
    robot_description = {'robot_description': ET.tostring(robot, encoding='unicode')}
    controllers_path = str(ROOT / 'config' / 'fr5_controllers.yaml')
    plugin_guard = ROOT / 'runtime' / 'native' / 'fr5_plugin_lifetime.so'
    if not plugin_guard.is_file():
        raise RuntimeError('FR5 runtime is missing. Run bash build_fr5_runtime.sh first.')
    core_nodes = []
    simulation = Node(package='mujoco_ros2_control', executable='ros2_control_node',
                      output='screen',
                      parameters=[robot_description, controllers_path, {'use_sim_time': True}])
    core_nodes.append(simulation)
    nodes = [simulation,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description, {'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='world_to_base',
             arguments=['--x', '0', '--y', '0', '--z', '0',
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'world', '--child-frame-id', 'base_link']),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '-c', '/controller_manager',
                        '--controller-manager-timeout', '30'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['fairino5_controller', '-c', '/controller_manager',
                        '--controller-manager-timeout', '30'], output='screen'),
    ]
    if moveit or rviz:
        config = (MoveItConfigsBuilder('fairino5_v6_robot',
                                      package_name='fairino5_v6_moveit2_config')
                  .robot_description(file_path=str(model_path))
                  .planning_pipelines(pipelines=['ompl'])
                  .to_moveit_configs())
        config.robot_description.update(robot_description)
        if moveit:
            move_group = Node(
                package='moveit_ros_move_group', executable='move_group', output='screen',
                additional_env={'LD_PRELOAD': str(plugin_guard)},
                parameters=[config.to_dict(), {'use_sim_time': True,
                    'publish_robot_description': True,
                    'publish_robot_description_semantic': True}])
            nodes.append(move_group)
            core_nodes.append(move_group)
        if rviz:
            from ament_index_python.packages import get_package_share_directory
            rviz_config = (Path(get_package_share_directory('fairino5_v6_moveit2_config'))
                           / 'config' / 'moveit.rviz')
            rviz_node = Node(
                package='rviz2', executable='rviz2', output='screen',
                additional_env={'LD_PRELOAD': str(plugin_guard)},
                arguments=['-d', str(rviz_config)],
                parameters=[config.to_dict(), {'use_sim_time': True}])
            nodes.append(rviz_node)
            core_nodes.append(rviz_node)
    # Closing a primary window also stops the other FR5 processes.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
                    on_exit=stop_when_core_exits)) for node in core_nodes]
    return handlers + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('moveit', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_nodes),
    ])
