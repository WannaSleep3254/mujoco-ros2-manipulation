"""Verify a configured MuJoCo / ros2_control / MoveIt robot in an isolated ROS domain."""

import argparse
from datetime import datetime, timezone
import math
import re
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint

from mujoco_ros2_manipulation.profiles import ROOT
from mujoco_ros2_manipulation.verification.runner import run_verification

class Verification(Node):
    def __init__(self, profile, progress=None):
        super().__init__(f'{profile.id}_verification')
        self.progress = progress or (lambda stage: None)
        self.profile = profile
        self.joint_names = profile.joints
        self.settings = profile.data['verification']
        self.positions = None
        self.joint_count = 0
        self.clock_count = 0
        self.clock_first = None
        self.clock_last = None
        self.clock_backwards = False
        self.transforms = set()
        self.create_subscription(JointState, '/joint_states', self.joints,
                                 qos_profile_sensor_data)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.tf, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf_static', self.tf,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def joints(self, message):
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in self.joint_names):
            return
        self.positions = [values[name] for name in self.joint_names]
        assert all(math.isfinite(value) for value in self.positions)
        self.joint_count += 1

    def clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if self.clock_last is not None and stamp < self.clock_last:
            self.clock_backwards = True
        if self.clock_first is None:
            self.clock_first = stamp
        self.clock_last = stamp
        self.clock_count += 1

    def tf(self, message):
        self.transforms.update(transform.child_frame_id for transform in message.transforms)

    def until(self, predicate, seconds, description):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f'Timeout: {description}')

    def result(self, future, seconds, description):
        self.until(future.done, seconds, description)
        value = future.result()
        if value is None:
            raise RuntimeError(f'No result: {description}')
        return value

    def execute(self, client, goal, description, seconds=30):
        handle = self.result(client.send_goal_async(goal), 10, description + ' acceptance')
        assert handle.accepted, description + ' was rejected'
        result = self.result(handle.get_result_async(), seconds, description + ' result')
        assert result.status == GoalStatus.STATUS_SUCCEEDED, str(result)
        return result.result

    def settle(self, target):
        consecutive = [0]
        last_count = [self.joint_count]
        tolerance = self.settings['tracking_tolerance_rad']

        def reached():
            if last_count[0] == self.joint_count:
                return False
            last_count[0] = self.joint_count
            if self.positions and max(abs(a - b) for a, b in zip(self.positions, target)) < tolerance:
                consecutive[0] += 1
            else:
                consecutive[0] = 0
            return consecutive[0] >= self.settings['settle_messages']

        self.until(reached, 10, f'joint tracking within {tolerance} rad')
        return [actual - desired for actual, desired in zip(self.positions, target)]

    def run(self):
        self.progress('state_discovery')
        self.until(lambda: self.positions is not None and self.clock_count > 10,
                   30, 'configured joint states and simulation clock')
        controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.until(controllers.service_is_ready, 20, 'controller manager')
        active = {}
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            response = self.result(controllers.call_async(ListControllers.Request()), 5,
                                   'controller states')
            active = {controller.name: controller.state for controller in response.controller}
            if all(active.get(name) == 'active' for name in
                   [self.profile.ros['state_controller'], self.profile.ros['trajectory_controller']]):
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        else:
            raise RuntimeError(f'Controllers did not activate: {active}')
        print(f'PASS: {len(self.joint_names)} joint states, clock and both active controllers', flush=True)

        self.progress('direct_trajectory')
        trajectory_client = ActionClient(self, FollowJointTrajectory,
            f'/{self.profile.ros["trajectory_controller"]}/follow_joint_trajectory')
        self.until(trajectory_client.server_is_ready, 20, 'trajectory action server')
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = list(self.joint_names)
        start = JointTrajectoryPoint()
        start.positions = list(self.positions)
        start.velocities = [0.0] * len(self.joint_names)
        start.time_from_start.nanosec = 100000000
        target = [float(value) for value in self.settings['direct_target_rad']]
        end = JointTrajectoryPoint()
        end.positions = target
        end.velocities = [0.0] * len(self.joint_names)
        duration_ns = round(self.settings['trajectory_duration_sec'] * 1e9)
        end.time_from_start.sec, end.time_from_start.nanosec = divmod(duration_ns, 1_000_000_000)
        trajectory_goal.trajectory.points = [start, end]
        direct_result = self.execute(trajectory_client, trajectory_goal, 'direct trajectory')
        assert direct_result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, str(direct_result)
        direct_error = self.settle(target)
        print('PASS: ros2_control trajectory -> MuJoCo movement', flush=True)

        self.progress('moveit_planning')
        planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.until(planner.service_is_ready, 30, 'MoveIt planning service')
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self.profile.ros['planning_group']
        motion.pipeline_id = 'ompl'
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = float(self.settings['velocity_scaling'])
        motion.max_acceleration_scaling_factor = float(self.settings['acceleration_scaling'])
        motion.start_state.is_diff = True
        motion.start_state.joint_state.name = list(self.joint_names)
        motion.start_state.joint_state.position = list(self.positions)
        moveit_target = self.settings['moveit_target_rad']
        goal_constraints = Constraints()
        for name, position in zip(self.joint_names, moveit_target):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = float(self.settings['goal_constraint_tolerance_rad'])
            constraint.tolerance_below = float(self.settings['goal_constraint_tolerance_rad'])
            constraint.weight = 1.0
            goal_constraints.joint_constraints.append(constraint)
        motion.goal_constraints = [goal_constraints]
        plan = self.result(planner.call_async(request), 20, 'MoveIt planning').motion_plan_response
        assert plan.error_code.val == MoveItErrorCodes.SUCCESS, str(plan.error_code)
        points = plan.trajectory.joint_trajectory.points
        assert len(points) > 1, 'MoveIt returned an empty trajectory'
        print(f'PASS: MoveIt OMPL planning ({len(points)} trajectory points)', flush=True)

        self.progress('moveit_execution')
        execution = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.until(execution.server_is_ready, 20, 'MoveIt execution action server')
        execution_goal = ExecuteTrajectory.Goal()
        execution_goal.trajectory = plan.trajectory
        executed = self.execute(execution, execution_goal, 'MoveIt execution', seconds=45)
        assert executed.error_code.val == MoveItErrorCodes.SUCCESS, str(executed)
        planned_target = dict(zip(plan.trajectory.joint_trajectory.joint_names,
                                  points[-1].positions))
        moveit_error = self.settle([planned_target[name] for name in self.joint_names])
        print('PASS: MoveIt execution -> ros2_control -> MuJoCo movement', flush=True)

        self.progress('workcell_checks')
        parameters = self.create_client(GetParameters, '/move_group/get_parameters')
        self.until(parameters.service_is_ready, 5, 'MoveIt robot description parameters')
        request = GetParameters.Request()
        request.names = ['robot_description']
        response = self.result(parameters.call_async(request), 5, 'MoveIt workcell geometry')
        actual_description = ET.fromstring(response.values[0].string_value)
        expected_description = ET.parse(self.profile.urdf_path).getroot()
        workcell_links = [link.get('name') for link in expected_description.findall('link')
                          if link.get('name').startswith('workcell_') and link.find('collision') is not None]
        for name in workcell_links:
            expected = expected_description.find(f"link[@name='{name}']/collision")
            actual = actual_description.find(f"link[@name='{name}']/collision")
            if actual is None or ET.tostring(actual).strip() != ET.tostring(expected).strip():
                raise RuntimeError(f'MoveIt workcell collision geometry mismatch: {name}')
        # Fixed geometry is part of the shared URDF, rather than a second async scene publisher.
        self.until(lambda: set(workcell_links).issubset(self.transforms), 5, 'workcell static TF')
        print(f'PASS: MoveIt workcell collision model and TF ({len(workcell_links)} objects)', flush=True)

        self.progress('tf_clock_checks')
        assert not self.clock_backwards, 'Simulation clock moved backwards'
        assert self.clock_last > self.clock_first
        missing_frames = set(self.profile.ros['tf_child_frames']) - self.transforms
        assert not missing_frames, f'Missing robot transforms: {sorted(missing_frames)}'
        return {
            'workcell_collision_links': workcell_links,
            'workcell_geometry_matches': True,
            'validated_utc': datetime.now(timezone.utc).isoformat(),
            'robot': self.profile.label, 'robot_id': self.profile.id,
            'robot_profile_sha256': self.profile.fingerprint,
            'joints': self.joint_names, 'controllers': active,
            'direct_tracking_error_rad': direct_error,
            'moveit_trajectory_points': len(points),
            'moveit_tracking_error_rad': moveit_error,
            'final_joint_positions_rad': self.positions,
            'joint_state_messages': self.joint_count,
            'clock_messages': self.clock_count,
            'clock_start_sec': self.clock_first, 'clock_end_sec': self.clock_last,
            'clock_monotonic': not self.clock_backwards,
            'clock_publisher_qos': [
                {'reliability': info.qos_profile.reliability.name,
                 'durability': info.qos_profile.durability.name}
                for info in self.get_publishers_info_by_topic('/clock')],
            'tf_child_frames': sorted(self.transforms),
        }


def main(default_robot='fr5'):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default=default_robot)
    parser.add_argument('--scene', default='default')
    parser.add_argument('--gui', action='store_true', help='also open MuJoCo and RViz')
    parser.add_argument('--domain', type=int, default=90)
    parser.add_argument('--screenshot-dir', help='capture this run\'s MuJoCo and RViz X11 windows')
    parser.add_argument('--stop-mode', choices=['interrupt', 'window'], default='interrupt',
                        help='stop via SIGINT or the MuJoCo rendering-window close path')
    args = parser.parse_args()
    if args.stop_mode == 'window' and not args.gui:
        parser.error('--stop-mode window requires --gui')
    if args.screenshot_dir and not args.gui:
        parser.error('--screenshot-dir requires --gui')
    for name, value in [('robot', args.robot), ('scene', args.scene)]:
        if not re.fullmatch(r'[a-z][a-z0-9_]*', value):
            parser.error(f'Invalid {name} id')
    return run_verification(args, ROOT, rclpy, Verification, Trigger)


if __name__ == '__main__':
    raise SystemExit(main())
