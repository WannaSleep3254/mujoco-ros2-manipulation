"""Verify the FR5 MuJoCo / ros2_control / MoveIt chain in its own ROS domain."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint


ROOT = Path(__file__).resolve().parent
JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
HOME = [0.0, -1.2, 1.2, -1.5, -1.5, 0.0]


class Verification(Node):
    def __init__(self):
        super().__init__('fr5_verification')
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

    def joints(self, message):
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in JOINTS):
            return
        self.positions = [values[name] for name in JOINTS]
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

        def reached():
            if last_count[0] == self.joint_count:
                return False
            last_count[0] = self.joint_count
            if self.positions and max(abs(a - b) for a, b in zip(self.positions, target)) < 0.01:
                consecutive[0] += 1
            else:
                consecutive[0] = 0
            return consecutive[0] >= 10

        self.until(reached, 10, 'joint tracking within 0.01 rad')
        return [actual - desired for actual, desired in zip(self.positions, target)]

    def run(self):
        self.until(lambda: self.positions is not None and self.clock_count > 10,
                   30, 'six joint states and simulation clock')
        controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.until(controllers.service_is_ready, 20, 'controller manager')
        active = {}
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            response = self.result(controllers.call_async(ListControllers.Request()), 5,
                                   'controller states')
            active = {controller.name: controller.state for controller in response.controller}
            if all(active.get(name) == 'active' for name in
                   ['joint_state_broadcaster', 'fairino5_controller']):
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        else:
            raise RuntimeError(f'Controllers did not activate: {active}')
        print('PASS: six joint states, clock and both active controllers', flush=True)

        trajectory_client = ActionClient(self, FollowJointTrajectory,
                                         '/fairino5_controller/follow_joint_trajectory')
        self.until(trajectory_client.server_is_ready, 20, 'trajectory action server')
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = JOINTS
        start = JointTrajectoryPoint()
        start.positions = list(self.positions)
        start.velocities = [0.0] * 6
        start.time_from_start.nanosec = 100000000
        target = list(HOME)
        target[0] = 0.2
        end = JointTrajectoryPoint()
        end.positions = target
        end.velocities = [0.0] * 6
        end.time_from_start.sec = 3
        trajectory_goal.trajectory.points = [start, end]
        direct_result = self.execute(trajectory_client, trajectory_goal, 'direct trajectory')
        assert direct_result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, str(direct_result)
        direct_error = self.settle(target)
        print('PASS: ros2_control trajectory -> MuJoCo movement', flush=True)

        planner = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.until(planner.service_is_ready, 30, 'MoveIt planning service')
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = 'fairino5_v6_group'
        motion.pipeline_id = 'ompl'
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.1
        motion.max_acceleration_scaling_factor = 0.1
        motion.start_state.is_diff = True
        motion.start_state.joint_state.name = JOINTS
        motion.start_state.joint_state.position = list(self.positions)
        moveit_target = [-0.2, -1.1, 1.1, -1.4, -1.5, 0.1]
        goal_constraints = Constraints()
        for name, position in zip(JOINTS, moveit_target):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.003
            constraint.tolerance_below = 0.003
            constraint.weight = 1.0
            goal_constraints.joint_constraints.append(constraint)
        motion.goal_constraints = [goal_constraints]
        plan = self.result(planner.call_async(request), 20, 'MoveIt planning').motion_plan_response
        assert plan.error_code.val == MoveItErrorCodes.SUCCESS, str(plan.error_code)
        points = plan.trajectory.joint_trajectory.points
        assert len(points) > 1, 'MoveIt returned an empty trajectory'
        print(f'PASS: MoveIt OMPL planning ({len(points)} trajectory points)', flush=True)

        execution = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.until(execution.server_is_ready, 20, 'MoveIt execution action server')
        execution_goal = ExecuteTrajectory.Goal()
        execution_goal.trajectory = plan.trajectory
        executed = self.execute(execution, execution_goal, 'MoveIt execution', seconds=45)
        assert executed.error_code.val == MoveItErrorCodes.SUCCESS, str(executed)
        planned_target = dict(zip(plan.trajectory.joint_trajectory.joint_names,
                                  points[-1].positions))
        moveit_error = self.settle([planned_target[name] for name in JOINTS])
        print('PASS: MoveIt execution -> ros2_control -> MuJoCo movement', flush=True)

        assert not self.clock_backwards, 'Simulation clock moved backwards'
        assert self.clock_last > self.clock_first
        assert len(self.transforms) >= 6, f'Missing robot transforms: {self.transforms}'
        return {
            'validated_utc': datetime.now(timezone.utc).isoformat(),
            'robot': 'FAIRINO FR5 V6', 'controllers': active,
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='also open MuJoCo and RViz')
    parser.add_argument('--domain', type=int, default=90)
    parser.add_argument('--stop-mode', choices=['interrupt', 'window'], default='interrupt',
                        help='stop via SIGINT or the MuJoCo rendering-window close path')
    args = parser.parse_args()
    if args.stop_mode == 'window' and not args.gui:
        parser.error('--stop-mode window requires --gui')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['FR5_ROS_DOMAIN_ID'] = str(args.domain)
    output = ROOT / 'runtime' / 'validation'
    output.mkdir(parents=True, exist_ok=True)
    mode = 'gui' if args.gui else 'headless'
    if args.stop_mode == 'window':
        mode += '_window'
    log_path = output / f'fr5_{mode}.log'
    report_path = output / f'fr5_{mode}.json'
    process = None
    node = None
    report = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                ['bash', str(ROOT / 'run_fr5.sh'), f'gui:={str(args.gui).lower()}',
                 f'rviz:={str(args.gui).lower()}', 'moveit:=true'],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = Verification()
            report = node.run()
            assert process.poll() is None, 'Launch process exited unexpectedly'
            launch_log = log_path.read_text()
            assert 'process has died' not in launch_log, 'A launch node failed; see the log'
            if args.gui:
                assert ('Ready to take commands for planning group fairino5_v6_group.'
                        in launch_log), 'RViz MoveIt panel is not ready'
                gpu = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=5)
                (output / 'fr5_gui_nvidia_smi.txt').write_text(gpu.stdout + gpu.stderr)
                report['mujoco_gpu_process_seen'] = 'ros2_control_node' in gpu.stdout
                report['rviz_gpu_process_seen'] = '/rviz2' in gpu.stdout
            report['gui'] = args.gui
            report['ros_domain_id'] = args.domain
            report['motion_checks_passed'] = True
            report['stop_mode'] = args.stop_mode
            if args.stop_mode == 'window':
                closer = node.create_client(Trigger, '/mujoco_ros2_control_node/close_window')
                node.until(closer.service_is_ready, 5, 'window close service')
                closed = node.result(closer.call_async(Trigger.Request()), 5, 'window close request')
                assert closed.success, closed.message
                process.wait(timeout=15)
                print('PASS: window close also stopped the FR5 launch', flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None and process.poll() is None:
            # Let ros2 launch signal each child once; signaling the entire group
            # here would deliver a second SIGINT while RViz is cleaning up.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    if report is not None:
        shutdown_log = log_path.read_text()
        report['shutdown_failures'] = [
            {'node': node_name, 'exit_code': int(code)}
            for node_name, code in re.findall(
                r'\[ERROR\] \[([^\]]+)\]: process has died \[pid \d+, exit code (-?\d+)',
                shutdown_log) if int(code) != 0]
        report['shutdown_timed_out'] = 'failed to terminate' in shutdown_log
        report['launch_return_code'] = process.returncode
        try:
            os.killpg(process.pid, 0)
            report['process_group_gone'] = False
        except ProcessLookupError:
            report['process_group_gone'] = True
        report['shutdown_clean'] = not (report['shutdown_failures'] or
                                       report['shutdown_timed_out'] or
                                       process.returncode != 0 or
                                       not report['process_group_gone'])
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        print(f'Launch log: {log_path}', flush=True)
        if not report['shutdown_clean']:
            print('WARN: motion checks passed, but shutdown failed; see shutdown_failures.',
                  flush=True)
        return 0 if report['shutdown_clean'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
