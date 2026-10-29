"""Verify the isolated native ROS grasp foundation, not block grasping.

A fresh checked FR5/UR5e target supplies fourteen scalar JointState positions,
three active controllers and native MuJoCo 3.12 observations. The empty gripper
closes to 0.01 m and opens, then cancellation or a deliberately missing phase
heartbeat must stop all three controllers. No GUI or HTTP server is started.
Every owned run retains its result, log and native diagnostics, including failure.
"""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import uuid

from mujoco_ros2_manipulation.grasping.ros_model import FINGER_JOINTS
from mujoco_ros2_manipulation.grasping.ros_runtime import check_grasp_runtime
from mujoco_ros2_manipulation.grasping.ros_target import load_ros_grasp_target
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.process_faults import process_failures
from mujoco_ros2_manipulation.verification.reports import atomic_json, utc_now
from mujoco_ros2_manipulation.verification.runner import group_gone, stop_process
from mujoco_ros2_manipulation.workcells.config import load_workcell


CONTROLLERS = ('left_controller', 'right_controller', 'gripper_controller')


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def finite_vector(value, count):
    return (type(value) is list and len(value) == count
            and all(type(item) in (int, float) and math.isfinite(item) for item in value))


def checked_observation(value, identity, target):
    """Check fixed run/source identity and scalar units before using a sample."""
    require(type(value) is dict and value.get('schema_version') == 1
            and value.get('kind') == 'ros-grasp-observation-v1', 'Unexpected native observation schema')
    for field in ('run_id', 'model_sha256', 'plugin_sha256'):
        require(value.get(field) == identity[field], 'Native observation identity differs: ' + field)
    require(value.get('native_version') == '3.12.0', 'Expected native MuJoCo 3.12.0')
    require(type(value.get('sequence')) is int and value['sequence'] > 0, 'Missing native sequence')
    require(type(value.get('time_sec')) in (int, float) and math.isfinite(value['time_sec'])
            and value['time_sec'] >= 0, 'Missing finite native time')
    require(type(value.get('stamp_ns')) is str and re.fullmatch(r'0|[1-9][0-9]*', value['stamp_ns'])
            and value.get('stamp_semantics') == 'rounded_native_mjData_time',
            'Native time must retain its rounded-time semantics')
    require(abs(int(value['stamp_ns']) - value['time_sec'] * 1e9) <= 1., 'Rounded native stamp differs')
    require(value.get('force_timing_verified') is False, 'This test does not verify contact-force timing')
    require(finite_vector(value.get('qpos'), 21) and finite_vector(value.get('qvel'), 20),
            'Expected full native 21-position / 20-velocity state')
    for field, names in (('arm_positions_rad', target.joints), ('finger_positions_m', FINGER_JOINTS)):
        positions = value.get(field)
        require(type(positions) is dict and set(positions) == set(names)
                and all(type(item) in (int, float) and math.isfinite(item) for item in positions.values()),
                'Missing complete native scalar positions: ' + field)
    layout = target.manifest['qpos_state_contract']['layout']
    for row in layout:
        if row['qpos_width'] != 1:
            continue
        field = 'finger_positions_m' if row['name'] in FINGER_JOINTS else 'arm_positions_rad'
        require(value[field][row['name']] == value['qpos'][row['qpos_address']],
                'Native named positions disagree with the full-state layout')
    return value


def compact_observation(value):
    """Keep proof identifiers, clocks and scalar positions, not 49 geom frames."""
    fields = ('run_id', 'model_sha256', 'plugin_sha256', 'native_version', 'sequence',
              'time_sec', 'stamp_ns', 'stamp_semantics', 'phase', 'armed', 'heartbeat_fresh',
              'force_timing_verified', 'arm_positions_rad', 'finger_positions_m')
    result = {field: deepcopy(value[field]) for field in fields}
    result['full_state_dimensions'] = {'qpos': len(value['qpos']), 'qvel': len(value['qvel'])}
    return result


def checked_joint_state(message, target):
    require(len(message.name) == 14 and len(set(message.name)) == 14
            and set(message.name) == set(target.allowed_scalar_joints),
            'JointState must contain exactly the fourteen declared arm/finger joints')
    require(len(message.position) == 14 and all(math.isfinite(value) for value in message.position),
            'JointState positions must be finite')
    return dict(stamp_sec=message.header.stamp.sec, stamp_nanosec=message.header.stamp.nanosec,
                positions={name: position for name, position in zip(message.name, message.position)})


def native_exit_proof(log):
    started = re.findall(r'\[INFO\] \[(ros2_control_node-\d+)\]: process started with pid \[(\d+)\]', log)
    if len(started) != 1:
        return dict(observed=False, reason='Native child start is missing or ambiguous')
    name, pid = started[0]
    marker = f'[{name}]: process has finished cleanly [pid {pid}]'
    return dict(observed=marker in log, node=name, pid=int(pid), clean_exit_marker=marker)


def verify(args):
    output = args.output.resolve()
    require(not output.exists(), 'Select a new output directory; existing evidence is preserved')
    require(output.is_relative_to((ROOT / 'runtime').resolve()), 'Select an output directory under this worktree runtime/')
    output.mkdir(parents=True, mode=0o700)
    log_path = output / 'launch.log'
    log_path.touch()
    receipt = output / 'result.json'
    record = dict(schema_version=1, status='running', stage='preflight', started_utc=utc_now(),
                  run_id=uuid.uuid4().hex, domain=args.domain, stop_mode=args.stop_mode,
                  scope=__doc__, automatic_retry=False, physical_grasp_verified=False,
                  moveit_attachment_verified=False, checks=[], cleanup_errors=[])
    process = node = client = heartbeat_timer = log_stream = ros = None
    initialized = False
    target = None
    source_hashes = {}

    def save():
        atomic_json(receipt, record)

    def stage(name):
        record['stage'] = name
        save()

    save()
    try:
        # Model and runtime checks precede even the unused-domain ROS probe.
        base = load_workcell('fr5_ur5e')
        target = load_ros_grasp_target(base, args.execution_environment, args.execution_manifest_sha256)
        require(target is not None, 'Select an explicit SHA-paired ROS grasp execution target')
        check_grasp_runtime(ROOT)
        source_files = (
            Path(__file__).resolve(), ROOT / 'native/step_monitor.hpp', ROOT / 'native/grasp_policy.hpp',
            ROOT / 'native/control_loop_shutdown.hpp', ROOT / 'native/grasp_ros2_plugin/src/observation.cpp',
            ROOT / 'native/grasp_ros2_plugin/plugin.xml',
        )
        source_hashes = {str(path): file_sha256(path) for path in source_files}
        identity = dict(run_id=record['run_id'], model_sha256=file_sha256(target.model_path),
                        plugin_sha256=file_sha256(ROOT / 'native/grasp_ros2_plugin/src/observation.cpp'))
        record['identity'] = dict(identity, execution_manifest_sha256=args.execution_manifest_sha256,
            target_fingerprint=target.fingerprint,
            native_library_sha256=file_sha256(ROOT / 'ros_ws/install/mujoco_ros2_control/lib/libmujoco_ros2_control.so'),
            native_executable_sha256=file_sha256(ROOT / 'ros_ws/install/mujoco_ros2_control/lib/mujoco_ros2_control/ros2_control_node'))
        record['source_sha256'] = source_hashes
        record['scalar_joint_units'] = target.description()['scalar_joint_units']
        save()

        import rclpy
        from action_msgs.msg import GoalStatus
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from trajectory_msgs.msg import JointTrajectoryPoint
        from mujoco_ros2_manipulation.workcells.verify_motion_reproduction import require_unused_domain

        ros = rclpy
        stage('unused_domain')
        require_unused_domain(args.domain)
        os.environ['MANIPULATION_ROS_DOMAIN_ID'] = str(args.domain)
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
        ros.init(domain_id=args.domain)
        initialized = True
        node = Node('ros_grasp_foundation_' + record['run_id'][:8])
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        guard_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latest = dict(observation=None, observation_receipt_sec=None, joints=None, guard=None, error=None)
        phase = [0]
        last_heartbeat = [None]

        def observation_callback(message):
            try:
                value = checked_observation(json.loads(message.data), identity, target)
                previous = latest['observation']
                if previous is not None:
                    require(value['sequence'] > previous['sequence'], 'Native observation sequence did not advance')
                    require(value['time_sec'] >= previous['time_sec'], 'Native observation clock reversed')
                latest['observation'] = value
                latest['observation_receipt_sec'] = time.monotonic()
            except Exception as error:
                latest['error'] = str(error)

        def joint_callback(message):
            try:
                latest['joints'] = checked_joint_state(message, target)
            except Exception as error:
                latest['error'] = str(error)

        def guard_callback(message):
            try:
                value = json.loads(message.data)
                require(type(value.get('latched')) is bool and type(value.get('controllers')) is dict,
                        'Malformed contact guard observation')
                latest['guard'] = value
            except Exception as error:
                latest['error'] = str(error)

        def publish_phase():
            publisher.publish(String(data=record['run_id'] + ':' + str(phase[0])))
            last_heartbeat[0] = time.monotonic()

        node.create_subscription(String, '/workcell/grasp_observation', observation_callback, qos)
        node.create_subscription(JointState, '/joint_states', joint_callback, qos)
        node.create_subscription(String, '/workcell/contact_stop', guard_callback, guard_qos)
        publisher = node.create_publisher(String, '/workcell/grasp_phase', qos)
        heartbeat_timer = node.create_timer(.1, publish_phase)
        client = ActionClient(node, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        command = ['bash', str(ROOT / 'run_scene.sh'), 'workcell:=fr5_ur5e', 'gui:=false', 'rviz:=false',
                   'execution_environment:=' + str(target.output),
                   'execution_manifest_sha256:=' + args.execution_manifest_sha256,
                   'grasp_run_id:=' + record['run_id']]
        environment = dict(os.environ, MANIPULATION_ROS_DOMAIN_ID=str(args.domain), ROS_DOMAIN_ID=str(args.domain),
                           MANIPULATION_DIAGNOSTICS_PATH=str(output / 'physics.json'),
                           MANIPULATION_DIAGNOSTICS_RUN_ID=record['run_id'])
        stage('launch')
        log_stream = log_path.open('wb')
        process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log_stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        record['owned_process_group'] = process.pid
        save()

        def wait(predicate, label, timeout=35.):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                require(process.poll() is None, 'Owned launch exited during ' + label)
                ros.spin_once(node, timeout_sec=.03)
                require(latest['error'] is None, 'Invalid ROS input: ' + str(latest['error']))
                value = predicate()
                if value:
                    return value
            raise TimeoutError(label)

        stage('native_observation')
        first = wait(lambda: latest['observation'], 'native observation', 60.)
        advancing = wait(lambda: latest['observation'] if latest['observation']['time_sec'] > first['time_sec']
                         and latest['observation']['sequence'] > first['sequence'] else None, 'advancing native clock')
        joints = wait(lambda: latest['joints'], 'fourteen scalar joint states')
        guard = wait(lambda: latest['guard'] if latest['guard'] and latest['guard'].get('phase') == 'ready'
                     and not latest['guard']['latched']
                     and all(latest['guard']['controllers'].get(name) == 'active' for name in CONTROLLERS)
                     else None, 'three active controllers', 60.)
        record['checks'].append(dict(name='native_identity_clock_and_three_controllers',
            initial_observation=compact_observation(first), advancing_observation=compact_observation(advancing),
            joint_state=joints, guard=deepcopy(guard)))
        save()
        wait(client.server_is_ready, 'gripper FollowJointTrajectory server')

        for phase_value, position, name in ((2, .01, 'empty_gripper_close'), (6, 0., 'empty_gripper_open')):
            stage(name)
            phase[0] = phase_value
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(FINGER_JOINTS)
            goal.trajectory.points = [JointTrajectoryPoint(positions=[position, position],
                                                           time_from_start=Duration(sec=1))]
            before = time.monotonic()
            accepted = client.send_goal_async(goal)
            handle = wait(lambda: accepted.result() if accepted.done() else None, 'gripper goal acceptance')
            require(handle.accepted, 'Gripper goal was rejected')
            future = handle.get_result_async()
            terminal = wait(lambda: future.result() if future.done() else None, 'gripper goal terminal', 8.)
            terminal_receipt = time.monotonic()
            require(terminal.status == GoalStatus.STATUS_SUCCEEDED and terminal.result.error_code == 0,
                    'Gripper action did not succeed')
            observation = wait(lambda: latest['observation']
                if latest['observation_receipt_sec'] >= terminal_receipt
                and max(abs(value - position) for value in latest['observation']['finger_positions_m'].values()) < .002
                else None, 'post-terminal native finger positions', 5.)
            record['checks'].append(dict(name=name, target_m=position, physical_grasp_claim=False,
                action_status=terminal.status, action_error_code=terminal.result.error_code,
                action_goal_id=bytes(handle.goal_id.uuid).hex(),
                sent_monotonic_sec=before, terminal_received_monotonic_sec=terminal_receipt,
                post_terminal_observation=compact_observation(observation)))
            save()

        stage('stop_' + args.stop_mode)
        stop_requested = time.monotonic()
        if args.stop_mode == 'cancel':
            phase[0] = 8
            publish_phase()
            expected_reason = 'grasp_cancel'
        else:
            heartbeat_timer.cancel()
            expected_reason = 'grasp_heartbeat'
        stopped = wait(lambda: latest['guard'] if latest['guard']
                       and latest['guard'].get('phase') == 'stopped' and latest['guard']['latched']
                       and all(latest['guard']['controllers'].get(name) == 'inactive' for name in CONTROLLERS)
                       else None, 'native stop and three inactive controllers')
        require(stopped.get('reason') == expected_reason, 'Unexpected native stop reason')
        record['checks'].append(dict(name='three_controller_stop', requested_stop_mode=args.stop_mode,
            requested_monotonic_sec=stop_requested, observed_monotonic_sec=time.monotonic(),
            last_heartbeat_publication_monotonic_sec=last_heartbeat[0], guard=deepcopy(stopped),
            physical_grasp_claim=False))
        record['status'] = 'passed'
    except BaseException as error:
        record.update(status='failed', error=dict(type=type(error).__name__, message=str(error)),
                      failure_stage=record['stage'])
    finally:
        record['stage'] = 'cleanup'
        save()
        if heartbeat_timer is not None:
            try:
                heartbeat_timer.cancel()
            except Exception as error:
                record['cleanup_errors'].append('heartbeat cancel: ' + str(error))
        forced = False
        if process is not None:
            try:
                forced = stop_process(process)
            except BaseException as error:
                record['cleanup_errors'].append('owned launch stop: ' + str(error))
        if log_stream is not None:
            log_stream.close()
        if client is not None:
            try:
                client.destroy()
            except Exception as error:
                record['cleanup_errors'].append('action client destroy: ' + str(error))
        if node is not None:
            try:
                node.destroy_node()
            except Exception as error:
                record['cleanup_errors'].append('node destroy: ' + str(error))
        if initialized and ros is not None and ros.ok():
            try:
                ros.shutdown()
            except Exception as error:
                record['cleanup_errors'].append('ROS shutdown: ' + str(error))
        log = log_path.read_text(errors='replace')
        native = native_exit_proof(log)
        failures = process_failures(log)
        gone = process is None
        if process is not None:
            try:
                gone = group_gone(process.pid)
            except OSError as error:
                record['cleanup_errors'].append('process group check: ' + str(error))
        clean = (process is not None and process.returncode == 0 and native['observed'] and gone
                 and not forced and not failures and not record['cleanup_errors']
                 and 'process has died' not in log and 'failed to terminate' not in log)
        record['cleanup'] = dict(launch_started=process is not None,
            launcher_return_code=process.returncode if process is not None else None,
            forced=forced, process_group_gone=gone, native_exit=native,
            child_process_failures=failures, clean=clean)
        if process is not None and not clean:
            record['status'] = 'failed'
            record.setdefault('error', dict(type='NativeShutdownFailure',
                                            message='A clean native child and launcher exit was not observed'))
        try:
            if target is not None:
                target.validate()
            require(all(file_sha256(Path(path)) == digest for path, digest in source_hashes.items()),
                    'Verification sources changed during the run')
        except Exception as error:
            record['status'] = 'failed'
            record['final_identity_error'] = dict(type=type(error).__name__, message=str(error))
        record.update(stage='complete', completed_utc=utc_now(), launch_log_sha256=file_sha256(log_path))
        if (output / 'physics.json').is_file():
            record['physics_sha256'] = file_sha256(output / 'physics.json')
        save()
    print(record['status'].upper() + ': ' + str(receipt), flush=True)
    return 0 if record['status'] == 'passed' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, required=True, help='An unused ROS domain, 0..232')
    parser.add_argument('--output', type=Path, required=True, help='A new evidence directory under this worktree runtime/')
    parser.add_argument('--execution-environment', type=Path, required=True)
    parser.add_argument('--execution-manifest-sha256', required=True)
    parser.add_argument('--stop-mode', choices=('cancel', 'heartbeat'), default='cancel')
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232 or re.fullmatch(r'[0-9a-f]{64}', args.execution_manifest_sha256) is None:
        parser.error('Expected unused domain 0..232 and an explicit lowercase manifest SHA256')
    try:
        return verify(args)
    except Exception as error:
        # Failure before a fresh output directory exists must not overwrite an
        # earlier run; failures after its creation are persisted by verify().
        print(type(error).__name__ + ': ' + str(error), flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
