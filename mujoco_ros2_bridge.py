"""ROS 2 position-command bridge for the single-joint MuJoCo demo."""

import argparse
from contextlib import nullcontext
import math
from pathlib import Path
import time

from builtin_interfaces.msg import Time
import mujoco
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


MODEL_PATH = Path(__file__).resolve().parent / 'models' / 'single_joint.xml'
COMMAND_TOPIC = '/mujoco/target_position'
STATE_TOPIC = '/joint_states'
# Reliable publishing supports both reliable rqt subscribers and best-effort clocks.
CLOCK_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


def simulation_stamp(seconds):
    sec, nanosec = divmod(round(seconds * 1_000_000_000), 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


class MuJoCoBridge(Node):
    def __init__(self, model_path):
        super().__init__('mujoco_ros2_bridge')
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        if (self.model.nq, self.model.nv, self.model.nu) != (1, 1, 1):
            raise ValueError('This bridge supports only the supplied single-joint model.')
        self.joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, 'joint1')
        self.actuator_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'joint1_servo')
        if self.joint_id < 0 or self.actuator_id < 0:
            raise ValueError('The model needs joint1 and its joint1_servo position actuator.')
        self.target = float(self.data.qpos[0])
        self.data.ctrl[self.actuator_id] = self.target
        mujoco.mj_forward(self.model, self.data)
        self.command_sub = self.create_subscription(
            Float64, COMMAND_TOPIC, self.receive_target, 10)
        self.state_pub = self.create_publisher(JointState, STATE_TOPIC, 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', CLOCK_QOS)
        self.get_logger().info(
            f'Model: {model_path}; position commands (rad): {COMMAND_TOPIC}; '
            f'states: {STATE_TOPIC}; simulation time: /clock')

    def receive_target(self, msg):
        if not math.isfinite(msg.data):
            self.get_logger().warning('Ignoring a non-finite target angle.')
            return
        if self.model.actuator_ctrllimited[self.actuator_id]:
            lower, upper = self.model.actuator_ctrlrange[self.actuator_id]
            if not lower <= msg.data <= upper:
                self.get_logger().warning('Ignoring a target outside actuator ctrlrange.')
                return
        self.target = msg.data

    def publish_state(self):
        stamp = simulation_stamp(self.data.time)
        self.clock_pub.publish(Clock(clock=stamp))
        state = JointState()
        state.header.stamp = stamp
        state.name = ['joint1']
        state.position = [float(self.data.qpos[0])]
        state.velocity = [float(self.data.qvel[0])]
        state.effort = [float(self.data.qfrc_actuator[0])]
        self.state_pub.publish(state)


def run_simulation(node, viewer, duration, state_rate):
    timestep = float(node.model.opt.timestep)
    publish_stride = max(1, round(1 / (state_rate * timestep)))
    step = 0
    start = time.monotonic()
    next_step = start
    next_render = start
    node.publish_state()
    while rclpy.ok():
        if duration and time.monotonic() - start >= duration:
            break
        if viewer is not None and not viewer.is_running():
            break
        rclpy.spin_once(node, timeout_sec=0)
        with viewer.lock() if viewer is not None else nullcontext():
            # ROS owns the control input; a GUI slider cannot overwrite the target.
            node.data.ctrl[node.actuator_id] = node.target
            mujoco.mj_step(node.model, node.data)
        step += 1
        if step % publish_stride == 0:
            node.publish_state()
        now = time.monotonic()
        if viewer is not None and now >= next_render:
            viewer.sync()
            next_render = now + 1 / 60
        next_step += timestep
        delay = next_step - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        elif delay < -0.1:
            # Do not spin in a large catch-up loop after a delayed frame.
            next_step = time.monotonic()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=MODEL_PATH)
    parser.add_argument('--headless', action='store_true', help='Run without a GUI.')
    parser.add_argument('--duration', type=float, default=0,
                        help='Stop after this many wall seconds; 0 runs until closed.')
    parser.add_argument('--state-rate', type=float, default=50,
                        help='Joint state and clock publication rate (Hz).')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not math.isfinite(args.state_rate) or args.state_rate <= 0:
        parser.error('--state-rate must be finite and positive')
    rclpy.init(args=ros_args)
    node = None
    try:
        node = MuJoCoBridge(args.model)
        if args.headless:
            run_simulation(node, None, args.duration, args.state_rate)
        else:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
                run_simulation(node, viewer, args.duration, args.state_rate)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
