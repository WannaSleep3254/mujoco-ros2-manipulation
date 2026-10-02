"""MuJoCo / ROS 2 integration test, headless or GUI (separate processes)."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

PROJECT_DIR = Path(__file__).resolve().parent
XML = (PROJECT_DIR / 'models' / 'single_joint.xml').read_text()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true',
                        help='Also verify the bridge with its GUI open.')
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_string(XML)
    data = mujoco.MjData(model)
    data.ctrl[0] = 0.5
    start = time.perf_counter()
    for _ in range(10000):
        mujoco.mj_step(model, data)
    elapsed = time.perf_counter() - start
    assert abs(data.qpos[0] - 0.5) < 0.01
    print(json.dumps({'mujoco': mujoco.__version__, 'physics_steps': 10000,
                      'elapsed_s': elapsed, 'simulated_s': data.time}), flush=True)
    rclpy.init()
    node = Node('mujoco_validation_client')
    received = []
    clocks = []
    reliable_clocks = []
    sub = node.create_subscription(JointState, '/joint_states',
                                   lambda msg: received.append(msg.position[0]), 10)
    clock_sub = node.create_subscription(
        Clock, '/clock',
        lambda msg: clocks.append(msg.clock.sec * 1_000_000_000 + msg.clock.nanosec),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    reliable_clock_sub = node.create_subscription(
        Clock, '/clock',
        lambda msg: reliable_clocks.append(
            msg.clock.sec * 1_000_000_000 + msg.clock.nanosec),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    pub = node.create_publisher(Float64, '/mujoco/target_position', 10)
    process = None
    results = []
    try:
        command = [
            sys.executable, str(PROJECT_DIR / 'mujoco_ros2_bridge.py'),
            '--duration', '25']
        if not args.gui:
            command.append('--headless')
        process = subprocess.Popen(command)
        deadline = time.monotonic() + 8
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            assert process.poll() is None, 'Simulator exited during startup'
            rclpy.spin_once(node, timeout_sec=0.05)
        assert pub.get_subscription_count() > 0, 'ROS command subscriber discovery timed out'
        for target in [0.5, -0.3]:
            received.clear()
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                pub.publish(Float64(data=target))
                rclpy.spin_once(node, timeout_sec=0.05)
                if len(received) >= 10 and all(abs(q-target) < 0.01 for q in received[-10:]):
                    break
            assert len(received) >= 10 and all(abs(q-target) < 0.01 for q in received[-10:]), 'ROS command tracking failed'
            results.append({'target_rad': target, 'actual_rad': received[-1],
                            'messages_received': len(received)})
        print(json.dumps({'ros2_cross_process': 'PASS',
                          'mode': 'gui' if args.gui else 'headless',
                          'tracking': results}), flush=True)
        for reliability, samples in [('best_effort', clocks), ('reliable', reliable_clocks)]:
            assert len(samples) >= 10 and all(
                b > a for a, b in zip(samples, samples[1:])
            ), f'Simulation clock failed for {reliability} subscribers'
            print(json.dumps({'simulation_clock': 'PASS', 'subscriber_qos': reliability,
                              'messages_received': len(samples),
                              'last_simulated_s': samples[-1] / 1_000_000_000}), flush=True)
    finally:
        if process is not None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
