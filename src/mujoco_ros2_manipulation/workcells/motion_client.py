"""Send one correlated simulation joint goal, or cancel a previously submitted ID."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.motion_request import (
    COMMAND_TOPIC, CANCEL_TOPIC, STATUS_TOPIC, STRATEGIES, command, decode_message, request_id,
)
from mujoco_ros2_manipulation.workcells.motion_server import status_qos


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--target", type=Path, help="JSON object: complete joint-name to radians mapping")
    source.add_argument("--home", action="store_true")
    source.add_argument("--cancel", help="Cancel this existing 32-character request ID")
    parser.add_argument("--workcell", default="fr5_ur5e")
    parser.add_argument("--strategy", choices=STRATEGIES, default="coordinated")
    parser.add_argument("--request-id")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--output", type=Path, help="Save received terminal status (a new file only)")
    args = parser.parse_args()
    if not 1 <= args.timeout <= 600:
        parser.error("timeout must be 1..600 seconds")
    if args.output and args.output.exists():
        parser.error("--output must be a new file")
    if args.cancel:
        payload, topic = dict(request_id=request_id(args.cancel)), CANCEL_TOPIC
    else:
        cell = load_workcell(args.workcell)
        target = dict(zip(cell.joints, cell.home)) if args.home else decode_message(args.target.read_text())
        payload, topic = command(target, args.strategy, args.request_id), COMMAND_TOPIC
        request_id(payload["request_id"])
    rclpy.init()
    node = Node("workcell_motion_client")
    terminal = []
    def status(message):
        record = json.loads(message.data)
        if record.get("request_id") == payload["request_id"] and record.get("status") in ("succeeded", "failed", "cancelled", "rejected"):
            terminal.append(record)
    node.create_subscription(String, STATUS_TOPIC, status, status_qos())
    publisher = node.create_publisher(String, topic, 10)
    began = time.monotonic()
    published = None
    try:
        while rclpy.ok() and time.monotonic() - began < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if published is None and publisher.get_subscription_count() >= 1 and node.count_publishers(STATUS_TOPIC) == 1:
                published = time.monotonic()
                publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
                print("SUBMITTED: " + payload["request_id"], flush=True)
            if terminal:
                result = dict(terminal[-1], client_publish_to_result_wall_sec=None if published is None else time.monotonic() - published)
                text = json.dumps(result, indent=2, allow_nan=False) + "\n"
                print(text, end="")
                if args.output:
                    with args.output.open("x") as stream:
                        stream.write(text)
                return 0 if result["status"] in ("succeeded", "cancelled") else 1
        raise RuntimeError("Timed out waiting for terminal status; the client timeout does not cancel the robot. Use --cancel with the submitted ID.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
