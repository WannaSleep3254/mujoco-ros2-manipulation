"""Publish a bounded diagnostic RGB8 pattern, using an observed simulation clock.

This is a transport fixture, not a camera model, calibration or sensor accuracy
measurement. It never publishes a clock, joint state or control command.
"""
import argparse
import json
import math
import time


def pattern(sequence, width=64, height=48):
    if type(sequence) is not int or sequence < 0 or (width, height) != (64, 48):
        raise ValueError("Expected bounded diagnostic pattern")
    return bytes(channel for y in range(height) for x in range(width)
                 for channel in ((x*4+sequence)%256, y*5%256, sequence%256))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--topic", default="/diagnostic/image")
    parser.add_argument("--duration", type=float, default=10)
    parser.add_argument("--rate", type=float, default=10)
    parser.add_argument("--gap-at", type=float, default=4)
    parser.add_argument("--gap-duration", type=float, default=1)
    args = parser.parse_args()
    if (not 0 <= args.domain <= 232 or not all(math.isfinite(v) for v in
            (args.duration,args.rate,args.gap_at,args.gap_duration))
            or not 0 < args.duration <= 120 or not 0 < args.rate <= 30
            or not 0 <= args.gap_at <= 120 or not 0 <= args.gap_duration <= 30):
        parser.error("Expected bounded domain, duration, rate and gap")
    import os
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    from .image_source import ImageSource
    ImageSource(topic=args.topic)  # Validate before ROS entities.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node = Node("diagnostic_rgb8_producer")
    qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(Image,args.topic,qos)
    stamp = [None]
    def tick(message):
        stamp[0] = message.clock
    node.create_subscription(Clock,"/clock",tick,qos)
    started,next_frame,count,last_stamp = time.monotonic(),0.,0,None
    try:
        while rclpy.ok() and time.monotonic()-started < args.duration:
            rclpy.spin_once(node,timeout_sec=.01)
            elapsed = time.monotonic()-started
            if stamp[0] is None or elapsed < next_frame or args.gap_at <= elapsed < args.gap_at+args.gap_duration:
                continue
            key = (stamp[0].sec,stamp[0].nanosec)
            if key == last_stamp:
                continue
            image = Image()
            image.header.stamp,image.header.frame_id = stamp[0],"diagnostic_optical"
            image.width,image.height,image.step,image.encoding,image.is_bigendian = 64,48,192,"rgb8",0
            image.data = pattern(count)
            publisher.publish(image)
            count,last_stamp,next_frame = count+1,key,elapsed+1/args.rate
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(dict(kind="diagnostic-pattern-only",published=count,
        clock="observed ROS /clock acquisition stamp",control_commands=0)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
