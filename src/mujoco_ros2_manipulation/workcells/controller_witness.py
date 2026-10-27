"""Bounded, passive controller observations; no control or physics operations.

The pure recorder has no ROS dependency. Source stamps and receipt times belong
to different clocks. Their association does not measure controller latency or
identify the exact native simulation step.
"""
from collections import Counter, deque
from copy import deepcopy
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time
import uuid


MAX_CAPACITY = 20_000
MAX_CONTROLLERS = 8
MAX_JOINTS = 32
MAX_STATUSES = 64
POINT_FIELDS = ("reference", "feedback", "output", "error", "desired", "actual")
VECTOR_FIELDS = ("positions", "velocities", "accelerations", "effort")
MAX_SECONDS = 1_000_000_000


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _stamp(value, *, signed=False):
    _require(type(value) is dict and set(value) == {"sec", "nanosec"}, "invalid_stamp")
    sec, nsec = value["sec"], value["nanosec"]
    _require(type(sec) is int and (-MAX_SECONDS if signed else 0) <= sec <= MAX_SECONDS
             and type(nsec) is int and 0 <= nsec < 1_000_000_000, "invalid_stamp")
    return sec * 1_000_000_000 + nsec


def _names(value):
    _require(type(value) in (list, tuple) and 1 <= len(value) <= MAX_JOINTS, "invalid_joints")
    _require(all(isinstance(name, str) and 0 < len(name) <= 128
                 and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_/]*", name) for name in value)
             and len(set(value)) == len(value), "invalid_joints")
    return list(value)


def _point(value, count):
    _require(type(value) is dict and set(value) == set(VECTOR_FIELDS) | {"time_from_start"},
             "invalid_point")
    record = {"time_from_start_ns": _stamp(value["time_from_start"], signed=True)}
    for field in VECTOR_FIELDS:
        values = value[field]
        _require(type(values) in (list, tuple) and len(values) in (0, count), "invalid_dimensions")
        _require(all(type(number) in (int, float) and math.isfinite(number)
                     for number in values), "nonfinite_value")
        record[field] = list(values)
    return record


def _controller(value):
    _require(type(value) is dict and {"header", "joint_names"} <= set(value)
             and set(value) <= {"header", "joint_names", *POINT_FIELDS}, "invalid_controller")
    header = value["header"]
    _require(type(header) is dict and set(header) == {"stamp", "frame_id"}
             and isinstance(header["frame_id"], str) and len(header["frame_id"]) <= 256
             and not any(ord(char) < 32 for char in header["frame_id"]), "invalid_header")
    names = _names(value["joint_names"])
    points = {field: _point(value[field], len(names)) for field in POINT_FIELDS if field in value}
    _require(any(all(field in points and len(points[field]["positions"]) == len(names)
                     for field in pair) for pair in (("reference", "feedback"), ("desired", "actual"))),
             "missing_state_positions")
    return dict(source_stamp_ns=_stamp(header["stamp"]), frame_id=header["frame_id"],
                joint_names=names, points=points)


def _statuses(value):
    _require(type(value) is dict and set(value) == {"status_list"}
             and type(value["status_list"]) in (list, tuple)
             and len(value["status_list"]) <= MAX_STATUSES, "invalid_statuses")
    statuses, seen = [], set()
    for status in value["status_list"]:
        _require(type(status) is dict and set(status) == {"goal_uuid", "goal_stamp", "status"},
                 "invalid_statuses")
        goal = status["goal_uuid"]
        _require(isinstance(goal, str) and re.fullmatch("[0-9a-f]{32}", goal)
                 and goal not in seen and type(status["status"]) is int
                 and 0 <= status["status"] <= 6, "invalid_statuses")
        seen.add(goal)
        statuses.append(dict(goal_uuid=goal, goal_stamp_ns=_stamp(status["goal_stamp"]),
                             status=status["status"]))
    return dict(status_list=statuses)


class ControllerWitness:
    """A fixed-capacity event ring. Malformed observations are counted and ignored.

    A backwards /clock sample starts a new observed epoch. Backwards controller
    stamps without that observed reset are rejected; old samples stay in the ring
    with their original epoch. No restart is inferred from a goal UUID or stamp.
    """
    def __init__(self, *, capacity=10_000, controllers=("left_controller", "right_controller"),
                 clock=time.monotonic_ns, session_id=None, domain=None):
        _require(type(capacity) is int and 1 <= capacity <= MAX_CAPACITY, "invalid_capacity")
        _require(type(controllers) in (tuple, list) and 1 <= len(controllers) <= MAX_CONTROLLERS
                 and all(isinstance(name, str) and len(name) <= 80
                         and re.fullmatch("[a-z][a-z0-9_]*", name) for name in controllers)
                 and len(set(controllers)) == len(controllers), "invalid_controllers")
        session_id = uuid.uuid4().hex if session_id is None else session_id
        _require(isinstance(session_id, str) and re.fullmatch("[0-9a-f]{32}", session_id), "invalid_session")
        _require(domain is None or type(domain) is int and 0 <= domain <= 232, "invalid_domain")
        self.session_id, self.capacity, self.clock = session_id, capacity, clock
        self.domain = domain
        self.controllers = tuple(controllers)
        self.controller_topics = {f"/{name}/controller_state" for name in controllers}
        self.status_topics = {f"/{name}/follow_joint_trajectory/_action/status" for name in controllers}
        self.topic_counts = {topic: 0 for topic in sorted(self.controller_topics | self.status_topics | {"/clock"})}
        self.events = deque(maxlen=capacity)
        self.counts, self.invalid_reasons = Counter(), Counter()
        self.first_received_ns = self.last_received_ns = None
        self.clock_ns, self.clock_epoch, self.clock_resets = None, 0, 0
        self.controller_stamps, self.joint_names = {}, {}
        self.finished = None

    def reject(self, reason="invalid_ros_message"):
        # Public converter failures use this fixed label, never an untrusted
        # exception message, preventing an unbounded error-key collection.
        self.counts["received"] += 1
        self.counts["invalid"] += 1
        self.invalid_reasons["invalid_ros_message"] += 1
        return False

    def observe(self, kind, topic, value, *, received_ns=None):
        self.counts["received"] += 1
        try:
            _require(self.finished is None, "already_finished")
            _require(kind in ("controller", "goal_status", "clock"), "invalid_kind")
            _require((kind == "controller" and topic in self.controller_topics)
                     or (kind == "goal_status" and topic in self.status_topics)
                     or (kind == "clock" and topic == "/clock"), "unregistered_topic")
            received = self.clock() if received_ns is None else received_ns
            _require(type(received) is int and received >= 0, "invalid_receipt")
            _require(self.last_received_ns is None or received >= self.last_received_ns,
                     "receipt_clock_backwards")
            if kind == "controller":
                payload = _controller(value)
                previous_names = self.joint_names.get(topic)
                _require(previous_names is None or payload["joint_names"] == previous_names,
                         "joint_names_changed")
                previous_stamp = self.controller_stamps.get(topic)
                _require(previous_stamp is None or payload["source_stamp_ns"] >= previous_stamp,
                         "controller_stamp_backwards")
            elif kind == "goal_status":
                payload = _statuses(value)
            else:
                payload = dict(source_stamp_ns=_stamp(value))
        except (ValueError, TypeError, OverflowError) as error:
            self.counts["invalid"] += 1
            reason = str(error) if isinstance(error, ValueError) else "invalid_structure"
            # Validation errors originate only from fixed strings above.
            if reason not in _INVALID_REASONS:
                reason = "invalid_structure"
            self.invalid_reasons[reason] += 1
            return False
        if kind == "clock":
            if self.clock_ns is not None and payload["source_stamp_ns"] < self.clock_ns:
                self.clock_epoch += 1
                self.clock_resets += 1
                self.controller_stamps.clear()
            self.clock_ns = payload["source_stamp_ns"]
        elif kind == "controller":
            self.controller_stamps[topic] = payload["source_stamp_ns"]
            self.joint_names.setdefault(topic, payload["joint_names"])
        event = dict(index=self.counts["accepted"], kind=kind, topic=topic,
                     receipt_monotonic_ns=received, clock_epoch=self.clock_epoch,
                     observed_clock_ns=self.clock_ns, **payload)
        if len(self.events) == self.capacity:
            self.counts["evicted_by_capacity"] += 1
        self.events.append(event)
        self.counts["accepted"] += 1
        self.counts[kind] += 1
        self.topic_counts[topic] += 1
        self.first_received_ns = received if self.first_received_ns is None else self.first_received_ns
        self.last_received_ns = received
        return True

    def finish(self, reason, *, signal_number=None, error_type=None):
        _require(reason in ("duration", "signal", "error", "stopped"), "invalid_completion")
        _require(signal_number is None or isinstance(signal_number, int) and not isinstance(signal_number, bool)
                 and signal_number in (2, 15),
                 "invalid_completion")
        _require(error_type is None or isinstance(error_type, str) and len(error_type) <= 128
                 and re.fullmatch("[A-Za-z_][A-Za-z0-9_]*", error_type), "invalid_completion")
        if self.finished is None:
            self.finished = dict(reason=reason, signal_number=signal_number, error_type=error_type,
                                 utc=datetime.now(timezone.utc).isoformat())

    def snapshot(self):
        return deepcopy(dict(schema_version=1, kind="controller-witness-v1", session_id=self.session_id,
            passive_subscribe_only=True, ros_domain_id=self.domain, completion=self.finished,
            bounds=dict(capacity=self.capacity, max_joint_count=MAX_JOINTS, max_goal_statuses=MAX_STATUSES),
            configured_controllers=list(self.controllers),
            declared_state_publish_rate_hz=50.0,
            rate_semantics="Configured controller default, not an observed rate measurement",
            clock=dict(source="Observed ROS /clock and copied header/goal stamps; common clock identity needs run evidence",
                receipt="time.monotonic_ns in this recorder process", observed_resets=self.clock_resets,
                first_receipt_ns=self.first_received_ns, last_receipt_ns=self.last_received_ns),
            counters={name: self.counts[name] for name in ("received", "accepted", "invalid", "controller",
                "goal_status", "clock", "evicted_by_capacity")},
            invalid_reasons=dict(self.invalid_reasons), truncated=self.counts["evicted_by_capacity"] > 0,
            topic_counts=self.topic_counts,
            retained_count=len(self.events), events=list(self.events),
            limitations=["No native step ID: timestamps support association, not exact same-step alignment.",
                "Source and receipt clocks are separate; controller/transport latency is not measured.",
                "Goal status topics associate action UUIDs; no request ID is inferred.",
                "A backwards /clock is an observed epoch change, not proof of the simulator restart cause.",
                "Bounded latest-event ring; DDS delivery loss is not measured by eviction/invalid counters."]))

    def save(self, path):
        """Atomically publish a new owner-only JSON file; never replace a file/link."""
        path = Path(path).absolute()
        raw = (json.dumps(self.snapshot(), ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n").encode()
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(prefix=".controller-witness-", dir=path.parent, delete=False) as stream:
                temporary = Path(stream.name)
                os.fchmod(stream.fileno(), 0o600)
                stream.write(raw)
                stream.flush()
                os.fsync(stream.fileno())
            os.link(temporary, path)  # An atomic new name; EEXIST also protects symlinks.
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return path


_INVALID_REASONS = {"already_finished", "invalid_kind", "unregistered_topic", "invalid_receipt",
    "receipt_clock_backwards", "invalid_controller", "invalid_header", "invalid_stamp", "invalid_joints",
    "invalid_point", "invalid_dimensions", "nonfinite_value", "missing_state_positions",
    "joint_names_changed", "controller_stamp_backwards", "invalid_statuses", "invalid_structure"}


def ros_stamp(stamp):
    """Copy scalar fields without importing ROS."""
    return dict(sec=stamp.sec, nanosec=stamp.nanosec)


def ros_controller(message):
    """Bound lengths before copying ROS arrays; preserve modern and legacy fields."""
    _require(len(message.joint_names) <= MAX_JOINTS, "invalid_joints")
    _require(not len(message.multi_dof_joint_names), "invalid_joints")
    value = dict(header=dict(stamp=ros_stamp(message.header.stamp), frame_id=message.header.frame_id),
                 joint_names=list(message.joint_names))
    for field in POINT_FIELDS:
        if hasattr(message, field):
            point = getattr(message, field)
            arrays = {}
            for vector in VECTOR_FIELDS:
                data = getattr(point, vector)
                _require(len(data) <= MAX_JOINTS, "invalid_dimensions")
                arrays[vector] = list(data)
            value[field] = dict(time_from_start=ros_stamp(point.time_from_start), **arrays)
    return value


def ros_statuses(message):
    _require(len(message.status_list) <= MAX_STATUSES, "invalid_statuses")
    statuses = []
    for status in message.status_list:
        raw = status.goal_info.goal_id.uuid
        _require(len(raw) == 16 and all(type(value) is int and 0 <= value <= 255 for value in raw),
                 "invalid_statuses")
        statuses.append(dict(goal_uuid=bytes(raw).hex(), goal_stamp=ros_stamp(status.goal_info.stamp),
                             status=status.status))
    return dict(status_list=statuses)


def main(argv=None):
    """Subscribe in an explicitly selected ROS domain and save on duration/SIGINT/SIGTERM."""
    import argparse
    import signal
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", required=True, type=int)
    parser.add_argument("--output", required=True, type=Path, help="New JSON file; parent directory must exist")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--capacity", type=int, default=10_000)
    parser.add_argument("--controllers", nargs="+", default=["left_controller", "right_controller"])
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232 or not math.isfinite(args.duration) or not .1 <= args.duration <= 3600:
        parser.error("domain must be 0..232 and duration 0.1..3600 seconds")
    if args.output.exists() or args.output.is_symlink() or not args.output.parent.is_dir():
        parser.error("output must be a new file with an existing parent directory")
    try:
        recorder = ControllerWitness(capacity=args.capacity, controllers=args.controllers, domain=args.domain)
    except ValueError as error:
        parser.error(str(error))
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    stopped = [None]
    previous = {sig: signal.signal(sig, lambda signum, frame: stopped.__setitem__(0, signum))
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    initialized = False
    failure = None
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from control_msgs.msg import JointTrajectoryControllerState
        from action_msgs.msg import GoalStatusArray
        from rosgraph_msgs.msg import Clock
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = Node("passive_controller_witness", enable_rosout=False, start_parameter_services=False,
                    use_global_arguments=False)
        # Humble creates a standard parameter-events publisher even with its
        # parameter services disabled. Remove that infrastructure writer before
        # capture; this node owns only subscriptions during observation.
        for publisher in tuple(node.publishers):
            node.destroy_publisher(publisher)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def callback(kind, topic, converter):
            def receive(message):
                received = time.monotonic_ns()
                try:
                    value = converter(message)
                except (ValueError, TypeError, AttributeError, OverflowError):
                    recorder.reject()
                else:
                    recorder.observe(kind, topic, value, received_ns=received)
            return receive
        for topic in sorted(recorder.controller_topics):
            node.create_subscription(JointTrajectoryControllerState, topic,
                                     callback("controller", topic, ros_controller), sensor)
        for topic in sorted(recorder.status_topics):
            node.create_subscription(GoalStatusArray, topic,
                                     callback("goal_status", topic, ros_statuses), status_qos)
        node.create_subscription(Clock, "/clock", callback("clock", "/clock", lambda message: ros_stamp(message.clock)), sensor)
        print("CONTROLLER_WITNESS_READY domain=" + str(args.domain), flush=True)
        deadline = time.monotonic() + args.duration
        while stopped[0] is None and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
        recorder.finish("signal" if stopped[0] else "duration" if time.monotonic() >= deadline else "stopped",
                        signal_number=stopped[0])
    except BaseException as error:
        failure = error
        recorder.finish("error", error_type=type(error).__name__)
    finally:
        try:
            recorder.save(args.output)
        finally:
            try:
                if node is not None:
                    node.destroy_node()
                if initialized and rclpy.ok():
                    rclpy.shutdown()
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
