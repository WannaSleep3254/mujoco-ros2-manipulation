"""ROS observer/publisher; only this executor thread touches ROS entities."""
import json
import queue
import time
import xml.etree.ElementTree as ET

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from ..workcells.motion_request import COMMAND_TOPIC, CANCEL_TOPIC, STATUS_TOPIC, decode_message


class ConsoleBridge(Node):
    def __init__(self, state, cell):
        super().__init__("workcell_operator_console")
        self.state, self.cell = state, cell
        self.outbox = queue.Queue(maxsize=8)
        self.identity_future = None
        self.identity_requested_at = None
        self.last_identity = 0
        self.commands = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.cancels = self.create_publisher(String, CANCEL_TOPIC, 10)
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self.joints, sensor)
        self.create_subscription(Clock, "/clock", lambda m: state.tick(m.clock.sec+m.clock.nanosec/1e9), sensor)
        self.create_subscription(String, STATUS_TOPIC, self.status_message, reliable)
        self.create_subscription(String, "/workcell/contact_stop", self.guard, reliable)
        self.parameters = self.create_client(GetParameters, "/mujoco_ros2_control_node/get_parameters")
        self.create_timer(0.05, self.send)
        self.create_timer(0.5, self.discover)

    def joints(self, message):
        self.state.joints(list(message.name), list(message.position), list(message.velocity),
                          source_stamp=message.header.stamp.sec + message.header.stamp.nanosec / 1e9)

    def guard(self, message):
        with self.state.lock:
            try:
                data = decode_message(message.data)
                self.state.guard = data if type(data.get("latched")) is bool and isinstance(data.get("phase"), str) else None
            except ValueError:
                self.state.guard = None

    def status_message(self, message):
        try:
            self.state.status(decode_message(message.data))
        except ValueError:
            with self.state.lock:
                self.state.server = None

    def discover(self):
        with self.state.lock:
            now = time.monotonic()
            self.state.graph = dict(clock=self.count_publishers("/clock"), joints=self.count_publishers("/joint_states"),
                status=self.count_publishers(STATUS_TOPIC), commands=self.commands.get_subscription_count(),
                guards=self.count_publishers("/workcell/contact_stop"))
            unique = self.state.graph["clock"] == 1 and self.state.graph["joints"] == 1 and self.parameters.service_is_ready()
            def clear_identity():
                self.state.identity = False
                self.state.observed_model_path = None
                self.state.identity_requested_at = self.state.identity_checked_at = None
            if not unique:
                clear_identity()
                if self.identity_future is not None:
                    self.identity_future.cancel()
                self.identity_future = self.identity_requested_at = None
                return
            if self.identity_future is not None:
                if self.identity_future.done():
                    try:
                        values = self.identity_future.result().values
                        robot = ET.fromstring(values[0].string_value)
                        hardware = robot.find("ros2_control/hardware")
                        model = hardware.find("param[@name='mujoco_model']").text
                        verified = (values[1].bool_value and hardware.find("plugin").text == "mujoco_ros2_control/MujocoSystemInterface"
                            and {j.get("name") for j in robot.findall("ros2_control/joint")} == set(self.cell.joints)
                            and isinstance(model, str) and bool(model) and self.identity_requested_at is not None)
                        if verified:
                            # Owned custom demo models use the same verified
                            # hardware/joints. The view owner checks their path;
                            # command admission still requires the base model.
                            self.state.observed_model_path = model
                            self.state.identity = model == str(self.cell.output / "cell.xml")
                            self.state.identity_requested_at = self.identity_requested_at
                            self.state.identity_checked_at = time.monotonic()
                        else:
                            clear_identity()
                    except Exception:
                        clear_identity()
                    self.identity_future = self.identity_requested_at = None
                elif now-self.last_identity > 3:
                    self.identity_future.cancel()
                    self.identity_future = self.identity_requested_at = None
                    clear_identity()
            if self.identity_future is None and now-self.last_identity > 2:
                self.last_identity = self.identity_requested_at = now
                try:
                    self.identity_future = self.parameters.call_async(GetParameters.Request(names=["robot_description", "use_sim_time"]))
                except Exception:
                    self.identity_future = self.identity_requested_at = None
                    clear_identity()

    def send(self):
        try:
            kind, payload = self.outbox.get_nowait()
        except queue.Empty:
            return
        if kind == "command":
            with self.state.lock:
                identifier = payload["request_id"]
                record = self.state.history.get(identifier)
                # Only reserve()'s still-current, unacknowledged local request
                # owns this queued publication. A server replacement clears its
                # active reservation; received/terminal records must not replay.
                if (self.state.active != identifier or identifier not in self.state.ui_ids
                        or not isinstance(record, dict) or record.get("status") != "submitting"
                        or record.get("stage") != "submitted" or record.get("command") != payload
                        or record.get("session_id") is not None):
                    return  # Preserve unknown/cancelled/completed outcomes as received.
                reason = self.state.reason(ignore_request=identifier)
                if reason:
                    self.state.status(dict(session_id=(self.state.server or {}).get("session_id", "console"),
                        request_id=identifier, status="rejected", stage="pre_dispatch", error=dict(message=reason), metrics={}))
                    return
                self.commands.publish(String(data=json.dumps(payload, allow_nan=False)))
        else:
            self.cancels.publish(String(data=json.dumps(payload)))
