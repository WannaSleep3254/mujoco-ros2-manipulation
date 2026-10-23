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
            self.state.graph = dict(clock=self.count_publishers("/clock"), joints=self.count_publishers("/joint_states"),
                status=self.count_publishers(STATUS_TOPIC), commands=self.commands.get_subscription_count(),
                guards=self.count_publishers("/workcell/contact_stop"))
            if self.state.graph["clock"] != 1 or self.state.graph["joints"] != 1 or not self.parameters.service_is_ready():
                self.state.identity = False
                self.state.observed_model_path = None
            if self.identity_future:
                if self.identity_future.done():
                    try:
                        values = self.identity_future.result().values
                        robot = ET.fromstring(values[0].string_value)
                        hardware = robot.find("ros2_control/hardware")
                        model = hardware.find("param[@name='mujoco_model']").text
                        verified = (values[1].bool_value and hardware.find("plugin").text == "mujoco_ros2_control/MujocoSystemInterface"
                            and {j.get("name") for j in robot.findall("ros2_control/joint")} == set(self.cell.joints))
                        self.state.observed_model_path = model if verified else None
                        self.state.identity = verified and model == str(self.cell.output / "cell.xml")
                    except Exception:
                        self.state.identity = False
                        self.state.observed_model_path = None
                    self.identity_future = None
                elif time.monotonic()-self.last_identity > 3:
                    self.identity_future.cancel()
                    self.identity_future = None
                    self.state.identity = False
                    self.state.observed_model_path = None
            if not self.identity_future and time.monotonic()-self.last_identity > 2 and self.parameters.service_is_ready():
                self.last_identity = time.monotonic()
                self.identity_future = self.parameters.call_async(GetParameters.Request(names=["robot_description", "use_sim_time"]))

    def send(self):
        try:
            kind, payload = self.outbox.get_nowait()
        except queue.Empty:
            return
        if kind == "command":
            with self.state.lock:
                reason = self.state.reason(ignore_request=payload["request_id"])
                if reason:
                    self.state.status(dict(session_id=(self.state.server or {}).get("session_id", "console"),
                        request_id=payload["request_id"], status="rejected", stage="pre_dispatch", error=dict(message=reason), metrics={}))
                    return
                self.commands.publish(String(data=json.dumps(payload, allow_nan=False)))
        else:
            self.cancels.publish(String(data=json.dumps(payload)))
