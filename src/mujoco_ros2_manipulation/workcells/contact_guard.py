"""Cancel both goals and keep both controllers inactive after the native stop latch."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import ListControllers, SwitchController
from mujoco_ros2_manipulation.workcells.contact_policy import ContactPolicy


class ContactGuard(Node):
    def __init__(self, controllers):
        super().__init__("workcell_contact_guard")
        self.policy = ContactPolicy(controllers)
        self.states = {}
        self.query = self.switch = None
        self.pending_at = 0
        self.cancel_started = None
        self.cancel_futures = []
        self.last_error = ""
        self.switch_count = 0
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, "/workcell/contact_stop", qos)
        self.create_subscription(
            Bool, "/mujoco_ros2_control_node/contact_stop", self.event, qos
        )
        self.manager = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.switcher = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.cancellers = [
            self.create_client(
                CancelGoal, f"/{name}/follow_joint_trajectory/_action/cancel_goal"
            )
            for name in controllers
        ]
        self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def event(self, message):
        self.policy.observe(message.data)
        self.publish()

    def publish(self):
        self.publisher.publish(
            String(
                data=json.dumps(
                    dict(
                        latched=self.policy.latched,
                        phase=self.policy.phase(self.states),
                        controllers=self.states,
                        deactivation_requests=self.switch_count,
                        error=self.last_error,
                    )
                )
            )
        )

    def tick(self):
        now = time.monotonic()
        if self.policy.latched and self.cancel_started is None:
            self.cancel_started = now
            for client in self.cancellers:
                if client.service_is_ready():
                    # Zero UUID and zero stamp select all goals on this arm.
                    self.cancel_futures.append(client.call_async(CancelGoal.Request()))
        # No ROS-clock wait or synchronous service call is made in this executor.
        if self.switch is not None:
            if self.switch.done():
                try:
                    if not self.switch.result().ok:
                        self.last_error = "Controller deactivation refused; retrying"
                except Exception as error:
                    self.last_error = str(error)
                self.switch = None
            elif now - self.pending_at > 3:
                self.switcher.remove_pending_request(self.switch)
                self.switch = None
                self.last_error = "Controller deactivation timed out; retrying"
            else:
                self.publish()
                return
        if self.query is not None:
            if not self.query.done():
                if now - self.pending_at > 2:
                    self.manager.remove_pending_request(self.query)
                    self.query = None
                    self.last_error = "Controller discovery timed out; retrying"
                self.publish()
                return
            try:
                self.states = {c.name: c.state for c in self.query.result().controller}
            except Exception as error:
                self.last_error = str(error)
            self.query = None
            targets = self.policy.targets(self.states)
            cancels_ready = all(f.done() for f in self.cancel_futures)
            if (
                targets
                and self.switcher.service_is_ready()
                and (cancels_ready or now - self.cancel_started >= 0.5)
            ):
                request = SwitchController.Request()
                request.deactivate_controllers = targets
                request.strictness = SwitchController.Request.STRICT
                request.timeout.sec = 2
                self.switch = self.switcher.call_async(request)
                self.switch_count += 1
                self.pending_at = now
        if (
            self.query is None
            and self.switch is None
            and self.manager.service_is_ready()
        ):
            self.query = self.manager.call_async(ListControllers.Request())
            self.pending_at = now
        self.publish()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controllers", nargs=2, required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ContactGuard(args.controllers)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
