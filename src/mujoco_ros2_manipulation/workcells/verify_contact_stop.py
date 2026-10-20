"""Inject a real test obstacle while both arms move, without client-side cancellation."""
import argparse
from copy import deepcopy
from pathlib import Path
import time
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from mujoco_ros2_control_msgs.srv import ResetWorld, SetPause, StepSimulation
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.diagnostics import (
    diagnostic_environment,
    finalize_physics,
)
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.workcells.config import load_workcell, identifier
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.verify import WorkcellVerification, require


class ContactStopVerification(WorkcellVerification):
    def __init__(self, cell, progress=None, paused_contact=False):
        super().__init__(cell, progress)
        self.paused_contact = paused_contact

    def call(self, kind, name, request):
        client = self.create_client(kind, name)
        self.until(client.service_is_ready, 10, name)
        return self.result(client.call_async(request), 5, name)

    def run(self):
        self.progress("contact_stop_discovery")
        self.until(
            lambda: self.positions is not None and self.clock_count > 10,
            30,
            "shared states",
        )
        self.ready_guard()
        for client in self.arm_clients.values():
            self.until(client.server_is_ready, 10, "arm action server")
        self.until(self.validity.service_is_ready, 15, "MoveIt collision checker")
        self.settle(self.cell.home)
        start_positions = self.state()
        start = self.clock_last + 1
        handles = []
        self.progress("both_arms_moving_before_contact")
        for inst in self.cell.instances:
            target = [
                self.commanded[n] + (0.25 if n == inst.joints[0] else 0)
                for n in inst.joints
            ]
            trajectory = self.trajectory(inst, target)
            trajectory.points[-1].time_from_start.sec = 5
            self.validate(trajectory)
            trajectory.header.stamp = Time(
                sec=int(start), nanosec=int((start - int(start)) * 1e9)
            )
            goal = FollowJointTrajectory.Goal(trajectory=trajectory)
            handle = self.result(
                self.arm_clients[inst.id].send_goal_async(goal),
                5,
                "moving arm acceptance",
            )
            require(handle.accepted, "Pre-contact goal rejected")
            handles.append((handle, handle.get_result_async()))
        self.until(
            lambda: all(
                max(abs(self.state()[n] - start_positions[n]) for n in inst.joints)
                > 0.01
                and max(abs(self.latest_velocities.get(n, 0)) for n in inst.joints)
                > 0.01
                for inst in self.cell.instances
            ),
            10,
            "both arms in motion",
        )
        moved = {
            inst.id: max(abs(self.state()[n] - start_positions[n]) for n in inst.joints)
            for inst in self.cell.instances
        }
        if self.paused_contact:
            paused = self.call(
                SetPause,
                "/mujoco_ros2_control_node/set_pause",
                SetPause.Request(paused=True),
            )
            require(
                paused.success, "Could not exercise paused single-step contact path"
            )
        self.progress("contact_injection")
        injected_at = time.monotonic()
        injected = self.call(
            Trigger, "/mujoco_ros2_control_node/inject_test_contact", Trigger.Request()
        )
        require(injected.success, injected.message)
        if self.paused_contact:
            stepped = self.call(
                StepSimulation,
                "/mujoco_ros2_control_node/step_simulation",
                StepSimulation.Request(steps=1),
            )
            require(stepped.success, "Paused contact step failed")
        # Deliberately do not call dispatch() or cancel_peers(): only the guard stops these goals.
        self.until(
            lambda: self.contact_guard and self.contact_guard["latched"],
            5,
            "native contact stop",
        )
        observed_after = time.monotonic() - injected_at
        self.until(
            lambda: all(future.done() for _, future in handles),
            5,
            "autonomous goal termination",
        )
        statuses = [future.result().status for _, future in handles]
        require(
            all(
                status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
                for status in statuses
            ),
            f"A contact-stopped goal was not canceled/aborted: {statuses}",
        )
        self.until(
            lambda: self.contact_guard["phase"] == "stopped",
            5,
            "both controllers inactive",
        )
        require(
            all(
                self.contact_guard["controllers"].get(i.controller) == "inactive"
                for i in self.cell.instances
            ),
            "Both arm controllers must be inactive",
        )
        stopped = self.state()
        stopped_clock = self.clock_last
        count = self.joint_count
        self.until(
            lambda: self.joint_count >= count + 20, 3, "fresh frozen joint states"
        )
        require(
            max(abs(self.state()[n] - stopped[n]) for n in self.joint_names) < 1e-12,
            "Arm moved after contact latch",
        )
        require(
            max(abs(self.latest_velocities[n]) for n in self.joint_names) == 0,
            "Nonzero velocity after contact latch",
        )
        self.progress("post_stop_command_rejection")
        for inst in self.cell.instances:
            trajectory = self.trajectory(inst, [stopped[n] + 0.1 for n in inst.joints])
            handle = self.result(
                self.arm_clients[inst.id].send_goal_async(
                    FollowJointTrajectory.Goal(trajectory=trajectory)
                ),
                5,
                "post-stop goal",
            )
            require(
                not handle.accepted, "Inactive controller accepted a new trajectory"
            )
            publisher = self.create_publisher(
                JointTrajectory, f"/{inst.controller}/joint_trajectory", 10
            )
            self.until(
                lambda: publisher.get_subscription_count() > 0,
                5,
                "direct command subscriber",
            )
            publisher.publish(trajectory)
        self.progress("latched_stop_cannot_be_reset")
        for kind, name, request in [
            (ResetWorld, "reset_world", ResetWorld.Request()),
            (SetPause, "set_pause", SetPause.Request(paused=False)),
            (StepSimulation, "step_simulation", StepSimulation.Request(steps=1)),
            (Trigger, "inject_test_contact", Trigger.Request()),
        ]:
            response = self.call(kind, "/mujoco_ros2_control_node/" + name, request)
            require(not response.success, "Mutation bypassed latched stop: " + name)
        before = self.contact_guard["deactivation_requests"]
        request = SwitchController.Request()
        request.activate_controllers = [i.controller for i in self.cell.instances]
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 2
        reactivated = self.call(
            SwitchController, "/controller_manager/switch_controller", request
        )
        require(reactivated.ok, "Could not exercise external reactivation watchdog")
        self.until(
            lambda: self.contact_guard["phase"] == "stopped"
            and self.contact_guard["deactivation_requests"] > before,
            5,
            "reactivation blocked by watchdog",
        )
        until = time.monotonic() + 1
        maximum_drift = [0.0]

        def held():
            maximum_drift[0] = max(
                maximum_drift[0],
                max(abs(self.state()[n] - stopped[n]) for n in self.joint_names),
            )
            return time.monotonic() >= until

        self.until(held, 3, "frozen world despite attempted commands")
        require(
            maximum_drift[0] < 1e-12, "A command/reset/reactivation moved a stopped arm"
        )
        require(
            self.clock_last - stopped_clock > 0.5 and not self.clock_backwards,
            "ROS clock did not progress monotonically after the physical stop",
        )
        native = self.call(
            Trigger, "/mujoco_ros2_control_node/contact_stop_status", Trigger.Request()
        )
        require(
            native.success and "latched" in native.message, "Native latch was cleared"
        )
        return dict(
            workcell_id=self.cell.id,
            paused_contact=self.paused_contact,
            contact_stop_test=True,
            both_arms_moving_before_contact_rad=moved,
            guard_observed_wall_sec=observed_after,
            goal_terminal_statuses=statuses,
            verifier_cancel_requests=0,
            new_goals_rejected=True,
            direct_commands_ignored=True,
            reset_resume_step_rejected=True,
            reactivation_watchdog_passed=True,
            post_stop_drift_rad=maximum_drift[0],
            clock_advance_after_stop_sec=self.clock_last - stopped_clock,
            stopped_positions_rad=stopped,
            contact_guard=deepcopy(self.contact_guard),
            simulated_clock_continues_while_physics_frozen=True,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell", default="fr5_ur5e")
    parser.add_argument("--domain", type=int, default=111)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--paused-contact", action="store_true")
    parser.add_argument("--screenshot-dir", type=Path)
    args = parser.parse_args()
    try:
        identifier(args.workcell)
    except ValueError as error:
        parser.error(str(error))
    args.robot, args.scene = args.workcell, "default"
    args.stop_mode = "interrupt"
    args.require_gpu = args.gui
    args.contact_stop_test = True
    return run_verification(
        args,
        ROOT,
        rclpy,
        lambda cell, progress: ContactStopVerification(
            cell, progress, args.paused_contact
        ),
        Trigger,
        profile_loader=lambda name, **kw: load_workcell(name, root=kw["root"]),
        validator=check_workcell,
        command_builder=lambda cell, args: [
            "bash",
            str(ROOT / "run_scene.sh"),
            f"workcell:={cell.id}",
            f"gui:={str(args.gui).lower()}",
            f"rviz:={str(args.gui).lower()}",
            "contact_test:=true",
        ],
        report_prefix="contact_stop_",
        launch_environment=diagnostic_environment,
        finalize=finalize_physics,
    )


if __name__ == "__main__":
    raise SystemExit(main())
