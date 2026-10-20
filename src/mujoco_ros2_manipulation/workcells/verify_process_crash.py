"""Kill one owned core process during motion and require bounded whole-launch shutdown."""

import argparse
import time

import rclpy
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.process_faults import ExpectedCrash
from mujoco_ros2_manipulation.verification.runner import group_gone, run_verification
from mujoco_ros2_manipulation.workcells.config import load_workcell, identifier
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.verify import WorkcellVerification, require


class ProcessCrashVerification(WorkcellVerification):
    def __init__(self, cell, progress=None, crash=None):
        super().__init__(cell, progress)
        self.crash = crash

    def run(self):
        self.progress("crash_test_discovery")
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
        initial = self.state()
        start = self.clock_last + 1
        results = []
        self.progress("both_arms_moving_before_crash")
        for inst in self.cell.instances:
            trajectory = self.trajectory(
                inst,
                [initial[n] + (0.5 if n == inst.joints[0] else 0) for n in inst.joints],
            )
            trajectory.points[-1].time_from_start.sec = 10
            self.validate(trajectory)
            trajectory.header.stamp = Time(
                sec=int(start), nanosec=int((start % 1) * 1e9)
            )
            handle = self.result(
                self.arm_clients[inst.id].send_goal_async(
                    FollowJointTrajectory.Goal(trajectory=trajectory)
                ),
                5,
                "arm acceptance",
            )
            require(handle.accepted, "Pre-crash motion was rejected")
            results.append(handle.get_result_async())
        self.until(
            lambda: all(
                max(abs(self.state()[n] - initial[n]) for n in inst.joints) > 0.01
                and max(abs(self.latest_velocities.get(n, 0)) for n in inst.joints)
                > 0.01
                for inst in self.cell.instances
            ),
            10,
            "both arms in motion",
        )
        require(
            not any(f.done() for f in results), "Goal finished before crash injection"
        )
        moved = {
            inst.id: max(abs(self.state()[n] - initial[n]) for n in inst.joints)
            for inst in self.cell.instances
        }
        remaining = start + 10 - self.clock_last
        require(remaining > 5, "Insufficient motion remaining for crash test")
        self.progress("owned_process_crash_injection")
        injected = self.crash.inject(
            self.launch_log_path.read_text(), self.process_group
        )
        started = time.monotonic()
        at_injection = self.state()
        maximum_drift = [0.0]
        samples_at_injection = self.joint_count

        def exited():
            maximum_drift[0] = max(
                maximum_drift[0],
                max(abs(self.state()[n] - at_injection[n]) for n in self.joint_names),
            )
            return self.launch_process.poll() is not None and group_gone(
                self.process_group
            )

        self.until(exited, 8, "automatic whole-launch shutdown")
        latency = time.monotonic() - started
        # Drain already queued DDS samples before checking that publication stopped.
        deadline = time.monotonic() + 0.3
        self.until(lambda: time.monotonic() >= deadline, 1, "drain queued samples")
        counts = (self.joint_count, self.clock_count)
        deadline = time.monotonic() + 0.5
        self.until(lambda: time.monotonic() >= deadline, 2, "no post-exit publishers")
        require(
            counts == (self.joint_count, self.clock_count),
            "Joint states or clock continued after process-group exit",
        )
        return dict(
            workcell_id=self.cell.id,
            process_crash_test=True,
            verification_scope="owned_core_crash_shutdown",
            crash_target=self.crash.target,
            injected_process=injected,
            both_arms_moving_before_crash_rad=moved,
            trajectory_remaining_at_crash_sim_sec=remaining,
            autonomous_shutdown_wall_sec=latency,
            shutdown_deadline_wall_sec=8,
            verifier_cancel_requests=0,
            verifier_shutdown_requests=0,
            post_exit_publication_stopped=True,
            post_fault_joint_samples=self.joint_count - samples_at_injection,
            post_fault_max_observed_drift_rad=maximum_drift[0],
            physical_hold_verified=False,
            physics_checks_passed=False,
            physics_checks_not_run_reason="Crash test checks process shutdown, not physical hold or full step coverage",
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell", default="fr5_ur5e")
    parser.add_argument(
        "--target", choices=("move_group", "ros2_control_node"), required=True
    )
    parser.add_argument("--domain", type=int, default=115)
    args = parser.parse_args()
    try:
        identifier(args.workcell)
    except ValueError as error:
        parser.error(str(error))
    args.robot, args.scene = args.workcell, "default"
    args.gui, args.screenshot_dir, args.stop_mode = False, None, "interrupt"
    crash = ExpectedCrash(args.target)
    return run_verification(
        args,
        ROOT,
        rclpy,
        lambda cell, progress: ProcessCrashVerification(cell, progress, crash),
        Trigger,
        profile_loader=lambda name, **kw: load_workcell(name, root=kw["root"]),
        validator=check_workcell,
        command_builder=lambda cell, args: [
            "bash",
            str(ROOT / "run_scene.sh"),
            f"workcell:={cell.id}",
            "gui:=false",
            "rviz:=false",
        ],
        report_prefix=f"crash_{args.target}_",
        expected_crash=crash,
    )


if __name__ == "__main__":
    raise SystemExit(main())
