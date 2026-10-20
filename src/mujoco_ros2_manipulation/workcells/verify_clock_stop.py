"""Verify missing/repeated ROS clock faults, intentional pause, and immutable dual-arm hold."""
import argparse
from pathlib import Path
import time

import rclpy
from mujoco_ros2_control_msgs.srv import SetPause
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.verification.diagnostics import (
    diagnostic_environment,
    finalize_physics,
)
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.verify import require
from mujoco_ros2_manipulation.workcells.verify_contact_stop import (
    ContactStopVerification,
)


class ClockStopVerification(ContactStopVerification):
    fault_service = "inject_test_clock_stall"
    fault_reason = "clock_stall"

    def __init__(self, cell, progress=None, mode="missing"):
        self.mode = mode
        self.clock_samples = []
        super().__init__(cell, progress)

    def clock(self, message):
        super().clock(message)
        self.clock_samples.append((time.monotonic(), self.clock_last))

    def before_motion(self):
        self.progress("intentional_pause_does_not_latch_clock_stop")
        paused = self.call(
            SetPause,
            "/mujoco_ros2_control_node/set_pause",
            SetPause.Request(paused=True),
        )
        require(paused.success, paused.message)
        drain = time.monotonic() + 0.2
        self.until(
            lambda: time.monotonic() >= drain, 1, "drain pre-pause clock samples"
        )
        pause_clock = self.clock_last
        began = time.monotonic()
        self.until(
            lambda: time.monotonic() - began >= 1.5,
            3,
            "intentional pause beyond watchdog deadline",
        )
        require(
            self.clock_last == pause_clock,
            "Intentional pause did not stop simulation time",
        )
        require(
            not self.contact_guard["latched"],
            "Intentional pause caused a false clock stop",
        )
        count = self.clock_count
        response = self.call(
            SetPause,
            "/mujoco_ros2_control_node/set_pause",
            SetPause.Request(paused=False),
        )
        require(response.success, response.message)
        self.until(
            lambda: self.clock_count > count + 10,
            3,
            "clock resumes after intentional pause",
        )
        require(not self.contact_guard["latched"], "Resume caused a false clock stop")

    def run(self):
        result = super().run()
        # The publication fault lasts ~1s; stationary physical hold continues after
        # recovery so ROS controllers can cancel using their simulation clock.
        require(
            result["guard_observed_wall_sec"] <= 2.0,
            "Clock fault response exceeded 2s wall deadline",
        )
        times = [
            (wall, stamp)
            for wall, stamp in self.clock_samples
            if wall >= self.fault_injected_wall - 0.1
        ]
        longest_gap = max(b[0] - a[0] for a, b in zip(times, times[1:]))
        repeated = [(a, b) for a, b in zip(times, times[1:]) if a[1] == b[1]]
        repeat_span = sum(b[0] - a[0] for a, b in repeated)
        if self.mode == "missing":
            require(longest_gap > 0.5, "No missing-clock interval observed")
        else:
            require(
                len(repeated) >= 10 and repeat_span > 0.5,
                "Repeated clock stamps were not observed",
            )
        result.update(
            verification_scope="clock_stall_dual_arm_hold",
            clock_fault_mode=self.mode,
            intentional_pause_test_wall_sec=1.5,
            intentional_pause_no_false_stop=True,
            longest_clock_message_gap_wall_sec=longest_gap,
            repeated_clock_messages=len(repeated),
            repeated_clock_span_wall_sec=repeat_span,
            physical_hold_verified=True,
            stop_observation_deadline_wall_sec=2.0,
        )
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("missing", "repeated"), default="missing")
    parser.add_argument("--domain", type=int, default=119)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--screenshot-dir", type=Path)
    args = parser.parse_args()
    args.robot, args.scene, args.stop_mode = "fr5_ur5e", "default", "interrupt"
    args.require_gpu, args.clock_stop_test = args.gui, True
    return run_verification(
        args,
        ROOT,
        rclpy,
        lambda cell, progress: ClockStopVerification(cell, progress, args.mode),
        Trigger,
        profile_loader=lambda name, **kw: load_workcell(name, root=kw["root"]),
        validator=check_workcell,
        command_builder=lambda cell, args: [
            "bash",
            str(ROOT / "run_scene.sh"),
            f"workcell:={cell.id}",
            f"gui:={str(args.gui).lower()}",
            f"rviz:={str(args.gui).lower()}",
            f"clock_test:={args.mode}",
        ],
        report_prefix=f"clock_{args.mode}_",
        launch_environment=diagnostic_environment,
        finalize=finalize_physics,
    )


if __name__ == "__main__":
    raise SystemExit(main())
