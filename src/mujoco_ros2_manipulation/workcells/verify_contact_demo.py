"""Simulation-only intentional contact with a fixed visible obstacle; no injection."""
import argparse
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.verification.diagnostics import (
    diagnostic_environment,
    finalize_physics,
)
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.contact_demo import (
    OBSTACLE_BODY,
    OBSTACLE_GEOM,
    OBSTACLE_XYZ,
    OBSTACLE_SIZE,
    FIRST_JOINT_OFFSETS,
    DURATION_SEC,
)
from mujoco_ros2_manipulation.workcells.verify import WorkcellVerification, require


class VisibleContactDemo(WorkcellVerification):
    def __init__(self, cell, progress=None, control=False):
        super().__init__(cell, progress)
        self.control = control

    def before_motion(self):
        pass

    def after_stop(self):
        pass

    def run(self):
        self.progress("visible_obstacle_discovery")
        self.until(
            lambda: self.positions is not None and self.clock_count > 10,
            30,
            "shared states and clock",
        )
        self.ready_guard()
        for client in self.arm_clients.values():
            self.until(client.server_is_ready, 15, "arm action server")
        require(
            not any(
                name.endswith("/inject_test_contact")
                for name, _ in self.get_service_names_and_types()
            ),
            "Fixed-obstacle demo must not expose contact injection",
        )
        self.settle(self.cell.home)
        self.before_motion()
        initial = self.state()
        start = self.clock_last + 1.0
        results = []
        peak_velocity = {i.id: 0.0 for i in self.cell.instances}
        samples = []
        commands = []
        for inst, offset in zip(self.cell.instances, FIRST_JOINT_OFFSETS):
            target = [
                self.commanded[n] + (offset if n == inst.joints[0] else 0)
                for n in inst.joints
            ]
            trajectory = self.trajectory(inst, target)
            trajectory.points[-1].time_from_start.sec = int(DURATION_SEC)
            trajectory.points[-1].time_from_start.nanosec = int(
                (DURATION_SEC % 1) * 1e9
            )
            returning = deepcopy(trajectory.points[0])
            returning.time_from_start.sec = int(2 * DURATION_SEC)
            trajectory.points.append(returning)
            commands.append(
                dict(
                    joints=trajectory.joint_names,
                    points=[
                        dict(
                            positions=list(p.positions),
                            seconds=p.time_from_start.sec,
                            nanoseconds=p.time_from_start.nanosec,
                        )
                        for p in trajectory.points
                    ],
                )
            )
            trajectory.header.stamp = Time(
                sec=int(start), nanosec=int((start % 1) * 1e9)
            )
            # Intentionally colliding simulation test: never pass this trajectory to a real robot.
            handle = self.result(
                self.arm_clients[inst.id].send_goal_async(
                    FollowJointTrajectory.Goal(trajectory=trajectory)
                ),
                5,
                "demo goal accepted",
            )
            require(handle.accepted, "Demo goal rejected")
            results.append(handle.get_result_async())
        self.progress("approach_fixed_obstacle")
        last_count = [-1]

        def observe():
            if self.joint_count != last_count[0]:
                last_count[0] = self.joint_count
                for inst in self.cell.instances:
                    peak_velocity[inst.id] = max(
                        peak_velocity[inst.id],
                        abs(self.latest_velocities.get(inst.joints[0], 0.0)),
                    )
                if not samples or self.clock_last - samples[-1]["clock_sec"] >= 0.1:
                    samples.append(
                        dict(
                            clock_sec=self.clock_last,
                            positions_rad=self.state(),
                            velocities_rad_sec=dict(self.latest_velocities),
                        )
                    )
            return self.contact_guard and self.contact_guard["latched"]

        if self.control:
            self.until(
                lambda: observe() or all(f.done() for f in results),
                2 * DURATION_SEC + 6,
                "control completes outbound and return motion",
            )
            require(
                not self.contact_guard["latched"],
                "Control unexpectedly contacted something",
            )
        else:
            self.until(observe, 10, "natural contact during commanded approach")
        observed_end_clock = self.clock_last
        self.until(lambda: all(f.done() for f in results), 5, "terminal arm results")
        statuses = [f.result().status for f in results]
        expected = (
            GoalStatus.STATUS_SUCCEEDED if self.control else GoalStatus.STATUS_CANCELED
        )
        require(all(s == expected for s in statuses), "Unexpected arm terminal status")
        if self.control:
            require(
                all(f.result().result.error_code == 0 for f in results),
                "Control trajectory failed",
            )
            self.settle(self.cell.home)
        else:
            self.until(
                lambda: self.contact_guard["phase"] == "stopped",
                5,
                "both controllers inactive",
            )
        final = self.state()
        displacement = {
            i.id: abs(final[i.joints[0]] - initial[i.joints[0]])
            for i in self.cell.instances
        }
        peak_displacement = {
            i.id: max(
                abs(s["positions_rad"][i.joints[0]] - initial[i.joints[0]])
                for s in samples
            )
            for i in self.cell.instances
        }
        require(
            all(v > 0.4 for v in peak_velocity.values()), "Movement too slow for demo"
        )
        if self.control:
            require(
                all(v > 1.8 for v in peak_displacement.values()),
                "Control did not reach outbound target",
            )
            require(
                all(v < 0.01 for v in displacement.values()),
                "Control did not return to its starting angle",
            )
        else:
            require(
                all(0.7 < v < 1.3 for v in displacement.values()),
                "Contact did not interrupt outbound motion",
            )
            require(
                start + 2 * DURATION_SEC - observed_end_clock > 4.5,
                "Contact occurred too close to planned completion",
            )
        # Observe both variants past the original end of the complete command.
        count = self.joint_count

        def observed_tail():
            observe()
            return (
                self.clock_last >= start + 2 * DURATION_SEC + 1.5
                and self.joint_count >= count + 75
            )

        self.until(
            observed_tail,
            2 * DURATION_SEC + 7,
            "full planned motion and stationary tail",
        )
        drift = max(abs(self.state()[n] - final[n]) for n in self.joint_names)
        if not self.control:
            require(
                drift == 0 and all(v == 0 for v in self.latest_velocities.values()),
                "Arms moved after contact stop",
            )
        for inst in self.cell.instances:
            expected_controller = "active" if self.control else "inactive"
            require(
                self.contact_guard["controllers"].get(inst.controller)
                == expected_controller,
                "Unexpected controller state",
            )
            if not self.control:
                retry = self.trajectory(inst, [final[n] for n in inst.joints])
                handle = self.result(
                    self.arm_clients[inst.id].send_goal_async(
                        FollowJointTrajectory.Goal(trajectory=retry)
                    ),
                    5,
                    "post-stop rejection",
                )
                require(not handle.accepted, "Stopped controller accepted a goal")
        self.after_stop()
        return dict(
            workcell_id=self.cell.id,
            control_without_obstacle=self.control,
            contact_stop_test=not self.control,
            natural_fixed_obstacle_contact=not self.control,
            contact_injection_requests=0,
            verifier_cancel_requests=0,
            commanded_first_joint_offsets_rad=list(FIRST_JOINT_OFFSETS),
            commanded_duration_sec=2 * DURATION_SEC,
            outbound_duration_sec=DURATION_SEC,
            commanded_trajectories=commands,
            command_sha256=hashlib.sha256(
                json.dumps(commands, sort_keys=True).encode()
            ).hexdigest(),
            motion_start_clock_sec=start,
            motion_end_clock_sec=start + 2 * DURATION_SEC,
            guard_latch_clock_sec=None if self.control else observed_end_clock,
            remaining_motion_at_contact_sec=None
            if self.control
            else start + 2 * DURATION_SEC - observed_end_clock,
            first_joint_peak_displacement_rad=peak_displacement,
            first_joint_displacement_rad=displacement,
            first_joint_peak_velocity_rad_sec=peak_velocity,
            initial_positions_rad=initial,
            stopped_positions_rad=final,
            goal_terminal_statuses=statuses,
            new_goals_rejected=not self.control,
            post_stop_drift_rad=None if self.control else drift,
            final_holding_drift_rad=drift,
            motion_sample_minimum_period_sec=0.1,
            contact_guard=deepcopy(self.contact_guard),
            motion_samples=samples,
            obstacle=dict(
                body=OBSTACLE_BODY,
                geom=OBSTACLE_GEOM,
                xyz_m=list(OBSTACLE_XYZ),
                size_m=list(OBSTACLE_SIZE),
                fixed=True,
                present=not self.control,
            ),
            obstacle_configuration_sha256=file_sha256(
                Path(__file__).with_name("contact_demo.py")
            ),
        )


def finalize_demo(report, cell, args):
    finalize_physics(report, cell, args)
    physics = report.data["physics"]
    report.update(
        contact_demo_model_sha256=file_sha256(report.directory / "contact_test.xml")
    )
    if args.control:
        report.update(control_no_contact_checks_passed=True)
        return
    require(
        all(
            OBSTACLE_BODY in p["bodies"]
            and any(n.startswith("left_") for n in p["bodies"])
            for p in physics["pairs"]
        ),
        "Unexpected contact pair during demo",
    )
    require(
        all(p["min_distance_m"] > -0.01 for p in physics["pairs"]),
        "Deep overlap instead of surface approach",
    )
    remaining = report.data["motion_end_clock_sec"] - physics["stop_time_sec"]
    require(remaining > 4.5, "Contact must interrupt a substantial remaining command")
    report.update(
        fixed_obstacle_contact_checks_passed=True,
        native_remaining_motion_at_contact_sec=remaining,
        native_contact_motion_elapsed_sec=physics["stop_time_sec"]
        - report.data["motion_start_clock_sec"],
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=122)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--control",
        action="store_true",
        help="Remove the obstacle; run the identical complete motion",
    )
    parser.add_argument(
        "--record-dir",
        type=Path,
        help="Capture the actual MuJoCo window; requires --gui, system OpenCV/Pillow/X11 tools",
    )
    args = parser.parse_args()
    if args.record_dir and not args.gui:
        parser.error("--record-dir requires --gui")
    args.robot, args.scene = "fr5_ur5e", "default"
    args.stop_mode, args.require_gpu, args.contact_stop_test = (
        "interrupt",
        args.gui,
        not args.control,
    )
    args.screenshot_dir = None

    def factory(cell, progress):
        if args.record_dir:
            from mujoco_ros2_manipulation.workcells.record_contact_demo import (
                RecordedContactDemo,
            )

            return RecordedContactDemo(
                cell, progress, args.record_dir, control=args.control
            )
        return VisibleContactDemo(cell, progress, control=args.control)

    return run_verification(
        args,
        ROOT,
        rclpy,
        factory,
        Trigger,
        profile_loader=lambda name, **kw: load_workcell(name, root=kw["root"]),
        validator=check_workcell,
        command_builder=lambda cell, args: [
            "bash",
            str(ROOT / "run_scene.sh"),
            f"workcell:={cell.id}",
            f"gui:={str(args.gui).lower()}",
            f"rviz:={str(args.gui).lower()}",
            "contact_demo:=true",
            f"contact_demo_control:={str(args.control).lower()}",
        ],
        report_prefix="control_contact_" if args.control else "visible_contact_",
        launch_environment=diagnostic_environment,
        finalize=finalize_demo,
    )


if __name__ == "__main__":
    raise SystemExit(main())
