"""Simulation-only intentional contact with a fixed visible obstacle; no injection."""
import argparse
from copy import deepcopy
from pathlib import Path
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

        def stopped():
            if self.joint_count != last_count[0]:
                last_count[0] = self.joint_count
                samples.append(
                    dict(
                        clock_sec=self.clock_last,
                        positions_rad=self.state(),
                        velocities_rad_sec=dict(self.latest_velocities),
                    )
                )
                for inst in self.cell.instances:
                    peak_velocity[inst.id] = max(
                        peak_velocity[inst.id],
                        abs(self.latest_velocities.get(inst.joints[0], 0.0)),
                    )
            return self.contact_guard and self.contact_guard["latched"]

        self.until(stopped, 10, "natural contact during commanded approach")
        self.until(lambda: all(f.done() for f in results), 5, "automatic cancellation")
        statuses = [f.result().status for f in results]
        require(
            all(s == GoalStatus.STATUS_CANCELED for s in statuses),
            "Both goals must be canceled by the guard",
        )
        self.until(
            lambda: self.contact_guard["phase"] == "stopped",
            5,
            "both controllers inactive",
        )
        frozen = self.state()
        displacement = {
            i.id: abs(frozen[i.joints[0]] - initial[i.joints[0]])
            for i in self.cell.instances
        }
        require(
            all(v > 0.7 for v in displacement.values()),
            "Movement too short for the demo",
        )
        require(
            all(v > 0.4 for v in peak_velocity.values()),
            "Movement too slow for the demo",
        )
        count = self.joint_count
        self.until(
            lambda: self.joint_count >= count + 75,
            5,
            "stationary post-contact observation",
        )
        drift = max(abs(self.state()[n] - frozen[n]) for n in self.joint_names)
        require(
            drift == 0 and all(v == 0 for v in self.latest_velocities.values()),
            "Arms moved after contact stop",
        )
        for inst in self.cell.instances:
            require(
                self.contact_guard["controllers"].get(inst.controller) == "inactive",
                "Controller still active",
            )
            retry = self.trajectory(inst, [frozen[n] for n in inst.joints])
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
            contact_stop_test=True,
            natural_fixed_obstacle_contact=True,
            contact_injection_requests=0,
            verifier_cancel_requests=0,
            commanded_first_joint_offsets_rad=list(FIRST_JOINT_OFFSETS),
            commanded_duration_sec=DURATION_SEC,
            first_joint_displacement_rad=displacement,
            first_joint_peak_velocity_rad_sec=peak_velocity,
            initial_positions_rad=initial,
            stopped_positions_rad=frozen,
            goal_terminal_statuses=statuses,
            new_goals_rejected=True,
            post_stop_drift_rad=drift,
            contact_guard=deepcopy(self.contact_guard),
            motion_samples=samples,
            obstacle=dict(
                body=OBSTACLE_BODY,
                geom=OBSTACLE_GEOM,
                xyz_m=list(OBSTACLE_XYZ),
                size_m=list(OBSTACLE_SIZE),
                fixed=True,
            ),
            obstacle_configuration_sha256=file_sha256(
                Path(__file__).with_name("contact_demo.py")
            ),
        )


def finalize_demo(report, cell, args):
    finalize_physics(report, cell, args)
    physics = report.data["physics"]
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
    report.update(fixed_obstacle_contact_checks_passed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=122)
    parser.add_argument("--gui", action="store_true")
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
        True,
    )
    args.screenshot_dir = None

    def factory(cell, progress):
        if args.record_dir:
            from mujoco_ros2_manipulation.workcells.record_contact_demo import (
                RecordedContactDemo,
            )

            return RecordedContactDemo(cell, progress, args.record_dir)
        return VisibleContactDemo(cell, progress)

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
        ],
        report_prefix="visible_contact_",
        launch_environment=diagnostic_environment,
        finalize=finalize_demo,
    )


if __name__ == "__main__":
    raise SystemExit(main())
