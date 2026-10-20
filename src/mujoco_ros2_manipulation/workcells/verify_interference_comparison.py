"""Simulation-only matched single-arm, actual contact, and coordinated avoidance runs."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time, Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import (
    diagnostic_environment,
    finalize_physics,
    check_stop_physics,
)
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.interference_comparison import (
    MODES,
    load_bundle,
    freeze_bundle,
)
from mujoco_ros2_manipulation.workcells.verify_interference import (
    InterferenceVerification,
    preflight,
)
from mujoco_ros2_manipulation.workcells.verify import require


def to_ros(curve, start=0):
    trajectory = JointTrajectory()
    trajectory.header.stamp = Time(sec=int(start), nanosec=round((start % 1) * 1e9))
    trajectory.joint_names = list(curve.names)
    for point in curve.points:
        sec, nsec = divmod(round(point["t"] * 1e9), 10 ** 9)
        trajectory.points.append(
            JointTrajectoryPoint(
                positions=list(map(float, point["p"])),
                velocities=list(map(float, point["v"])),
                accelerations=list(map(float, point["a"])),
                time_from_start=Duration(sec=sec, nanosec=nsec),
            )
        )
    return trajectory


class ComparisonVerification(InterferenceVerification):
    def __init__(self, cell, progress, bundle, mode):
        self.bundle_path = Path(bundle).resolve()
        self.bundle_hash = file_sha256(self.bundle_path)
        self.bundle, self.curves = load_bundle(self.bundle_path, cell)
        super().__init__(cell, progress, self.bundle["case_id"])
        self.mode = mode
        self.samples = []
        self.sample_after = None
        self.peer_drift = 0.0

    def joints(self, message):
        super().joints(message)
        if self.sample_after is not None:
            if (
                not self.samples
                or self.clock_last - self.samples[-1]["clock_sec"] >= 0.05
            ):
                self.samples.append(
                    dict(
                        clock_sec=self.clock_last,
                        positions_rad=self.state(),
                        velocities_rad_sec=dict(self.latest_velocities),
                    )
                )
            if self.mode in ("left", "right"):
                peer = next(i for i in self.cell.instances if i.id != self.mode)
                self.peer_drift = max(
                    self.peer_drift,
                    max(
                        abs(self.state()[n] - self.sample_after[n]) for n in peer.joints
                    ),
                )

    def save_comparison(self, **values):
        atomic_json(
            self.launch_log_path.parent / "comparison_motion.json",
            dict(
                **(
                    dict(
                        mode=self.mode,
                        bundle_sha256=self.bundle_hash,
                        samples=self.samples,
                    )
                    | values
                ),
            ),
        )

    def run(self):
        self.discover()
        require(
            not any(
                name.endswith("/inject_test_contact")
                for name, _ in self.get_service_names_and_types()
            ),
            "Comparison must not expose a contact injection service",
        )
        # Recheck the same saved splines on every fresh launch, before any experimental command.
        checks = {
            key: self.assess(
                [self.curves[key]], self.bundle["initial_rad"], complete=False
            )
            for key in ("left", "right")
        }
        require(
            all(c["invalid_samples"] == 0 for c in checks.values()),
            "Single-arm control is colliding",
        )
        parallel = self.assess(
            [self.curves["left"], self.curves["right"]], self.bundle["initial_rad"]
        )
        require(
            parallel["inter_arm_collision_samples"] > 0,
            "No arm-to-arm reference collision",
        )
        avoidance = self.assess([self.curves["avoidance"]], self.bundle["initial_rad"])
        require(avoidance["invalid_samples"] == 0, "Saved avoidance plan is colliding")
        self.execute_joint(to_ros(self.curves["setup"]), dict(self.commanded), "setup")
        require(
            self.commanded == self.bundle["initial_rad"],
            "Preparation did not reach exact commanded start",
        )
        initial_measured = self.state()
        self.before_avoidance_execution()  # optional recorder; camera only, no model changes
        self.sample_after = dict(initial_measured)
        start = self.clock_last + 1.0
        duration = (
            self.curves["avoidance"].duration
            if self.mode == "avoidance"
            else self.curves["left"].duration
        )
        comparison_duration = (
            max(c.duration for k, c in self.curves.items() if k != "setup") + 2
        )
        if self.mode == "avoidance":
            commands = self.split(to_ros(self.curves["avoidance"]), start)
        else:
            commands = [
                (i, to_ros(self.curves[i.id], start))
                for i in self.cell.instances
                if self.mode == "collision" or i.id == self.mode
            ]
        self.save_comparison(stage="before_dispatch", motion_start_clock_sec=start)
        if self.mode == "collision":
            # Explicit simulation-only negative control. Guard stays armed; no contact injection,
            # qpos teleport, client cancellation or automatic use from normal dispatch paths.
            service = self.create_client(
                GetParameters, "/mujoco_ros2_control_node/get_parameters"
            )
            self.until(service.service_is_ready, 5, "simulation hardware parameters")
            params = self.result(
                service.call_async(
                    GetParameters.Request(names=["robot_description", "use_sim_time"])
                ),
                5,
                "simulation hardware",
            )
            require(
                "<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>"
                in params.values[0].string_value
                and params.values[1].bool_value is True,
                "Intentional contact requires MuJoCo simulation hardware",
            )
            results = []
            pending = [
                self.arm_clients[i.id].send_goal_async(
                    FollowJointTrajectory.Goal(trajectory=t)
                )
                for i, t in commands
            ]
            for future in pending:
                handle = self.result(future, 5, "negative-control goal acceptance")
                require(handle.accepted, "Negative control rejected before motion")
                results.append(handle.get_result_async())
            require(
                start - self.clock_last > 0.1,
                "Both goals must be accepted before start",
            )
            self.progress("natural_arm_to_arm_contact")
            self.until(
                lambda: self.contact_guard["latched"],
                duration + 5,
                "actual arm contact",
            )
            latch_clock = self.clock_last
            self.until(
                lambda: all(f.done() for f in results)
                and self.contact_guard["phase"] == "stopped",
                5,
                "both arms stopped",
            )
            statuses = [f.result().status for f in results]
            require(statuses == [5, 5], "Guard must cancel both moving arms")
            require(self.contact_guard["reason"] == "contact", "Unexpected stop reason")
            require(
                start + duration - latch_clock > 1,
                "Contact must interrupt unfinished commands",
            )
            stopped = self.state()
        else:
            latch_clock = None
            self.dispatch(commands, scheduled_start=start)
            statuses = [f.result().status for _, f in self.handles]
            target = dict(self.bundle["initial_rad"])
            for _, t in commands:
                target.update(zip(t.joint_names, t.points[-1].positions))
            self.settle([target[n] for n in self.joint_names])
            require(
                not self.contact_guard["latched"],
                "Unexpected contact in normal or avoidance execution",
            )
            stopped = self.state()
        # All four recordings cover the longest plan plus an equal stationary tail.
        count = self.joint_count
        self.until(
            lambda: self.clock_last >= start + comparison_duration
            and self.joint_count > count + 50,
            comparison_duration + 5,
            "complete aligned comparison timeline",
        )
        drift = max(abs(self.state()[n] - stopped[n]) for n in self.joint_names)
        if self.mode == "collision":
            require(
                drift == 0 and all(v == 0 for v in self.latest_velocities.values()),
                "Post-contact position drift",
            )
            require(
                all(
                    self.contact_guard["controllers"].get(i.controller) == "inactive"
                    for i in self.cell.instances
                ),
                "Both controllers must stay inactive",
            )
            for inst in self.cell.instances:
                retry = self.result(
                    self.arm_clients[inst.id].send_goal_async(
                        FollowJointTrajectory.Goal(
                            trajectory=self.trajectory(
                                inst, [stopped[n] for n in inst.joints]
                            )
                        )
                    ),
                    5,
                    "post-stop rejection",
                )
                require(not retry.accepted, "Stopped controller accepted a new command")
        else:
            require(
                self.peer_drift < 0.01,
                "Stationary peer moved during single-arm control",
            )
        require(
            file_sha256(self.bundle_path) == self.bundle_hash,
            "Bundle changed during run",
        )
        hashes = {
            k: hashlib.sha256(
                json.dumps(c.record(), sort_keys=True).encode()
            ).hexdigest()
            for k, c in self.curves.items()
        }
        record = dict(
            workcell_id=self.cell.id,
            comparison_mode=self.mode,
            bundle_sha256=self.bundle_hash,
            case_file_sha256=self.case_sha256,
            source_plan_run_id=self.bundle["source_run_id"],
            initial_commanded_rad=self.bundle["initial_rad"],
            initial_measured_rad=initial_measured,
            goal_constraint_rad=self.bundle["goal_rad"],
            command_hashes=hashes,
            motion_start_clock_sec=start,
            motion_duration_sec=duration,
            comparison_duration_sec=comparison_duration,
            guard_latch_clock_sec=latch_clock,
            remaining_motion_at_contact_sec=None
            if latch_clock is None
            else start + duration - latch_clock,
            goal_terminal_statuses=statuses,
            contact_injection_requests=0,
            verifier_cancel_requests=0,
            post_stop_drift_rad=drift if self.mode == "collision" else None,
            stationary_peer_drift_rad=self.peer_drift,
            final_positions_rad=self.state(),
            contact_guard=deepcopy(self.contact_guard),
            individual_checks=checks,
            parallel_reference=parallel,
            avoidance_check=avoidance,
        )
        record["movement_rad"] = {
            i.id: max(abs(self.state()[n] - initial_measured[n]) for n in i.joints)
            for i in self.cell.instances
        }
        if self.mode != "collision":
            active = [n for _, t in commands for n in t.joint_names]
            record["max_goal_constraint_error_rad"] = max(
                abs(self.state()[n] - self.bundle["goal_rad"][n]) for n in active
            )
            require(
                record["max_goal_constraint_error_rad"] < 0.01,
                "Case goal was not reached",
            )
            require(
                all(
                    record["movement_rad"][i.id] > 0.8
                    for i in self.cell.instances
                    if self.mode == "avoidance" or i.id == self.mode
                ),
                "Commanded arm did not traverse the case",
            )
        self.save_comparison(stage="complete", **record)
        return dict(record, motion_samples=self.samples)


def finalize_comparison(report, cell, args):
    if args.mode != "collision":
        return finalize_physics(report, cell, args)
    path = report.directory / "physics.json"
    physics = json.loads(path.read_text())
    report.update(physics=physics, physics_report_sha256=file_sha256(path))
    check_stop_physics(
        physics,
        run_id=report.run_id,
        source_sha256=file_sha256(ROOT / "native/step_monitor.hpp"),
        timestep=cell.data["physics"]["timestep"],
    )
    require(
        physics["stop_reason"] == "contact" and physics["pairs"],
        "Expected actual contact",
    )
    prefixes = [i.prefix for i in cell.instances]
    for pair in physics["pairs"]:
        a, b = pair["bodies"]
        require(
            (a.startswith(prefixes[0]) and b.startswith(prefixes[1]))
            or (a.startswith(prefixes[1]) and b.startswith(prefixes[0])),
            "Expected opposite-arm contact",
        )
        require(
            pair["min_distance_m"] > -0.01, "Deep overlap instead of surface approach"
        )
    remaining = (
        report.data["motion_start_clock_sec"]
        + report.data["motion_duration_sec"]
        - physics["stop_time_sec"]
    )
    require(remaining > 1, "Native contact occurred near planned completion")
    report.update(
        physics_checks_passed=True, native_remaining_motion_at_contact_sec=remaining
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--freeze-report",
        type=Path,
        help="Freeze a completed verify_interference report; requires --bundle output",
    )
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--mode", choices=MODES)
    parser.add_argument("--domain", type=int, default=129)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--record-dir", type=Path)
    args = parser.parse_args()
    if args.freeze_report:
        if args.mode or args.gui or args.record_dir:
            parser.error("Freezing does not launch a simulation")
        print(freeze_bundle(args.freeze_report, args.bundle, load_workcell()))
        return 0
    if not args.mode:
        parser.error("--mode is required for execution")
    if args.record_dir and not args.gui:
        parser.error("--record-dir requires --gui")

    def factory(cell, progress):
        if args.record_dir:
            from mujoco_ros2_manipulation.workcells.record_interference import (
                RecordingMixin,
            )

            class RecordedComparison(RecordingMixin, ComparisonVerification):
                def __init__(self):
                    super().__init__(cell, progress, args.bundle, args.mode)
                    self.setup_recording(args.record_dir, "motion.mp4")

                def run(self):
                    return self.recorded_run(super().run)

            return RecordedComparison()
        return ComparisonVerification(cell, progress, args.bundle, args.mode)

    def validator(cell):
        data, _ = load_bundle(args.bundle, cell)
        preflight(cell, data["case_id"])

    args.robot, args.scene, args.stop_mode = "fr5_ur5e", "default", "interrupt"
    args.require_gpu, args.screenshot_dir = args.gui, None
    return run_verification(
        args,
        ROOT,
        rclpy,
        factory,
        Trigger,
        profile_loader=lambda *a, **kw: load_workcell(root=kw["root"]),
        validator=validator,
        command_builder=lambda cell, args: [
            "bash",
            str(ROOT / "run_scene.sh"),
            f"workcell:={cell.id}",
            f"planning_case:={load_bundle(args.bundle,cell)[0]['case_id']}",
            f"gui:={str(args.gui).lower()}",
            f"rviz:={str(args.gui).lower()}",
        ],
        report_prefix="interference_comparison_" + args.mode + "_",
        launch_environment=diagnostic_environment,
        finalize=finalize_comparison,
    )


if __name__ == "__main__":
    raise SystemExit(main())
