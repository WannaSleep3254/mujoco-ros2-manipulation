"""Compare original both-arm detour with geometry-preserving departure timing."""
import argparse
from copy import deepcopy
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import (
    diagnostic_environment,
    finalize_physics,
)
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.interference_comparison import load_bundle
from mujoco_ros2_manipulation.workcells.verify_interference import (
    InterferenceVerification,
    preflight,
)
from mujoco_ros2_manipulation.workcells.verify_interference_comparison import to_ros
from mujoco_ros2_manipulation.workcells.verify import require, seconds
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator
from mujoco_ros2_manipulation.workcells.path_quality import (
    evaluate_path,
    evaluate_states,
    sample_schedule,
)
from mujoco_ros2_manipulation.workcells.timing import load_timing, prepare_timing


class TimingVerification(InterferenceVerification):
    def __init__(self, cell, progress, bundle, plan, mode):
        self.bundle_path, self.plan_path = Path(bundle).resolve(), Path(plan).resolve()
        self.bundle_hash, self.plan_hash = file_sha256(bundle), file_sha256(plan)
        self.bundle, self.original = load_bundle(bundle, cell)
        self.evaluator = DistanceEvaluator(cell)
        try:
            self.timing, self.scheduled = load_timing(
                plan, bundle, cell, self.evaluator
            )
        except BaseException:
            self.evaluator.close()
            raise
        self.mode = mode
        self.observations = []
        self.tracking = False
        super().__init__(cell, progress, self.bundle["case_id"])

    def destroy_node(self):
        try:
            self.evaluator.close()
        finally:
            result = super().destroy_node()
        return result

    def joints(self, message):
        super().joints(message)
        if self.tracking and self.positions is not None:
            stamp = seconds(message.header.stamp)
            if not self.observations or stamp > self.observations[-1]["clock_sec"]:
                self.observations.append(dict(clock_sec=stamp, state_rad=self.state()))

    def assess_schedule(self, curves):
        count = 0
        for _, state in sample_schedule(
            curves, self.bundle["initial_rad"], self.timing["policy"]["sample_dt_sec"]
        ):
            require(self.validity_of(state).valid, "Scheduled path rejected by MoveIt")
            count += 1
        return count

    def run(self):
        self.discover()
        self.progress("timing_prepare_start")
        self.execute_joint(
            to_ros(self.original["setup"]), dict(self.commanded), "setup"
        )
        require(
            self.commanded == self.bundle["initial_rad"],
            "Mismatched common starting command",
        )
        curves = (
            [self.original["avoidance"]] if self.mode == "baseline" else self.scheduled
        )
        self.progress("timing_distance_preflight")
        planned = evaluate_path(
            curves,
            self.bundle["initial_rad"],
            self.cell,
            self.evaluator,
            dt=self.timing["policy"]["sample_dt_sec"],
        )
        if self.mode == "timing":
            threshold = (
                self.timing["policy"]["min_inter_arm_distance_m"]
                + self.timing["policy"]["tracking_reserve_m"]
            )
            require(
                planned["minimum"]["distance_m"] >= threshold,
                "Scheduled clearance insufficient before dispatch",
            )
        count = self.assess_schedule(curves)
        self.before_avoidance_execution()
        self.tracking = True
        start = self.clock_last + 1
        commands = (
            self.split(to_ros(curves[0]), start)
            if self.mode == "baseline"
            else [
                (inst, to_ros(curve, start))
                for inst, curve in zip(self.cell.instances, curves)
            ]
        )
        self.progress("timing_execute_" + self.mode)
        self.dispatch(commands, scheduled_start=start)
        goal = {
            n: p
            for _, t in commands
            for n, p in zip(t.joint_names, t.points[-1].positions)
        }
        self.settle([goal[n] for n in self.joint_names])
        duration = max(c.duration for c in curves)
        # Equal comparison horizon includes the old detour even when timing finishes early.
        horizon = (
            max(
                self.original["avoidance"].duration,
                max(c.duration for c in self.scheduled),
            )
            + 1
        )
        self.progress("timing_observe_tail")
        self.until(
            lambda: self.clock_last >= start + horizon,
            horizon + 5,
            "comparison holding tail",
        )
        self.tracking = False
        observed = evaluate_states(
            (
                (s["clock_sec"] - start, s["state_rad"])
                for s in self.observations
                if s["clock_sec"] >= start
            ),
            self.cell,
            self.evaluator,
        )
        require(not self.contact_guard["latched"], "Unexpected contact/clock stop")
        require(
            all(f.result().status == 4 for _, f in self.handles),
            "Both controllers must succeed",
        )
        error = max(
            abs(self.state()[n] - self.bundle["goal_rad"][n]) for n in self.joint_names
        )
        require(error < 0.01, "Case goal not reached")
        if self.mode == "timing":
            require(
                observed["minimum"]["distance_m"]
                >= self.timing["policy"]["min_inter_arm_distance_m"],
                "Measured joint-state clearance below policy",
            )
        require(
            file_sha256(self.plan_path) == self.plan_hash
            and file_sha256(self.bundle_path) == self.bundle_hash,
            "Inputs changed during execution",
        )
        check_workcell(self.cell)
        path = self.launch_log_path.parent / "observations.json"
        atomic_json(path, dict(samples=self.observations, motion_start_clock_sec=start))
        return dict(
            workcell_id=self.cell.id,
            timing_mode=self.mode,
            case_file_sha256=self.case_sha256,
            bundle_sha256=self.bundle_hash,
            timing_plan_sha256=self.plan_hash,
            policy=self.timing["policy"],
            policy_sha256=self.timing["policy_sha256"],
            evaluator=self.evaluator.metadata(),
            initial_commanded_rad=self.bundle["initial_rad"],
            goal_constraint_rad=self.bundle["goal_rad"],
            planned=planned,
            observed=observed,
            moveit_samples=count,
            motion_start_clock_sec=start,
            motion_duration_sec=duration,
            comparison_duration_sec=horizon,
            delayed_arm=self.timing["delayed_arm"],
            departure_delay_sec=self.timing["delay_sec"],
            goal_terminal_statuses=[f.result().status for _, f in self.handles],
            max_goal_error_rad=error,
            observations_path=str(path),
            observations_sha256=file_sha256(path),
            verifier_cancel_requests=0,
            contact_injection_requests=0,
            contact_guard=deepcopy(self.contact_guard),
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--mode", choices=["baseline", "timing"])
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--record-dir", type=Path)
    parser.add_argument("--domain", type=int, default=132)
    args = parser.parse_args()
    if args.prepare:
        if args.mode or args.gui or args.record_dir:
            parser.error("--prepare does not launch a simulation")
        cell = load_workcell()
        check_workcell(cell)
        with DistanceEvaluator(cell) as evaluator:
            print(prepare_timing(args.bundle, args.plan, cell, evaluator))
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

            class RecordedTiming(RecordingMixin, TimingVerification):
                def __init__(self):
                    super().__init__(cell, progress, args.bundle, args.plan, args.mode)
                    self.setup_recording(args.record_dir, "motion.mp4")

                def run(self):
                    return self.recorded_run(super().run)

            return RecordedTiming()
        return TimingVerification(cell, progress, args.bundle, args.plan, args.mode)

    def validator(cell):
        b, _ = load_bundle(args.bundle, cell)
        preflight(cell, b["case_id"])
        with DistanceEvaluator(cell) as evaluator:
            load_timing(args.plan, args.bundle, cell, evaluator)

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
        report_prefix="timing_" + args.mode + "_",
        launch_environment=diagnostic_environment,
        finalize=finalize_physics,
    )


if __name__ == "__main__":
    raise SystemExit(main())
