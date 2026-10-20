"""Reject interfering independent plans, then execute a checked both-arm plan."""

import argparse
from copy import deepcopy
import json
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from moveit_msgs.msg import MoveItErrorCodes
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import (
    diagnostic_environment,
    finalize_physics,
)
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.workcells.config import load_workcell, identifier
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.interference import (
    load_case,
    Spline,
    stretch,
    sampled_parallel,
)
from mujoco_ros2_manipulation.workcells.trajectory import partitions
from mujoco_ros2_manipulation.workcells.verify import WorkcellVerification, require


def preflight(cell, case):
    metadata = check_workcell(cell)
    _, data = load_case(cell, case)
    for key in ("start_rad", "goal_rad"):
        for name, value in data[key].items():
            limits = metadata["joint_limits"][name]
            require(
                limits["lower"] <= value <= limits["upper"],
                "Case exceeds joint limit: " + name,
            )


class ParallelPlanRejected(RuntimeError):
    def __init__(self, assessment):
        super().__init__("Refusing interfering parallel plans before dispatch")
        self.assessment = assessment


class InterferenceVerification(WorkcellVerification):
    def __init__(self, cell, progress=None, case="dual_arm_crossing"):
        super().__init__(cell, progress)
        self.case_path, self.case = load_case(cell, case)
        self.case_sha256 = file_sha256(self.case_path)
        self.evidence = {}
        self.initial = None
        self.assessments = {}
        self.executions = []

    def save_evidence(self, stage):
        path = self.launch_log_path.parent / "trajectories.json"
        atomic_json(
            path,
            dict(
                schema_version=1,
                stage=stage,
                case=self.case,
                case_sha256=self.case_sha256,
                initial_rad=self.initial,
                assessments=self.assessments,
                executions=self.executions,
                **self.evidence,
            ),
        )
        return path

    def planned(self, group, target, start):
        answer = self.plan(group, target, start)
        require(
            answer.error_code.val == MoveItErrorCodes.SUCCESS, "MoveIt failed: " + group
        )
        return self.anchored(answer.trajectory.joint_trajectory)

    def assess(self, curves, initial, *, complete=True):
        samples = sampled_parallel(
            curves, initial, complete=complete, dt=self.case["sample_dt_sec"]
        )
        invalid, cross, witness = 0, 0, None
        for stamp, state in samples:
            answer = self.validity_of(state)
            if answer.valid:
                continue
            invalid += 1
            pairs = [(c.contact_body_1, c.contact_body_2) for c in answer.contacts]
            prefixes = [i.prefix for i in self.cell.instances]
            inter_arm = any(
                (a.startswith(prefixes[0]) and b.startswith(prefixes[1]))
                or (a.startswith(prefixes[1]) and b.startswith(prefixes[0]))
                for a, b in pairs
            )
            cross += int(inter_arm)
            if witness is None or (inter_arm and not witness["inter_arm"]):
                witness = dict(
                    time_from_start_sec=stamp,
                    state_rad=state,
                    contacts=pairs,
                    inter_arm=inter_arm,
                )
        return dict(
            samples=len(samples),
            invalid_samples=invalid,
            inter_arm_collision_samples=cross,
            first_collision=witness,
        )

    def split(self, trajectory, start):
        indices = partitions(trajectory.joint_names, self.cell.instances)
        commands = []
        for inst in self.cell.instances:
            arm = JointTrajectory()
            arm.header.stamp = Time(sec=int(start), nanosec=int((start % 1) * 1e9))
            arm.joint_names = inst.joints
            for point in trajectory.points:
                cloned = deepcopy(point)
                for key in ("positions", "velocities", "accelerations", "effort"):
                    values = getattr(point, key)
                    if values:
                        setattr(cloned, key, [values[j] for j in indices[inst.id]])
                arm.points.append(cloned)
            commands.append((inst, arm))
        return commands

    def checked_dispatch(
        self, commands, initial, *, scheduled_start, expect_rejection=False
    ):
        curves = [Spline.from_ros(t) for _, t in commands]
        assessment = self.assess(curves, initial)
        if assessment["invalid_samples"]:
            raise ParallelPlanRejected(assessment)
        require(
            not expect_rejection,
            "Reference plans did not demonstrate interference; no goals dispatched",
        )
        self.dispatch(commands, scheduled_start=scheduled_start)
        return assessment

    def before_avoidance_execution(self):
        pass

    def execute_joint(self, trajectory, initial, label):
        curve = Spline.from_ros(trajectory)
        check = self.assess([curve], initial)
        require(check["invalid_samples"] == 0, "Coordinated spline is colliding")
        if label == "avoidance":
            self.before_avoidance_execution()
        start = self.clock_last + 1.0
        before = self.state()
        # Validate before adding the absolute start time, then send the same splits.
        self.dispatch(self.split(trajectory, start), scheduled_start=start)
        endpoint = dict(zip(curve.names, curve.points[-1]["p"]))
        error = self.settle([endpoint[n] for n in self.joint_names])
        require(
            all(f.result().status == 4 for _, f in self.handles),
            "Both goals must succeed",
        )
        require(
            not self.contact_guard["latched"], "A stop interlock interrupted execution"
        )
        record = dict(
            label=label,
            duration_sec=curve.duration,
            collision_check=check,
            common_start_sec=start,
            goal_terminal_statuses=[f.result().status for _, f in self.handles],
            tracking_error_rad=error,
            movement_rad={
                i.id: max(abs(self.state()[n] - before[n]) for n in i.joints)
                for i in self.cell.instances
            },
        )
        self.executions.append(record)
        return record

    def discover(self):
        self.progress("interference_discovery")
        self.until(
            lambda: self.positions is not None and self.clock_count > 10,
            30,
            "shared states",
        )
        self.ready_guard()
        for client in self.arm_clients.values():
            self.until(client.server_is_ready, 10, "arm server")
        for client in (self.validity, self.planner):
            self.until(client.service_is_ready, 15, "MoveIt service")
        for inst in self.cell.instances:
            service = self.create_client(
                GetParameters, f"/{inst.controller}/get_parameters"
            )
            self.until(service.service_is_ready, 5, "controller parameters")
            request = GetParameters.Request(names=["interpolation_method"])
            response = self.result(
                service.call_async(request), 5, "controller interpolation"
            )
            require(
                response.values[0].string_value == "splines",
                "Controller spline interpolation required",
            )
        service = self.create_client(GetParameters, "/move_group/get_parameters")
        self.until(service.service_is_ready, 5, "MoveIt parameters")
        keys = [
            "ompl.path_tolerance",
            "ompl.resample_dt",
            "ompl.both_arms.longest_valid_segment_fraction",
        ]
        response = self.result(
            service.call_async(GetParameters.Request(names=keys)),
            5,
            "strict planning parameters",
        )
        observed = [p.double_value for p in response.values]
        expected = [
            self.case["planning"][k]
            for k in ("path_tolerance", "resample_dt", "longest_valid_segment_fraction")
        ]
        require(observed == expected, "Strict scenario planning parameters not applied")
        self.settle(self.cell.home)
        for key in ("start_rad", "goal_rad"):
            require(
                self.validity_of(self.case[key]).valid,
                "Case endpoint is colliding: " + key,
            )
    def run(self):
        self.discover()
        self.progress("move_to_common_start")
        setup = self.planned("both_arms", self.case["start_rad"], self.commanded)
        self.evidence["setup"] = Spline.from_ros(setup).record()
        self.save_evidence("setup_plan")
        self.execute_joint(setup, dict(self.commanded), "setup")
        initial = self.initial = dict(self.commanded)
        measured_initial = self.state()
        goal = self.case["goal_rad"]
        independent = []
        self.progress("independent_arm_plans")
        for inst in self.cell.instances:
            trajectory = self.planned(
                inst.group, {n: goal[n] for n in inst.joints}, initial
            )
            independent.append((inst, trajectory))
        common_duration = max(Spline.from_ros(t).duration for _, t in independent)
        independent = [(inst, stretch(t, common_duration)) for inst, t in independent]
        single_checks = {}
        for inst, trajectory in independent:
            curve = Spline.from_ros(trajectory)
            check = self.assess([curve], initial, complete=False)
            require(
                check["invalid_samples"] == 0,
                "Independent arm plan is not safe with peer at start",
            )
            single_checks[inst.id] = check
            self.evidence["independent_" + inst.id] = curve.record()
            self.assessments[inst.id] = check
            self.save_evidence("independent_" + inst.id)
        self.progress("parallel_interference_rejected_before_dispatch")
        before_dispatch = self.dispatched
        try:
            self.checked_dispatch(
                independent,
                initial,
                scheduled_start=self.clock_last + 1,
                expect_rejection=True,
            )
        except ParallelPlanRejected as error:
            parallel = error.assessment
        else:
            raise RuntimeError("Interfering plans passed the dispatch gate")
        self.assessments["parallel"] = parallel
        self.save_evidence("parallel_rejected")
        require(
            parallel["inter_arm_collision_samples"] > 0,
            "Reference plans did not demonstrate cross-arm interference",
        )
        require(self.dispatched == before_dispatch, "Unsafe arm goals were dispatched")
        self.progress("coordinated_avoidance_plan")
        coordinated = self.planned("both_arms", goal, initial)
        self.evidence["coordinated"] = Spline.from_ros(coordinated).record()
        self.save_evidence("coordinated_plan")
        execution = self.execute_joint(coordinated, initial, "avoidance")
        require(
            all(v > 0.8 for v in execution["movement_rad"].values()),
            "Both arms did not traverse the crossing case",
        )
        require(
            max(abs(self.state()[n] - goal[n]) for n in self.joint_names) < 0.01,
            "Case goal was not reached",
        )
        require(
            file_sha256(self.case_path) == self.case_sha256,
            "Case changed during verification",
        )
        path = self.save_evidence("complete")
        return dict(
            workcell_id=self.cell.id,
            interference_case=self.case["id"],
            case_file_sha256=self.case_sha256,
            initial_commanded_rad=initial,
            initial_measured_rad=measured_initial,
            goal_constraint_rad=goal,
            independent_arm_checks=single_checks,
            parallel_reference=parallel,
            reference_duration_sec=common_duration,
            unsafe_goals_dispatched=0,
            parallel_dispatch_gate_passed=True,
            coordinated_execution=execution,
            setup_execution=self.executions[0],
            planning_wall_timings=self.planning_timings,
            interpolation="controller splines: linear/cubic/quintic according to available derivatives",
            planning_parameters=self.case["planning"],
            interpolation_max_dt_sec=self.case["sample_dt_sec"],
            interpolation_max_step_rad=0.025,
            trajectories_path=str(path),
            trajectories_sha256=file_sha256(path),
            contact_guard=deepcopy(self.contact_guard),
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="dual_arm_crossing")
    parser.add_argument("--domain", type=int, default=126)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--screenshot-dir", type=Path)
    parser.add_argument("--record-dir", type=Path)
    args = parser.parse_args()
    try:
        identifier(args.case)
    except ValueError as error:
        parser.error(str(error))
    if args.record_dir and not args.gui:
        parser.error("--record-dir requires --gui")
    factory = lambda cell, progress: InterferenceVerification(cell, progress, args.case)
    if args.record_dir:
        from mujoco_ros2_manipulation.workcells.record_interference import (
            RecordedInterference,
        )

        factory = lambda cell, progress: RecordedInterference(
            cell, progress, args.case, args.record_dir
        )
    args.robot, args.scene, args.stop_mode = "fr5_ur5e", "default", "interrupt"
    args.require_gpu = args.gui
    return run_verification(
        args,
        ROOT,
        rclpy,
        factory,
        Trigger,
        profile_loader=lambda *a, **kw: load_workcell(root=kw["root"]),
        validator=lambda cell: preflight(cell, args.case),
        command_builder=lambda cell, args: [
            "bash",
            str(ROOT / "run_scene.sh"),
            f"workcell:={cell.id}",
            f"planning_case:={args.case}",
            f"gui:={str(args.gui).lower()}",
            f"rviz:={str(args.gui).lower()}",
        ],
        report_prefix="interference_" + args.case + "_",
        launch_environment=diagnostic_environment,
        finalize=finalize_physics,
    )


if __name__ == "__main__":
    raise SystemExit(main())
