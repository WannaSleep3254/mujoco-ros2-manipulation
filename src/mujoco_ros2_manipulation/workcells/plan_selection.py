"""Bounded, recorded planning candidates; reject grazing splines before dispatch."""
from copy import deepcopy
import math
import time

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path
from mujoco_ros2_manipulation.workcells.environment_clearance import required_environment_distance

MAX_PLAN_CANDIDATES = 12


class PlanCandidateRejected(ValueError):
    """A proposal adapter explicitly reports an unusable assembled trajectory."""

    def __init__(self, message, *, kind="invalid_trajectory"):
        if kind != "invalid_trajectory":
            raise ValueError("PlanCandidateRejected kind must be invalid_trajectory")
        self.kind = kind
        super().__init__(message)


class PlanCandidatesExhausted(RuntimeError):
    """The bounded search ended without a candidate eligible for dispatch."""


def invalid_trajectory_diagnostic(trajectory):
    """Record bounded time/layout evidence without serializing position arrays.

    This runs only after spline validation rejected a ROS trajectory. Nonfinite
    numbers are labelled explicitly so writing JSON cannot replace the rejection
    with a serialization error. Truncation is explicit; this is diagnostic data,
    never a second acceptance gate.
    """
    def number(value):
        if type(value) in (int, float):
            try:
                if math.isfinite(value):
                    return value
            except OverflowError:
                pass
            return dict(nonfinite=str(value))
        return dict(unsupported_type=type(value).__name__)

    def stamp(point):
        value = point.time_from_start
        return dict(sec=number(value.sec), nanosec=number(value.nanosec),
                    seconds=number(value.sec + value.nanosec * 1e-9))

    points = trajectory.points
    count = len(points)
    captured = points[:2000]
    times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
             for point in captured]
    nonfinite = [index for index, value in enumerate(times)
                 if type(value) not in (int, float) or not math.isfinite(value)]
    nonincreasing = [index for index in range(1, len(times))
                     if times[index] <= times[index - 1]]
    issues = []
    if not 2 <= count <= 2000:
        issues.append("point_count_outside_2_2000")
    if nonfinite:
        issues.append("nonfinite_time")
    if times and times[0] != 0:
        issues.append("nonzero_start")
    if nonincreasing:
        issues.append("nonincreasing_time")
    end = points[-1].time_from_start if points else None
    end_sec = None if end is None else end.sec + end.nanosec * 1e-9
    if end_sec is not None and math.isfinite(end_sec):
        if end_sec <= 0:
            issues.append("nonpositive_end")
        if end_sec > 35:
            issues.append("end_exceeds_35s")
    return dict(
        schema_version=1, joint_count=len(trajectory.joint_names), point_count=count,
        captured_point_count=len(captured), truncated=count > len(captured),
        timing_issues=issues, nonfinite_time_indices=nonfinite,
        nonincreasing_time_indices=nonincreasing,
        first_time=None if not points else stamp(points[0]),
        last_time=None if not points else stamp(points[-1]),
        points=[dict(index=index, time_from_start=stamp(point),
                     positions_count=len(point.positions), velocities_count=len(point.velocities),
                     accelerations_count=len(point.accelerations))
                for index, point in enumerate(captured)],
    )


class _ObservedEvaluator:
    """Keep completed distance queries when sampling is interrupted."""

    def __init__(self, evaluator, progress):
        self.evaluator = evaluator
        self.progress = progress

    def at(self, state):
        result = self.evaluator.at(state)
        self.progress["samples"] += 1
        minimum = self.progress["minimum"]
        if minimum is None or result["distance_m"] < minimum["distance_m"]:
            self.progress["minimum"] = dict(state_rad=dict(state), **deepcopy(result))
        environment = result.get("environment")
        if environment is not None:
            previous = self.progress.get("environment_minimum")
            if previous is None or environment["distance_m"] < previous["distance_m"]:
                self.progress["environment_minimum"] = dict(state_rad=dict(state), **deepcopy(environment))
        return result


def select_clearance_plan(
    propose, initial, cell, evaluator, policy, moveit_check, save,
    *, max_candidates=MAX_PLAN_CANDIDATES, checkpoint=None, diagnostics=None,
    validate_curve=None, environment_policy=None,
):
    """Return the first valid trajectory, preserving every bounded attempt.

    ``checkpoint`` may raise to stop the search; its exception is re-raised unchanged.
    The caller supplies a shared request budget through that hook. ``diagnostics``
    is updated in place before each save. No action is dispatched by this function.
    ``validate_curve`` checks adapter-specific joint/endpoint contracts only after
    spline validation. Its exceptions fail the request; they are never retried.
    """
    if (
        isinstance(max_candidates, bool)
        or not isinstance(max_candidates, int)
        or not 1 <= max_candidates <= MAX_PLAN_CANDIDATES
    ):
        raise ValueError(f"max_candidates must be an integer from 1 to {MAX_PLAN_CANDIDATES}")
    if diagnostics is not None and not isinstance(diagnostics, dict):
        raise TypeError("diagnostics must be a dict")
    diagnostics = {} if diagnostics is None else diagnostics
    threshold = policy["min_inter_arm_distance_m"] + policy["tracking_reserve_m"]
    environment_options = {} if environment_policy is None else dict(
        environment_reject_below=required_environment_distance(environment_policy))
    candidates = []
    started = time.monotonic()
    diagnostics.update(status="running", stage="proposal", max_candidates=max_candidates,
                       attempted=0, selected_attempt=None, elapsed_wall_sec=0.0,
                       candidates=candidates)
    diagnostics.pop("error", None)
    diagnostics.pop("save_error", None)
    for attempt in range(1, max_candidates + 1):
        candidate_started = time.monotonic()
        record = dict(attempt=attempt, accepted=False, status="running", stage="proposal",
                      elapsed_wall_sec=0.0)
        candidates.append(record)
        diagnostics["attempted"] = attempt
        checkpoint_failed = False
        failure = None

        def stage(name):
            record["stage"] = diagnostics["stage"] = name

        def check():
            nonlocal checkpoint_failed
            if checkpoint is not None:
                try:
                    checkpoint()
                except BaseException:
                    checkpoint_failed = True
                    raise

        def reject(kind, reason):
            record.update(status="rejected", rejection_kind=kind, rejection=reason)

        try:
            check()
            try:
                code, trajectory = propose()
            except PlanCandidateRejected as error:
                reject(error.kind, str(error))
                check()
            else:
                record["moveit_error_code"] = code
                check()
                if code != 1:
                    reject("planner_failed", f"MoveIt plan failed: {code}")
                else:
                    stage("trajectory")
                    try:
                        curve = Spline.from_ros(trajectory)
                    except ValueError as error:
                        reject("invalid_trajectory", str(error))
                        try:
                            record["trajectory_diagnostic"] = invalid_trajectory_diagnostic(trajectory)
                        except Exception as diagnostic_error:
                            # Diagnostic extraction must not hide the spline rejection.
                            record["trajectory_diagnostic_error"] = dict(
                                type=type(diagnostic_error).__name__, message=str(diagnostic_error))
                    else:
                        record["curve"] = curve.record()
                        if validate_curve is not None:
                            validate_curve(curve)
                        stage("native")
                        check()
                        # This prefix evidence has no time coordinate; only the complete
                        # native result below represents the fully assessed path.
                        progress = record["native_progress"] = dict(samples=0, minimum=None)
                        native = record["native"] = evaluate_path(
                            [curve], initial, cell, _ObservedEvaluator(evaluator, progress),
                            dt=policy["sample_dt_sec"], reject_below=threshold, checkpoint=check,
                            **environment_options,
                        )
                        check()
                        if not native["complete"]:
                            kind = native.get("rejection_kind", "native_clearance")
                            reject(kind, "Environment clearance below planning minimum" if kind == "environment_clearance"
                                   else "Native inter-arm clearance below planning minimum")
                        else:
                            stage("moveit")
                            check()
                            result = record["moveit"] = moveit_check(curve)
                            check()
                            if result["invalid_samples"] != 0:
                                reject("moveit_invalid", "MoveIt rejected interpolated schedule")
                            else:
                                stage("selection")
                                check()
                                record.update(accepted=True, status="selected")
                                diagnostics.update(status="selected", selected_attempt=attempt)
            if not record["accepted"] and attempt == max_candidates:
                diagnostics.update(status="exhausted", stage="complete")
        except BaseException as error:
            failure = error
            status = "interrupted" if checkpoint_failed or isinstance(error, KeyboardInterrupt) else "failed"
            detail = dict(type=type(error).__name__, message=str(error))
            record.update(status=status, error=detail, evaluation_error=detail)
            diagnostics.update(status=status, error=detail)
            raise
        finally:
            record["elapsed_wall_sec"] = time.monotonic() - candidate_started
            diagnostics["elapsed_wall_sec"] = time.monotonic() - started
            try:
                save(candidates)
            except BaseException as error:
                detail = dict(type=type(error).__name__, message=str(error))
                if failure is not None:
                    # A persistence failure must not replace a cancel/budget exception.
                    record["save_error"] = detail
                    diagnostics["save_error"] = detail
                else:
                    record.update(accepted=False, status="failed", error=detail)
                    diagnostics.update(status="failed", selected_attempt=None, error=detail)
                    raise
        if record["accepted"]:
            return trajectory
    raise PlanCandidatesExhausted(
        f"No plan satisfies native clearance and MoveIt within {max_candidates} candidates; no candidate dispatched"
    )
