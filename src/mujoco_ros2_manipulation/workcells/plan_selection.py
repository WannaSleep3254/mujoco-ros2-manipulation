"""Bounded, recorded planning candidates; reject grazing splines before dispatch."""
from copy import deepcopy
import time

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path

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
        return result


def select_clearance_plan(
    propose, initial, cell, evaluator, policy, moveit_check, save,
    *, max_candidates=MAX_PLAN_CANDIDATES, checkpoint=None, diagnostics=None,
):
    """Return the first valid trajectory, preserving every bounded attempt.

    ``checkpoint`` may raise to stop the search; its exception is re-raised unchanged.
    The caller supplies a shared request budget through that hook. ``diagnostics``
    is updated in place before each save. No action is dispatched by this function.
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
                    else:
                        record["curve"] = curve.record()
                        stage("native")
                        check()
                        # This prefix evidence has no time coordinate; only the complete
                        # native result below represents the fully assessed path.
                        progress = record["native_progress"] = dict(samples=0, minimum=None)
                        native = record["native"] = evaluate_path(
                            [curve], initial, cell, _ObservedEvaluator(evaluator, progress),
                            dt=policy["sample_dt_sec"], reject_below=threshold, checkpoint=check,
                        )
                        check()
                        if not native["complete"]:
                            reject("native_clearance", "Native clearance below planning minimum")
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
