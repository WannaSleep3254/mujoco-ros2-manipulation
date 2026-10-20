"""Bounded, recorded planning candidates; reject grazing splines before dispatch."""
from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path

MAX_PLAN_CANDIDATES = 12


def select_clearance_plan(
    propose, initial, cell, evaluator, policy, moveit_check, save
):
    """No action dispatch here. Every candidate, including rejection, is saved first."""
    threshold = policy["min_inter_arm_distance_m"] + policy["tracking_reserve_m"]
    candidates = []
    for attempt in range(1, MAX_PLAN_CANDIDATES + 1):
        code, trajectory = propose()
        record = dict(attempt=attempt, moveit_error_code=code, accepted=False)
        candidates.append(record)
        try:
            if code == 1:
                try:
                    curve = Spline.from_ros(trajectory)
                except ValueError as error:
                    record["rejection"] = str(error)
                else:
                    record["curve"] = curve.record()
                    native = record["native"] = evaluate_path(
                        [curve],
                        initial,
                        cell,
                        evaluator,
                        dt=policy["sample_dt_sec"],
                        reject_below=threshold,
                    )
                    if native["complete"]:
                        check = record["moveit"] = moveit_check(curve)
                        record["accepted"] = check["invalid_samples"] == 0
                    else:
                        record["rejection"] = "Native clearance below planning minimum"
        except Exception as error:
            record["evaluation_error"] = dict(
                type=type(error).__name__, message=str(error)
            )
            raise
        finally:
            save(candidates)
        if record["accepted"]:
            return trajectory
    raise RuntimeError(
        f"No plan satisfies native clearance and MoveIt within {MAX_PLAN_CANDIDATES} candidates; no candidate dispatched"
    )
