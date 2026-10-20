"""Validate matched original-detour / timed execution reports and quality changes."""
import json
from pathlib import Path
from mujoco_ros2_manipulation.workcells.interference import finite
from mujoco_ros2_manipulation.model_build.integrity import file_sha256


def compare_timing(paths, plan_path):
    if len(paths) != 2:
        raise ValueError("Exactly two reports required")
    reports = [json.loads(Path(p).read_text()) for p in paths]
    if {r["timing_mode"] for r in reports} != {"baseline", "timing"} or len(
        {r["run_id"] for r in reports}
    ) != 2:
        raise ValueError("Expected two independent baseline/timing launches")
    for key in (
        "bundle_sha256",
        "timing_plan_sha256",
        "model_manifest_sha256",
        "case_file_sha256",
        "policy",
        "policy_sha256",
        "evaluator",
        "initial_commanded_rad",
        "goal_constraint_rad",
        "comparison_duration_sec",
    ):
        if reports[0][key] != reports[1][key]:
            raise ValueError("Unmatched timing comparison: " + key)
    plan = json.loads(Path(plan_path).read_text())
    if (
        reports[0]["timing_plan_sha256"] != file_sha256(plan_path)
        or reports[0]["bundle_sha256"] != plan["bundle_sha256"]
    ):
        raise ValueError("Timing plan hash mismatch")
    for r in reports:
        if (
            r["status"] != "passed"
            or not r["motion_checks_passed"]
            or not r["physics_checks_passed"]
            or not r["shutdown_clean"]
            or not r["process_group_gone"]
            or r["goal_terminal_statuses"] != [4, 4]
            or r["physics"]["contacts"] != 0
            or r["physics"]["stop_latched"]
            or r["contact_injection_requests"]
            or r["verifier_cancel_requests"]
            or r["max_goal_error_rad"] >= 0.01
        ):
            raise ValueError("Failed timing comparison run")
    for r in reports:
        for name in ("planned", "observed"):
            m = r[name]
            values = [
                m["minimum"]["distance_m"],
                m["joint_total_variation_rad"],
                m["duration_sec"],
                *m["tip_frame_path_m"].values(),
            ]
            if (
                not m["complete"]
                or m["samples"] < 2
                or not finite(values)
                or m["joint_total_variation_rad"] <= 0
                or m["duration_sec"] <= 0
            ):
                raise ValueError("Invalid/nonfinite quality metrics")
    by_mode = {r["timing_mode"]: r for r in reports}
    before = by_mode["baseline"]
    after = by_mode["timing"]
    policy = plan["policy"]
    if (
        after["planned"]["minimum"]["distance_m"]
        < policy["min_inter_arm_distance_m"] + policy["tracking_reserve_m"]
        or after["observed"]["minimum"]["distance_m"]
        < policy["min_inter_arm_distance_m"]
    ):
        raise ValueError("Timing clearance below policy")
    if (
        after["planned"]["joint_total_variation_rad"]
        > plan["reference"]["joint_total_variation_rad"] * 1.001
    ):
        raise ValueError("Timing changed nominal geometry")
    if (
        not after["planned"]["joint_total_variation_rad"]
        < before["planned"]["joint_total_variation_rad"]
    ):
        raise ValueError("No spatial travel reduction against original detour")
    for arm, length in plan["reference"]["tip_frame_path_m"].items():
        if after["planned"]["tip_frame_path_m"][arm] > length * 1.003 + 1e-6:
            raise ValueError("Tip-frame path geometry changed")
    return dict(
        schema_version=1,
        matched=True,
        bundle_sha256=plan["bundle_sha256"],
        timing_plan_sha256=file_sha256(plan_path),
        departure_delay_sec=plan["delay_sec"],
        delayed_arm=plan["delayed_arm"],
        policy=policy,
        reduction_fraction=dict(
            joint_travel=1
            - after["planned"]["joint_total_variation_rad"]
            / before["planned"]["joint_total_variation_rad"],
            duration=1 - after["motion_duration_sec"] / before["motion_duration_sec"],
        ),
        runs={
            r["timing_mode"]: dict(
                run_id=r["run_id"],
                report_sha256=file_sha256(p),
                planned=r["planned"],
                observed=r["observed"],
                duration_sec=r["motion_duration_sec"],
                goal_terminal_statuses=r["goal_terminal_statuses"],
                physics_steps=r["physics"]["steps"],
            )
            for p, r in zip(paths, reports)
        },
    )
