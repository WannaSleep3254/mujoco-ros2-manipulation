"""Freeze one verified plan bundle and reject mismatched comparison evidence."""
import json
from pathlib import Path

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.interference import (
    Spline,
    load_case,
    sampled_parallel,
)
from mujoco_ros2_manipulation.verification.reports import atomic_json

MODES = ("left", "right", "collision", "avoidance")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load_bundle(path, cell):
    data = json.loads(Path(path).read_text())
    require(data["schema_version"] == 1, "Unsupported comparison bundle")
    case_path, case = load_case(cell, data["case_id"])
    require(
        data["case_sha256"] == file_sha256(case_path), "Case changed since planning"
    )
    require(
        data["model_manifest_sha256"] == file_sha256(cell.output / "source.json"),
        "Model changed since planning",
    )
    names = set(cell.joints)
    initial = data["initial_rad"]
    require(
        set(initial) == names and set(data["goal_rad"]) == names,
        "Incomplete bundle state",
    )
    require(data["goal_rad"] == case["goal_rad"], "Bundle target differs from case")
    require(
        max(abs(initial[n] - case["start_rad"][n]) for n in names) <= 0.0031,
        "Bundle start differs from case",
    )
    home = dict(zip(cell.joints, cell.home))
    curves = {key: Spline(**value) for key, value in data["curves"].items()}
    require(
        set(curves) == {"setup", "left", "right", "avoidance"},
        "Incomplete bundle curves",
    )
    for key in ("setup", "avoidance"):
        require(set(curves[key].names) == names, "Incomplete joint plan")
    for inst in cell.instances:
        require(
            tuple(inst.joints) == curves[inst.id].names,
            "Wrong independent joint ownership",
        )
    for key, curve in curves.items():
        start = home if key == "setup" else initial
        end = initial if key == "setup" else data["goal_rad"]
        require(
            max(
                abs(curve.points[0]["p"][j] - start[n])
                for j, n in enumerate(curve.names)
            )
            < 1e-9,
            "Mismatched exact commanded start",
        )
        tolerance = 1e-9 if key == "setup" else 0.0031
        require(
            max(
                abs(curve.points[-1]["p"][j] - end[n])
                for j, n in enumerate(curve.names)
            )
            <= tolerance,
            "Mismatched endpoint",
        )
        sampled_parallel([curve], start, complete=key in ("setup", "avoidance"))
    sampled_parallel([curves["left"], curves["right"]], initial)
    return data, curves


def freeze_bundle(report_path, output, cell):
    path = Path(report_path).resolve()
    report = json.loads(path.read_text())
    require(
        report["status"] == "passed"
        and report["motion_checks_passed"]
        and report["physics_checks_passed"]
        and report["shutdown_clean"],
        "Only completed successful verification can be frozen",
    )
    source = Path(report["trajectories_path"])
    require(
        file_sha256(source) == report["trajectories_sha256"],
        "Trajectory source hash mismatch",
    )
    evidence = json.loads(source.read_text())
    require(evidence["stage"] == "complete", "Incomplete trajectory evidence")
    data = dict(
        schema_version=1,
        case_id=report["interference_case"],
        case_sha256=report["case_file_sha256"],
        model_manifest_sha256=report["model_manifest_sha256"],
        source_run_id=report["run_id"],
        source_report_sha256=file_sha256(path),
        source_trajectories_sha256=file_sha256(source),
        initial_rad=report["initial_commanded_rad"],
        goal_rad=report["goal_constraint_rad"],
        curves={
            key: evidence[value]
            for key, value in dict(
                setup="setup",
                left="independent_left",
                right="independent_right",
                avoidance="coordinated",
            ).items()
        },
    )
    # Spline records use 'names', matching the constructor. Validate before publishing.
    output = Path(output).resolve()
    require(not output.exists(), "Bundle output already exists")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".candidate.json")
    try:
        atomic_json(temporary, data)
        load_bundle(temporary, cell)
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    return output


def compare_reports(paths):
    reports = [json.loads(Path(p).read_text()) for p in paths]
    require(
        len(reports) == 4 and {r["comparison_mode"] for r in reports} == set(MODES),
        "Exactly four fresh comparison runs are required",
    )
    require(
        len({r["run_id"] for r in reports}) == 4, "Runs must have separate launches"
    )
    for key in (
        "bundle_sha256",
        "model_manifest_sha256",
        "case_file_sha256",
        "initial_commanded_rad",
        "goal_constraint_rad",
        "command_hashes",
        "comparison_duration_sec",
    ):
        require(
            all(r[key] == reports[0][key] for r in reports),
            "Unmatched comparison: " + key,
        )
    for report in reports:
        require(
            report["status"] == "passed"
            and report["motion_checks_passed"]
            and report["physics_checks_passed"]
            and report["shutdown_clean"]
            and report["process_group_gone"],
            "Failed comparison run",
        )
        require(
            report["verifier_cancel_requests"] == 0
            and report["contact_injection_requests"] == 0,
            "Comparison must use genuine trajectory contact and autonomous stop",
        )
        physics = report["physics"]
        if report["comparison_mode"] == "collision":
            require(
                physics["stop_reason"] == "contact"
                and physics["stop_latched"]
                and physics["contacts"] > 0
                and report["remaining_motion_at_contact_sec"] > 1
                and report["post_stop_drift_rad"] == 0
                and report["goal_terminal_statuses"] == [5, 5],
                "Contact control did not stop both arms",
            )
        else:
            require(
                physics["contacts"] == 0
                and not physics["stop_latched"]
                and report["goal_terminal_statuses"]
                == ([4, 4] if report["comparison_mode"] == "avoidance" else [4]),
                "Normal/avoidance motion failed",
            )
    return dict(
        schema_version=1,
        matched=True,
        bundle_sha256=reports[0]["bundle_sha256"],
        categories={
            "1_normal": ["left", "right"],
            "2_without_avoidance": "collision",
            "3_coordinated_avoidance": "avoidance",
        },
        runs={
            r["comparison_mode"]: dict(
                run_id=r["run_id"],
                report_sha256=file_sha256(p),
                contacts=r["physics"]["contacts"],
                duration_sec=r["motion_duration_sec"],
                goal_terminal_statuses=r["goal_terminal_statuses"],
            )
            for p, r in zip(paths, reports)
        },
    )
