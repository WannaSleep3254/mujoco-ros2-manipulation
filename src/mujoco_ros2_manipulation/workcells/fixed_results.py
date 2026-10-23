"""Pure fixed-suite evidence checks and complete-catalog outcome accounting.

Inputs are original bytes, never a reconstructed/pretty-printed substitute. Local
paths and publication privacy are the outer runner/public adapter's responsibility.
This module does not open files, call ROS, evaluate geometry or select cases.
"""
from copy import deepcopy
import hashlib
import json
import math
import re
from statistics import median

from mujoco_ros2_manipulation.workcells.fixed_suite import planned_rows, suite_hash

POLICY_NUMBERS = dict(min_inter_arm_distance_m=.02, tracking_reserve_m=.01,
                      sample_dt_sec=.01, delay_step_sec=.1, max_delay_sec=4.)
PLANNING_FAILURES = {"candidate_exhausted", "budget_exhausted", "planner_failed"}
STATUSES = {"completed", "failed", "not_run"}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value, *, nullable=False, nonnegative=False):
    if value is None and nullable:
        return None
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    _require(valid and (not nonnegative or value >= 0), "Expected finite numeric result value")
    return value


def _integer(value, *, minimum=0):
    _require(type(value) is int and value >= minimum, "Expected bounded integer result value")
    return value


def _digest(value):
    _require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{64}", value), "Invalid evidence SHA256")
    return value


def _json_value(value):
    if isinstance(value, dict):
        _require(all(isinstance(key, str) for key in value), "Expected string JSON keys")
        for item in value.values():
            _json_value(item)
    elif isinstance(value, list):
        for item in value:
            _json_value(item)
    elif value is None or type(value) in (str, bool, int):
        if type(value) is int:
            _finite(value)
    else:
        _finite(value)


def _pairs(pairs):
    result = {}
    for key, value in pairs:
        _require(key not in result, "Duplicate evidence JSON key")
        result[key] = value
    return result


def _load(raw):
    if raw is None:
        return None, None
    _require(type(raw) is bytes, "Evidence must be original JSON bytes or null")
    try:
        value = json.loads(raw, object_pairs_hook=_pairs)
    except (UnicodeError, ValueError) as error:
        raise ValueError("Invalid original evidence JSON: " + str(error)) from error
    _require(isinstance(value, dict), "Expected evidence JSON object")
    _json_value(value)
    return value, hashlib.sha256(raw).hexdigest()


def _state(value, names):
    _require(isinstance(value, dict) and set(value) == set(names), "Fixed state joint identity mismatch")
    for position in value.values():
        _finite(position)
    return value


def _base(suite, row):
    _json_value(suite)
    _require(suite.get("policy") == POLICY_NUMBERS, "Fixed suite lacks the exact shared evaluation policy")
    cases = {case["case_id"]: case for case in suite["cases"]}
    _require(isinstance(row, dict) and row.get("case_id") in cases, "Unknown fixed row case")
    case = cases[row["case_id"]]
    _integer(row.get("repeat"), minimum=1)
    _integer(row.get("index"))
    _require(type(row.get("selected")) is bool, "Fixed row selection must be boolean")
    catalog = planned_rows(suite, row["repeat"], [case["case_id"]] if row["selected"] else [])
    _require(row["index"] < len(catalog), "Fixed row index exceeds its catalog")
    expected = catalog[row["index"]]
    for key in ("case_id", "variant", "max_plan_candidates", "repeat", "index", "selected"):
        _require(row.get(key) == expected[key] and type(row.get(key)) is type(expected[key]), "Fixed row identity/cap/order mismatch")
    identifier = f"r{row['repeat']:02d}-{case['case_id']}-{row['variant']}"
    _require(row.get("row_id", identifier) == identifier and row.get("strategy", "coordinated") == "coordinated"
             and row.get("expectation", case["expectation"]) == case["expectation"], "Fixed row label/strategy/expectation mismatch")
    digest = suite_hash(suite)
    _require(row.get("suite_id", suite["id"]) == suite["id"] and row.get("suite_sha256", digest) == digest,
             "Fixed row suite identity mismatch")
    domain = row.get("domain")
    _require(domain is None or (type(domain) is int and 0 <= domain <= 232), "Invalid fixed-row ROS domain")
    return case, dict(schema_version=1, suite_id=suite["id"], suite_sha256=digest, row_id=identifier,
                     index=row["index"], case_id=case["case_id"], workcell=case["workcell"],
                     workcell_sha256=_digest(suite["workcell_fingerprints"][case["workcell"]]),
                     variant=row["variant"], repeat=row["repeat"], max_plan_candidates=row["max_plan_candidates"],
                     strategy="coordinated", expectation=case["expectation"], selected=row["selected"], domain=domain)


def _policy(suite, case, fixed):
    policy = fixed.get("policy")
    expected = dict(schema_version=1, workcell=case["workcell"],
                    case="eval_post" if case["policy_name"] == "eval_post_timing" else "dual_arm_crossing", **suite["policy"])
    _require(policy == expected, "Fixed row loaded policy differs from its case/shared suite policy")
    return policy


def _path_metrics(data, *, progress=None):
    if data is None and progress is None:
        return dict(status="absent", samples=None, minimum_distance_m=None, prefix_minimum_distance_m=None,
                    duration_sec=None, sample_dt_max_sec=None, joint_travel_rad=None, tip_frame_path_m=None)
    source = data if data is not None else progress
    _require(isinstance(source, dict), "Malformed sampled path evidence")
    complete = data is not None and data.get("complete") is True
    if data is not None:
        _require(type(data.get("complete")) is bool, "Path completion must be explicit")
    count = _integer(source.get("samples"))
    minimum = source.get("minimum")
    distance = None if minimum is None else _finite(minimum.get("distance_m"))
    _require(not complete or (count >= 2 and distance is not None), "Complete path lacks sampled minimum")
    joint = source.get("joint_total_variation_rad") if complete else None
    tips = source.get("tip_frame_path_m") if complete else None
    if tips is not None:
        _require(isinstance(tips, dict), "Malformed tip travel metrics")
        for value in tips.values():
            _finite(value, nonnegative=True)
    return dict(status="complete" if complete else ("prefix" if data is not None else "interrupted"), samples=count,
                minimum_distance_m=distance if complete else None,
                prefix_minimum_distance_m=None if complete else distance,
                duration_sec=_finite(source.get("duration_sec"), nullable=True, nonnegative=True) if complete else None,
                sample_dt_max_sec=_finite(source.get("sample_dt_max_sec"), nullable=True, nonnegative=True) if complete else None,
                joint_travel_rad=_finite(joint, nullable=True, nonnegative=True), tip_frame_path_m=deepcopy(tips))


def _metrics(trace, cap):
    budget = trace.get("compute_budget")
    if budget is not None:
        _require(isinstance(budget, dict) and type(budget.get("exceeded")) is bool, "Malformed computation budget")
        _require(_finite(budget.get("limit_wall_sec"), nonnegative=True) == 5.0, "Scored budget differs from five wall seconds")
        _finite(budget.get("elapsed_wall_sec"), nonnegative=True)
    selection = trace.get("plan_selection")
    _require(selection is None or isinstance(selection, dict), "Malformed spatial selection object")
    candidates = [] if selection is None else selection.get("candidates")
    if selection is not None:
        _require(isinstance(candidates, list) and all(isinstance(candidate, dict) for candidate in candidates)
                 and type(selection.get("max_candidates")) is int and selection["max_candidates"] == cap,
                 "Spatial selection cap/candidates mismatch")
        _require(type(selection.get("attempted")) is int and selection["attempted"] == len(candidates) <= cap,
                 "Spatial candidate count exceeds its declared bound")
        _require(all(type(candidate.get("attempt")) is int for candidate in candidates)
                 and [candidate.get("attempt") for candidate in candidates] == list(range(1, len(candidates)+1)),
                 "Spatial candidate attempts are missing or reordered")
    selected = None if selection is None else selection.get("selected_attempt")
    selected_kind = None
    if selected is not None:
        _integer(selected, minimum=1)
        _require(selected <= len(candidates) and candidates[selected-1].get("accepted") is True
                 and selection.get("status") == "selected", "Invalid selected candidate index/status")
        selected_kind = candidates[selected-1].get("proposal_kind")
    latest = next((candidate for candidate in reversed(candidates) if candidate.get("curve")), None)
    progress = None if latest is None else latest.get("native_progress")
    planned = _path_metrics(trace.get("planned_clearance"), progress=progress)
    observed = _path_metrics(trace.get("observed_clearance"))
    duration = None
    curves = trace.get("planned_trajectories", [])
    if curves:
        _require(isinstance(curves, list) and all(isinstance(c, dict) and c.get("points") for c in curves), "Malformed retained planned trajectory")
        duration = max(_finite(curve["points"][-1]["t"], nonnegative=True) for curve in curves)
    plans = trace.get("plans", [])
    _require(isinstance(plans, list), "Malformed planning RPC records")
    rpc_wall = server_time = 0.0
    for plan in plans:
        _require(isinstance(plan, dict) and plan.get("source") in (None, "moveit_rpc"), "Scored rows require fresh MoveIt RPCs")
        rpc_wall += _finite(plan.get("wall_sec"), nonnegative=True)
        server_time += _finite(plan.get("server_planning_sec"), nonnegative=True)
    events, spans = trace.get("events", []), trace.get("spans", [])
    _require(isinstance(events, list) and isinstance(spans, list), "Malformed trace timeline")
    _require(all(isinstance(event, dict) and "wall_sec" in event and isinstance(event.get("name"), str) for event in events)
             and all(isinstance(span, dict) for span in spans), "Malformed trace event/span records")
    times = [_finite(event["wall_sec"], nonnegative=True) for event in events]
    _require(times == sorted(times), "Non-monotonic trace events")
    request_metrics = trace.get("metrics", {})
    _require(isinstance(request_metrics, dict), "Malformed request metrics")
    def metric(name):
        return _finite(request_metrics.get(name), nullable=True, nonnegative=True)
    settle = next((span.get("duration_wall_sec") for span in spans if span.get("name") == "settling"), None)
    actions = trace.get("action_results", [])
    _require(isinstance(actions, list) and all(isinstance(action, dict) and type(action.get("status")) is int
             and type(action.get("error_code")) is int for action in actions), "Malformed action result evidence")
    return dict(compute_wall_sec=None if budget is None else budget["elapsed_wall_sec"],
                compute_limit_wall_sec=None if budget is None else budget["limit_wall_sec"],
                budget_exceeded=None if budget is None else budget["exceeded"],
                planning_rpc_recorded_calls=len(plans), planning_rpc_wall_sec=rpc_wall,
                server_planning_sec=server_time, candidate_count=len(candidates), selected_attempt=selected,
                selected_proposal_kind=selected_kind, planned_candidate_attempt=trace.get("planned_candidate_attempt"),
                candidate_duration_sec=duration, planned=planned, observed=observed,
                dispatched_goals=_integer(trace.get("dispatched_goals", 0)),
                action_results=deepcopy(actions),
                first_motion_wall_sec=metric("received_to_first_motion_wall_sec"),
                actions_done_wall_sec=metric("received_to_actions_done_wall_sec"),
                settled_wall_sec=metric("received_to_settled_wall_sec"),
                terminal_wall_sec=metric("received_to_terminal_wall_sec"),
                settling_duration_wall_sec=_finite(settle, nullable=True, nonnegative=True))


def summarize_row(suite, planned_row, *, fixed_case_raw=None, trace_raw=None, verification_raw=None):
    """Validate source identities and retain failures without calling a failed plan a success."""
    case, result = _base(suite, planned_row)
    fixed, fixed_sha = _load(fixed_case_raw)
    trace, trace_sha = _load(trace_raw)
    verification, verification_sha = _load(verification_raw)
    _require(result["selected"] or all(value is None for value in (fixed, trace, verification)), "Unselected row has execution evidence")
    result.update(status="not_run", scored=False, request_id=None, request_status=None, outcome=None,
                  fixed_status=None if fixed is None else fixed.get("status"),
                  verification_status=None if verification is None else verification.get("status"),
                  source_sha256=dict(fixed_case=fixed_sha, trace=trace_sha, verification=verification_sha),
                  attempted=False, declared_request_id=None, delivery_unknown=False,
                  failure_kind=None, failure_stage=None, errors=[], metrics=None,
                  initial_error_rad=None, not_run_reason="not_selected" if not result["selected"] else "not_attempted",
                  verification=None)
    if fixed is not None:
        _require(type(fixed.get("schema_version")) is int and fixed["schema_version"] == 1 and fixed.get("case") == case
                 and fixed.get("case_id") == case["case_id"] and fixed.get("workcell") == case["workcell"]
                 and fixed.get("variant") == result["variant"] and fixed.get("repeat") == result["repeat"]
                 and fixed.get("row_index") == result["index"] and fixed.get("suite_sha256") == result["suite_sha256"]
                 and fixed.get("policy_name") == case["policy_name"], "Fixed-case source identity mismatch")
        _require(fixed.get("status") in ("running", "completed", "aborted"), "Unknown fixed-case lifecycle status")
        _digest(fixed.get("source_sha256"))
        _require(type(fixed.get("scored_attempted")) is bool, "Fixed case lacks explicit scored-attempt provenance")
        result["attempted"] = fixed["scored_attempted"]
        result["declared_request_id"] = fixed.get("scored_request_id")
        if result["declared_request_id"] is not None:
            _require(isinstance(result["declared_request_id"], str) and re.fullmatch(r"[0-9a-f]{32}", result["declared_request_id"]), "Invalid declared scored request ID")
        _require(result["attempted"] == (result["declared_request_id"] is not None), "Scored attempt and declared request ID mismatch")
        _policy(suite, case, fixed)
        settings = fixed.get("scored_settings", {})
        _require(settings.get("compute_budget_wall_sec") == 5.0 and type(settings.get("max_plan_candidates")) is int
                 and settings["max_plan_candidates"] == result["max_plan_candidates"]
                 and settings.get("goal_tolerance_rad") == .003, "Fixed scored settings changed")
        if fixed.get("error") is not None:
            result["errors"].append(dict(source="fixed_case", error=deepcopy(fixed["error"])))
    if verification is not None:
        _require(type(verification.get("schema_version")) is int and verification["schema_version"] == 2 and verification.get("status") in ("passed", "failed", "interrupted"),
                 "Verification is not a terminal raw report")
        _require(isinstance(verification.get("run_id"), str) and verification["run_id"], "Missing verification run identity")
        if verification.get("robot_profile_sha256") is not None:
            _require(verification["robot_profile_sha256"] == result["workcell_sha256"], "Verification workcell identity mismatch")
        if result["domain"] is not None:
            _require(verification.get("ros_domain_id") == result["domain"], "Verification ROS domain mismatch")
        if verification.get("fixed_case_result_sha256") is not None:
            _require(verification["fixed_case_result_sha256"] == fixed_sha, "Verification fixed-case raw SHA mismatch")
        physics = verification.get("physics", {})
        _require(isinstance(physics, dict), "Malformed verification physics record")
        for key in ("contact_steps", "steps", "invalid_values"):
            if key in physics:
                _integer(physics[key])
        for key in ("motion_checks_passed", "physics_checks_passed", "shutdown_clean", "process_group_gone"):
            _require(key not in verification or type(verification[key]) is bool, "Verification gate must be boolean")
        result["verification"] = dict(run_id=verification.get("run_id"),
            motion_checks_passed=verification.get("motion_checks_passed"), physics_checks_passed=verification.get("physics_checks_passed"),
            shutdown_clean=verification.get("shutdown_clean"), process_group_gone=verification.get("process_group_gone"),
            physics_complete=physics.get("complete"), contact_steps=physics.get("contact_steps"),
            physics_steps=physics.get("steps"), invalid_values=physics.get("invalid_values"),
            model_manifest_sha256=verification.get("model_manifest_sha256"), physics_report_sha256=verification.get("physics_report_sha256"))
        for key in ("model_manifest_sha256", "physics_report_sha256"):
            if verification.get(key) is not None:
                _digest(verification[key])
        if verification.get("error") is not None:
            result["errors"].append(dict(source="verification", error=deepcopy(verification["error"])))
    scored = None if fixed is None else fixed.get("scored")
    if trace is None:
        _require(fixed is None or fixed.get("status") != "completed", "Completed row is missing raw scored evidence")
        if result["attempted"]:
            result.update(status="failed", outcome="missing_scored_trace", delivery_unknown=True,
                          failure_kind="missing_scored_trace", not_run_reason=None)
        else:
            _require(scored is None, "Unattempted row has a scored entry")
            if fixed is not None or verification is not None:
                result["not_run_reason"] = "preparation_or_verification_failed"
        return result
    _require(result["attempted"], "Raw scored trace has no recorded publication attempt")
    _require(isinstance(scored, dict) and trace is not None, "Missing raw scored trace")
    _require("verification_injection" not in trace and not trace.get("preparation_source"), "Scored trace contains a substitution/injection")
    request = scored.get("request_id")
    _require(isinstance(request, str) and re.fullmatch(r"[0-9a-f]{32}", request), "Invalid scored request ID")
    _require(type(trace.get("schema_version")) is int and trace["schema_version"] == 1 and trace.get("request_id") == request
             and result["declared_request_id"] == request
             and scored.get("trace_sha256") == trace_sha and scored.get("trace_file") == request + ".json"
             and scored.get("role") == "scored" and scored.get("status") == trace.get("status"), "Scored trace hash/identity/status mismatch")
    expected_command = dict(schema_version=1, request_id=request, strategy="coordinated", target_rad=case["target_rad"])
    _require(trace.get("command") == expected_command and scored.get("command") == expected_command,
             "Scored command differs from fixed target or strategy")
    _require(trace.get("status") in ("succeeded", "failed", "cancelled", "rejected"), "Scored trace is not terminal")
    _require(scored.get("initial_state_rad") == trace.get("initial_state_rad"), "Scored initial state differs from raw trace")
    if trace.get("initial_state_rad") is not None:
        initial = _state(trace["initial_state_rad"], case["initial_rad"])
        result["initial_error_rad"] = max(abs(initial[name]-case["initial_rad"][name]) for name in initial)
    metrics = _metrics(trace, result["max_plan_candidates"])
    result.update(scored=True, request_id=request, request_status=trace["status"], metrics=metrics,
                  failure_kind=trace.get("failure_kind"), failure_stage=trace.get("failure_stage"),
                  status="failed", not_run_reason=None)
    if trace.get("error") is not None:
        result["errors"].append(dict(source="trace", error=deepcopy(trace["error"])))
    if trace["status"] == "succeeded":
        _require(not trace.get("no_motion_required") and metrics["dispatched_goals"] == 2
                 and len(metrics["action_results"]) == 2
                 and all(action.get("status") == 4 and action.get("error_code") == 0 for action in metrics["action_results"])
                 and metrics["first_motion_wall_sec"] is not None and metrics["settled_wall_sec"] is not None
                 and metrics["budget_exceeded"] is False
                 and metrics["planned"]["status"] == metrics["observed"]["status"] == "complete"
                 and metrics["planned"]["minimum_distance_m"] >= .03
                 and metrics["observed"]["minimum_distance_m"] >= .02,
                 "Succeeded trace lacks complete actions, movement, budget or clearance gates")
        _require(metrics["selected_attempt"] is not None, "Succeeded spatial request lacks a selected candidate")
        _require(isinstance(trace.get("contact_guard"), dict) and trace["contact_guard"].get("latched") is False, "Succeeded trace lacks a clear native interlock")
        result["outcome"] = "motion_succeeded"
    elif trace.get("failure_kind") in PLANNING_FAILURES or trace.get("failure_kind") == "invalid_goal":
        _require(trace["status"] == "failed" and metrics["dispatched_goals"] == 0 and not metrics["action_results"] and metrics["first_motion_wall_sec"] is None,
                 "Planning/endpoint failure dispatched or moved")
        if case["expectation"] == "invalid_goal" and trace.get("failure_kind") == "invalid_goal":
            result["outcome"] = "expected_invalid_goal"
        elif case["expectation"] == "valid_goal" and trace.get("failure_kind") in PLANNING_FAILURES:
            result["outcome"] = "planning_failed"
        else:
            result["outcome"] = "unexpected_failure"
    else:
        result["outcome"] = "unexpected_failure"
    complete = fixed["status"] == "completed" and verification is not None and verification["status"] == "passed"
    if complete:
        _digest(verification.get("model_manifest_sha256"))
        _digest(verification.get("physics_report_sha256"))
        _require(verification.get("fixed_case_result_sha256") == fixed_sha, "Completed verification lacks the raw fixed-case source hash")
        _require(type(verification.get("launch_return_code")) is int and verification["launch_return_code"] == 0,
                 "Completed verification has no clean launch return code")
        physics = verification.get("physics", {})
        _require(verification.get("robot_profile_sha256") == result["workcell_sha256"]
                 and verification.get("motion_checks_passed") is True and verification.get("physics_checks_passed") is True
                 and verification.get("shutdown_clean") is True and verification.get("process_group_gone") is True
                 and physics.get("complete") is True and physics.get("run_id") == verification.get("run_id")
                 and physics.get("contact_steps") == 0 and physics.get("invalid_values") == 0,
                 "Completed row lacks clean verified zero-contact physics")
        _require(result["initial_error_rad"] is not None and result["initial_error_rad"] <= suite["initial_tolerance_rad"],
                 "Completed scored row exceeded the fixed initial tolerance")
        _require(scored.get("outcome") == result["outcome"] and result["outcome"] != "unexpected_failure"
                 and ((case["expectation"] == "valid_goal" and result["outcome"] in ("motion_succeeded", "planning_failed"))
                      or (case["expectation"] == "invalid_goal" and result["outcome"] == "expected_invalid_goal")),
                 "Completed row outcome differs from its fixed expectation")
        result["status"] = "completed"
    return result


def _group(rows):
    counts = {status:sum(row["status"] == status for row in rows) for status in STATUSES}
    successful = sum(row["status"] == "completed" and row["outcome"] == "motion_succeeded" for row in rows)
    rejected = sum(row["status"] == "completed" and row["outcome"] == "expected_invalid_goal" for row in rows)
    selected = sum(row["selected"] for row in rows)
    scored = sum(row["scored"] for row in rows)
    denominator = lambda total, numerator: None if total == 0 else numerator / total
    times = [row["metrics"]["compute_wall_sec"] for row in rows
             if row["scored"] and row["metrics"]["compute_wall_sec"] is not None]
    valid = [row for row in rows if row["expectation"] == "valid_goal"]
    invalid = [row for row in rows if row["expectation"] == "invalid_goal"]
    valid_selected = sum(row["selected"] for row in valid)
    valid_scored = sum(row["scored"] for row in valid)
    invalid_selected = sum(row["selected"] for row in invalid)
    invalid_scored = sum(row["scored"] for row in invalid)
    return dict(planned=len(rows), selected=selected, attempted=sum(row["attempted"] for row in rows), scored=scored, **counts,
                motion_denominators=dict(all_planned=len(valid), selected=valid_selected, scored=valid_scored),
                invalid_goal_denominators=dict(all_planned=len(invalid), selected=invalid_selected, scored=invalid_scored),
                motion_succeeded=successful, expected_invalid_goal=rejected,
                planning_failed=sum(row["outcome"] == "planning_failed" for row in rows),
                raw_request_succeeded=sum(row["request_status"] == "succeeded" for row in rows),
                motion_success_rate_all_planned=denominator(len(valid), successful),
                motion_success_rate_selected=denominator(valid_selected, successful),
                motion_success_rate_scored=denominator(valid_scored, successful),
                expected_invalid_rejection_rate_selected=denominator(invalid_selected, rejected),
                expected_invalid_rejection_rate_scored=denominator(invalid_scored, rejected),
                compute_all_scored=dict(count=len(times), minimum=None if not times else min(times),
                                        median=None if not times else median(times), maximum=None if not times else max(times)))


def aggregate_results(suite, planned_catalog, rows):
    """Require the complete original catalog; never drop failed/unselected rows."""
    _require(isinstance(planned_catalog, list) and planned_catalog, "Expected complete planned row catalog")
    _require(isinstance(rows, list) and len(rows) == len(planned_catalog), "Missing or extra compact result rows")
    normalized = []
    for row in planned_catalog:
        _, identity = _base(suite, row)
        normalized.append(identity)
    repeats = max(row["repeat"] for row in normalized)
    selected_by_case = {}
    for row in normalized:
        previous = selected_by_case.setdefault(row["case_id"], row["selected"])
        _require(previous == row["selected"], "Case selection changed across variants/repeats")
    expected = planned_rows(suite, repeats, [case for case, selected in selected_by_case.items() if selected])
    _require(len(expected) == len(normalized), "Planned catalog dropped fixed cases or repeats")
    for original, identity, result in zip(expected, normalized, rows):
        _require(original["index"] == identity["index"], "Planned catalog reordered or duplicated rows")
        _json_value(result)
        _require(all(result.get(key) == value for key, value in identity.items()), "Compact result suite/row identity mismatch")
        _require(result.get("status") in STATUSES and type(result.get("scored")) is bool and type(result.get("attempted")) is bool,
                 "Invalid compact outcome status")
        _require(not result["scored"] or result["attempted"], "Observed scored trace lacks an attempt")
        _require(result["scored"] == (result.get("request_id") is not None), "Compact scored/request identity mismatch")
        _require(result["status"] != "completed" or result["scored"], "Completed row has no scored request")
        _require(result["selected"] or (result["status"] == "not_run" and not result["scored"]), "Unselected row has a measured outcome")
        if result["scored"]:
            _digest(result["source_sha256"]["trace"])
    ids = [row["request_id"] for row in rows if row["scored"]]
    _require(len(ids) == len(set(ids)), "A scored request was reused across fixed rows")
    run_ids = [row["verification"]["run_id"] for row in rows if row.get("verification") is not None]
    _require(len(run_ids) == len(set(run_ids)), "Verification run was reused across fixed rows")
    valid = [row for row in rows if row["expectation"] == "valid_goal"]
    invalid = [row for row in rows if row["expectation"] == "invalid_goal"]
    return dict(schema_version=1, suite_id=suite["id"], suite_sha256=suite_hash(suite),
                all_rows=_group(rows), valid_goal=_group(valid), expected_invalid_goal=_group(invalid),
                by_variant={variant["id"]:dict(valid_goal=_group([row for row in valid if row["variant"] == variant["id"]]),
                    expected_invalid_goal=_group([row for row in invalid if row["variant"] == variant["id"]])) for variant in suite["variants"]},
                by_case={case["case_id"]:_group([row for row in rows if row["case_id"] == case["case_id"]]) for case in suite["cases"]},
                note="All planned rows retained. Expected-invalid goals have a separate denominator. Timing includes every scored outcome; path prefix minima are never full-path minima. Raw paths/privacy need a separate public adapter.")
