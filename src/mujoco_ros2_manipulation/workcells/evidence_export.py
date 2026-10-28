"""Explicit public-evidence projections; complete runtime records remain local."""
from copy import deepcopy
import hashlib
import json
import math
import re


SESSION_FIELDS = ("schema_version", "workcell", "code_commit", "model_manifest", "policy", "evaluator", "workcell_contract", "implementation_sha256",
                  "compute_budget_sec", "max_plan_candidates", "selection_policy", "environment_policy", "environment_policy_sha256")
MODEL_FIELDS = ("schema_version", "workcell_sha256", "inputs", "outputs", "joints", "home_rad", "joint_limits")
TRACE_FIELDS = ("schema_version", "request_id", "status", "command", "initial_state_rad", "joint_state_stamp_simulation_sec",
                "state_receive_age_wall_sec", "clock_receive_age_wall_sec", "plans", "events", "spans", "metrics",
                "planned_clearance", "clearance_endpoints", "clearance_diagnostic", "trajectories", "planned_trajectories",
                "source_trajectories", "departure_search", "compute_budget", "dispatched_goals", "action_results", "failure_stage", "failure_kind", "error",
                "plan_selection", "endpoint_moveit_validity", "planned_candidate_attempt", "environment_policy", "environment_policy_sha256")
SETUP_FIELDS = ("schema_version", "request_id", "status", "command", "initial_state_rad", "trajectories")
RESULT_FIELDS = ("schema_version", "robot_id", "status", "stage", "motion_checks_passed", "shutdown_clean", "error", "scene_id",
                 "robot", "robot_profile_sha256", "model_manifest_sha256", "gui", "stop_mode", "historical_witness_reproduced",
                 "fixed_input_requests", "physics_checks_passed", "shutdown_timed_out", "launch_return_code", "launch_started", "process_group_gone")
PHYSICS_FIELDS = ("schema_version", "monitor_source_sha256", "mujoco_version", "complete", "steps", "engine_steps", "timestep_sec",
                  "integrator", "first_contact_evaluation_time_sec", "last_state_time_sec", "simulated_sec", "wall_span_sec",
                  "paced_real_time_factor", "contact_steps", "contacts", "active_contacts", "invalid_values", "time_discontinuities",
                  "pair_overflow", "physics_step", "monitor_step", "stop_enabled", "stop_latched", "stop_reason", "stop_step",
                  "stop_time_sec", "hold_ticks", "hold_time_sec")
VERIFICATION_FIELDS = ("schema_version", "date", "scope", "run_id", "fixture_sha256", "source_sha256", "historical_witness_reproduced",
                       "endpoint_moveit_validity", "status", "requests", "evidence", "maintenance")
REQUEST_ROW_FIELDS = ("role", "request_id", "strategy", "status", "trace_file", "trace_sha256", "dispatched_goals", "error", "failure_stage",
                      "timing", "clearance_diagnostic", "repeat", "initial_drift_rad", "historical_failure_reproduced")
TIMING_FIELDS = ("request_metrics", "wall_spans", "planning_calls", "simulation_reservation", "events", "failure_stage", "note")
FORBIDDEN_KEYS = frozenset(("host", "hostname", "username", "user", "runtime", "environ", "env", "environment_variables", "cwd",
                            "home_directory", "pid", "process_id", "kernel", "cpu", "logical_cpus", "memory_total_bytes", "platform",
                            "origin_ns", "code_worktree_dirty", "log_path", "trace_path", "reproduction_path", "command_trace_directory",
                            "source_path", "python_executable", "api_key", "access_token", "password", "secret"))
EXPORT_NOTE = "privacy-filtered extract; complete original retained locally"
PLAN_FIELDS = ("group", "wall_sec", "server_planning_sec", "requested_pipeline_id", "requested_planner_id", "selected_planner_id", "attempts", "allowed_planning_sec", "goal_tolerance_rad", "error_code", "source", "note")
EVENT_FIELDS = ("name", "wall_sec", "simulation_sec", "joint_state_stamp_simulation_sec", "arm", "status", "error_code", "accepted")
SPAN_FIELDS = ("name", "start_wall_sec", "status", "error", "end_wall_sec", "duration_wall_sec", "reason")


PLAN_SELECTION_FIELDS = ("group", "start_rad", "target_rad", "strategy", "goal_tolerance_rad", "threshold_m",
                         "proposal_goal_tolerances_rad", "status", "stage", "max_candidates", "attempted",
                         "selected_attempt", "elapsed_wall_sec", "candidates", "error", "save_error")
CANDIDATE_FIELDS = ("attempt", "accepted", "status", "stage", "elapsed_wall_sec", "moveit_error_code", "curve",
                    "native", "native_progress", "moveit", "rejection_kind", "rejection", "error", "evaluation_error", "save_error",
                    "proposal_kind", "arm_order", "waypoint_state_rad", "segment_trajectories")
NATIVE_FIELDS = ("complete", "samples", "sample_dt_max_sec", "duration_sec", "joint_total_variation_rad", "joint_travel_rad",
                 "arm_joint_path_rad", "tip_frame_path_m", "minimum", "distance_semantics", "environment_minimum", "rejection_kind")
DISTANCE_FIELDS = ("time_sec", "state_rad", "distance_m", "bodies", "nearest_points_m", "tips_m", "environment")
ENVIRONMENT_POLICY_FIELDS = ("schema_version", "min_environment_distance_m", "tracking_reserve_m")
ENVIRONMENT_DISTANCE_FIELDS = ("distance_m", "bodies", "nearest_points_m", "pair_count", "scope")
ENVIRONMENT_MINIMUM_FIELDS = ("time_sec", "state_rad", *ENVIRONMENT_DISTANCE_FIELDS)
MOVEIT_FIELDS = ("complete", "samples", "invalid_samples", "inter_arm_collision_samples", "witness", "first_collision")
MOVEIT_WITNESS_FIELDS = ("time_from_start_sec", "state_rad", "contacts", "inter_arm")
COMPUTE_BUDGET_FIELDS = ("limit_wall_sec", "elapsed_wall_sec", "exceeded", "enforcement")
ERROR_FIELDS = ("type", "message")


def _select(data, names):
    if not isinstance(data, dict):
        raise ValueError("Expected evidence object")
    return {name: deepcopy(data[name]) for name in names if name in data}


def audit_public_evidence(data, *, forbidden_tokens=()):
    """Reject private metadata and paths at any depth, including arbitrary strings."""
    count = 0
    tokens = [token for token in forbidden_tokens if isinstance(token, str) and len(token) > 2]
    absolute = re.compile(r"(?:^|[\s\"'=])/(?:[^/\s]+/)+[^\s]*")
    secret = re.compile(r"(?i)(?:bearer\s+[a-z0-9._-]+|(?:password|api[_-]?key|access[_-]?token)\s*[:=])")
    def visit(value, trail):
        nonlocal count
        if isinstance(value, dict):
            for key, item in value.items():
                if not isinstance(key, str) or key.lower() in FORBIDDEN_KEYS:
                    raise ValueError("Private evidence field at " + trail)
                visit(key, trail + ".<key>")
                visit(item, trail + "." + key)
        elif isinstance(value, list):
            for index, item in enumerate(value):
                visit(item, trail + "." + str(index))
        elif isinstance(value, str):
            count += 1
            if (value.startswith(("/", "~", "file:")) or "/home/" in value or "/tmp/" in value
                    or re.search(r"[A-Za-z]:[\\/]", value) or absolute.search(value) or secret.search(value)
                    or re.search(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}", value)):
                raise ValueError("Private path or credential-like string at " + trail)
            for token in tokens:
                if re.search(r"(?<![a-zA-Z0-9_])" + re.escape(token) + r"(?![a-zA-Z0-9_])", value, re.IGNORECASE):
                    raise ValueError("Local identity token at " + trail)
        elif value is None or type(value) in (int, bool):
            return
        elif type(value) is float and math.isfinite(value):
            return
        else:
            raise ValueError("Unexpected/nonfinite evidence value at " + trail)
    visit(data, "evidence")
    return dict(passed=True, checked_strings=count, absolute_paths=False, host_environment_metadata=False, monotonic_origin=False)


def _records(data):
    result = deepcopy(data)
    for key, fields in (("plans", PLAN_FIELDS), ("planning_calls", PLAN_FIELDS),
                        ("events", EVENT_FIELDS), ("spans", SPAN_FIELDS), ("wall_spans", SPAN_FIELDS)):
        if key in result:
            result[key] = [_select(row, fields) for row in result[key]]
    if result.get("error") is not None:
        result["error"] = _select(result["error"], ("type", "message"))
    return result


def _command(data):
    fields = {"schema_version", "request_id", "strategy", "target_rad"}
    if not isinstance(data, dict) or set(data) != fields or type(data["schema_version"]) is not int or data["schema_version"] != 1:
        raise ValueError("Evidence command must match the strict four-field v1 contract")
    if (not isinstance(data["request_id"], str) or not re.fullmatch(r"[0-9a-f]{32}", data["request_id"])
            or data["strategy"] not in ("coordinated", "departure_timing") or not isinstance(data["target_rad"], dict)
            or len(data["target_rad"]) != 12):
        raise ValueError("Evidence command identity/strategy/joint count mismatch")
    for name, position in data["target_rad"].items():
        if (not isinstance(name, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", name)
                or type(position) not in (int, float) or not math.isfinite(position)):
            raise ValueError("Evidence command requires named finite numeric joints")
    return deepcopy(data)


def _timing(data):
    result = _records(_select(data, TIMING_FIELDS))
    if "wall_clock" in data:
        result["wall_clock"] = _select(data["wall_clock"], ("wall", "simulation"))
    return result


def _numeric_mapping(data):
    if not isinstance(data, dict) or any(
        not isinstance(name, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", name)
        or type(value) not in (int, float) or not math.isfinite(value)
        for name, value in data.items()
    ):
        raise ValueError("Expected named finite numeric evidence values")
    return deepcopy(data)


def _curve(data):
    result = _select(data, ("names", "points"))
    if "points" in result:
        result["points"] = [_select(point, ("t", "p", "v", "a")) for point in result["points"]]
    return result


def _finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _environment_policy(data):
    result = _select(data, ENVIRONMENT_POLICY_FIELDS)
    if (set(result) != set(ENVIRONMENT_POLICY_FIELDS) or type(result["schema_version"]) is not int
            or result["schema_version"] != 1):
        raise ValueError("Expected public environment-policy schema 1")
    for name, lower, upper in (("min_environment_distance_m", .001, .1), ("tracking_reserve_m", 0., .05)):
        value = result[name]
        if not _finite_number(value) or not lower <= value <= upper:
            raise ValueError("Expected bounded numeric environment-policy values")
    return result


def _environment_distance(data, *, minimum=False):
    """Retain only bounded public witness fields, never arbitrary nested data."""
    result = _select(data, ENVIRONMENT_MINIMUM_FIELDS if minimum else ENVIRONMENT_DISTANCE_FIELDS)
    if not set(ENVIRONMENT_DISTANCE_FIELDS) <= set(result):
        raise ValueError("Expected complete public environment-distance fields")
    distance, count, bodies, points = (result[name] for name in ("distance_m", "pair_count", "bodies", "nearest_points_m"))
    if (not _finite_number(distance)
            or type(count) is not int or not 1 <= count <= 4096
            or result["scope"] != "moving_robot_vs_fixed_environment"):
        raise ValueError("Expected finite environment distance and bounded pair count/scope")
    if (type(bodies) is not list or len(bodies) != 2
            or any(type(name) is not str or not name.strip() or len(name) > 256 for name in bodies)
            or bodies[0] == bodies[1]):
        raise ValueError("Expected two bounded public environment body names")
    if (type(points) is not list or len(points) != 2
            or any(type(point) is not list or len(point) != 3
                   or any(not _finite_number(value) for value in point)
                   for point in points)):
        raise ValueError("Expected two finite public environment witness points")
    if "time_sec" in result:
        stamp = result["time_sec"]
        if not _finite_number(stamp) or not 0 <= stamp <= 1e9:
            raise ValueError("Expected bounded public environment witness time")
    if "state_rad" in result:
        state = result["state_rad"]
        if (not isinstance(state, dict) or not 1 <= len(state) <= 32
                or any(not isinstance(name, str) or len(name) > 128 for name in state)):
            raise ValueError("Expected bounded named environment witness state")
        if any(not _finite_number(value) for value in state.values()):
            raise ValueError("Expected named finite numeric environment witness state")
        result["state_rad"] = _numeric_mapping(state)
    return result


def _environment_rejection(value):
    if value not in ("native_clearance", "environment_clearance"):
        raise ValueError("Unexpected public native rejection kind")
    return value


def _environment_additions(data):
    """Project additive environment witnesses while leaving legacy fields intact."""
    result = deepcopy(data)
    if not isinstance(result, dict):
        return result
    if result.get("environment_minimum") is not None:
        result["environment_minimum"] = _environment_distance(result["environment_minimum"], minimum=True)
    minimum = result.get("minimum")
    if isinstance(minimum, dict) and minimum.get("environment") is not None:
        minimum["environment"] = _environment_distance(minimum["environment"])
    if "environment_minimum" in result and "rejection_kind" in result:
        result["rejection_kind"] = _environment_rejection(result["rejection_kind"])
    return result


def _environment_trace_additions(result):
    if "planned_clearance" in result:
        result["planned_clearance"] = _environment_additions(result["planned_clearance"])
    diagnostic = result.get("clearance_diagnostic")
    if isinstance(diagnostic, dict):
        if diagnostic.get("environment_witness") is not None:
            diagnostic["environment_witness"] = _environment_distance(diagnostic["environment_witness"], minimum=True)
            if diagnostic.get("violation_scope") is not None:
                diagnostic["violation_scope"] = _environment_rejection(diagnostic["violation_scope"])
        witness = diagnostic.get("witness")
        if isinstance(witness, dict):
            if witness.get("scope") == "moving_robot_vs_fixed_environment":
                diagnostic["witness"] = _environment_distance(witness, minimum=True)
            elif witness.get("environment") is not None:
                witness["environment"] = _environment_distance(witness["environment"])
    clearance_endpoints = result.get("clearance_endpoints")
    endpoints = clearance_endpoints.get("endpoints", {}) if isinstance(clearance_endpoints, dict) else {}
    if isinstance(endpoints, dict):
        for endpoint in (endpoints.get("start"), endpoints.get("goal")):
            if isinstance(endpoint, dict) and endpoint.get("environment") is not None:
                endpoint["environment"] = _environment_distance(endpoint["environment"])


def _native(data, *, progress=False):
    result = _select(data, ("samples", "minimum", "environment_minimum", "rejection_kind") if progress else NATIVE_FIELDS)
    if result.get("minimum") is not None:
        minimum = result["minimum"] = _select(result["minimum"], DISTANCE_FIELDS)
        if "state_rad" in minimum:
            minimum["state_rad"] = _numeric_mapping(minimum["state_rad"])
        if minimum.get("environment") is not None:
            minimum["environment"] = _environment_distance(minimum["environment"])
    if result.get("environment_minimum") is not None:
        result["environment_minimum"] = _environment_distance(result["environment_minimum"], minimum=True)
    if "rejection_kind" in result:
        result["rejection_kind"] = _environment_rejection(result["rejection_kind"])
    for key in ("joint_travel_rad", "arm_joint_path_rad", "tip_frame_path_m"):
        if key in result:
            result[key] = _numeric_mapping(result[key])
    return result


def _moveit(data):
    result = _select(data, MOVEIT_FIELDS)
    for key in ("witness", "first_collision"):
        if result.get(key) is None:
            continue
        witness = result[key] = _select(result[key], MOVEIT_WITNESS_FIELDS)
        if "state_rad" in witness:
            witness["state_rad"] = _numeric_mapping(witness["state_rad"])
        if "contacts" in witness:
            contacts = []
            for contact in witness["contacts"]:
                if isinstance(contact, dict):
                    contacts.append(_select(contact, ("body1", "body2")))
                elif isinstance(contact, list) and len(contact) == 2 and all(isinstance(name, str) for name in contact):
                    contacts.append(deepcopy(contact))
                else:
                    raise ValueError("Expected public MoveIt contact body pair")
            witness["contacts"] = contacts
    return result


def _plan_selection(data):
    result = _select(data, PLAN_SELECTION_FIELDS)
    for key in ("start_rad", "target_rad"):
        if key in result:
            result[key] = _numeric_mapping(result[key])
    for key in ("error", "save_error"):
        if result.get(key) is not None:
            result[key] = _select(result[key], ERROR_FIELDS)
    if "candidates" in result:
        candidates = []
        for source in result["candidates"]:
            candidate = _select(source, CANDIDATE_FIELDS)
            for key in ("error", "evaluation_error", "save_error"):
                if candidate.get(key) is not None:
                    candidate[key] = _select(candidate[key], ERROR_FIELDS)
            if "curve" in candidate:
                candidate["curve"] = _curve(candidate["curve"])
            if "waypoint_state_rad" in candidate:
                candidate["waypoint_state_rad"] = _numeric_mapping(candidate["waypoint_state_rad"])
            if "segment_trajectories" in candidate:
                segments = candidate["segment_trajectories"]
                if not isinstance(segments, list) or len(segments) > 2:
                    raise ValueError("Expected at most two public waypoint segments")
                candidate["segment_trajectories"] = [_curve(segment) for segment in segments]
            if "native" in candidate:
                candidate["native"] = _native(candidate["native"])
            if "native_progress" in candidate:
                candidate["native_progress"] = _native(candidate["native_progress"], progress=True)
            if "moveit" in candidate:
                candidate["moveit"] = _moveit(candidate["moveit"])
            candidates.append(candidate)
        result["candidates"] = candidates
    return result


def _reject_verification_injection(data):
    # Filtering away this marker would misrepresent a test substitution as a
    # measured solver failure or a fresh planning RPC. It needs a separate
    # publication contract, including its same-request source-path provenance.
    if "verification_injection" in data:
        raise ValueError("Verification-only injected trace cannot be exported as ordinary motion evidence")
    for plan in data.get("plans", []):
        source = plan.get("source") if isinstance(plan, dict) else None
        if isinstance(source, str) and source.startswith("verification_"):
            raise ValueError("Verification-only planning source cannot be exported without its injection contract")


def export_evidence(raw, kind, *, forbidden_tokens=()):
    """Project validated motion-record shapes, not arbitrary documents or general PII.

    Retained measurement values are copied verbatim. Unexpected free text still
    needs review; the audit checks specific paths, identities and metadata only.
    """
    if not isinstance(raw, bytes):
        raise ValueError("Expected complete uncompressed source bytes")
    data = json.loads(raw)
    fields = {"session": SESSION_FIELDS, "trace": TRACE_FIELDS, "setup": SETUP_FIELDS,
              "result": RESULT_FIELDS, "physics": PHYSICS_FIELDS, "verification": VERIFICATION_FIELDS}
    if kind not in fields:
        raise ValueError("Unknown evidence export kind")
    if kind in ("trace", "setup"):
        _reject_verification_injection(data)
    result = _records(_select(data, fields[kind]))
    if kind in ("trace", "setup"):
        result["command"] = _command(result["command"])
    if kind == "trace":
        _environment_trace_additions(result)
        if "plan_selection" in result:
            result["plan_selection"] = _plan_selection(result["plan_selection"])
        if "compute_budget" in result:
            result["compute_budget"] = _select(result["compute_budget"], COMPUTE_BUDGET_FIELDS)
        if "endpoint_moveit_validity" in result:
            result["endpoint_moveit_validity"] = _select(result["endpoint_moveit_validity"], ("start", "goal"))
    if kind == "session" and "model_manifest" in result:
        result["model_manifest"] = _select(result["model_manifest"], MODEL_FIELDS)
    if kind in ("session", "trace"):
        if "environment_policy" in result:
            result["environment_policy"] = _environment_policy(result["environment_policy"])
        if "environment_policy_sha256" in result:
            digest = result["environment_policy_sha256"]
            if not isinstance(digest, str) or not re.fullmatch("[0-9a-f]{64}", digest):
                raise ValueError("Expected public environment-policy SHA256")
    if kind == "result":
        for original, public in (("fixture_sha256", "measured_fixture_sha256"), ("physics_report_sha256", "physics_raw_source_sha256"),
                                 ("reproduction_sha256", "reproduction_raw_source_sha256")):
            if original in data:
                result[public] = deepcopy(data[original])
    if kind == "verification":
        result["requests"] = [_records(_select(row, REQUEST_ROW_FIELDS)) for row in data.get("requests", [])]
        for row in result["requests"]:
            if "timing" in row:
                row["timing"] = _timing(row["timing"])
        if "maintenance" in data:
            result["maintenance"] = _select(data["maintenance"], ("requests", "statuses", "reproduction_source_sha256"))
    result["evidence_export"] = dict(schema_version=1, kind=kind, raw_source_sha256=hashlib.sha256(raw).hexdigest(),
                                    retained_fields=list(result), note=EXPORT_NOTE)
    audit_public_evidence(result, forbidden_tokens=forbidden_tokens)
    return result


def encoded_evidence(data):
    return (json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")
