"""Explicit public-evidence projections; complete runtime records remain local."""
from copy import deepcopy
import hashlib
import json
import math
import re


SESSION_FIELDS = ("schema_version", "workcell", "code_commit", "model_manifest", "policy", "evaluator", "workcell_contract", "implementation_sha256")
MODEL_FIELDS = ("schema_version", "workcell_sha256", "inputs", "outputs", "joints", "home_rad", "joint_limits")
TRACE_FIELDS = ("schema_version", "request_id", "status", "command", "initial_state_rad", "joint_state_stamp_simulation_sec",
                "state_receive_age_wall_sec", "clock_receive_age_wall_sec", "plans", "events", "spans", "metrics",
                "planned_clearance", "clearance_endpoints", "clearance_diagnostic", "trajectories", "planned_trajectories",
                "source_trajectories", "departure_search", "compute_budget", "dispatched_goals", "action_results", "failure_stage", "error")
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
PLAN_FIELDS = ("group", "wall_sec", "server_planning_sec", "requested_pipeline_id", "requested_planner_id", "selected_planner_id", "attempts", "allowed_planning_sec", "goal_tolerance_rad", "error_code")
EVENT_FIELDS = ("name", "wall_sec", "simulation_sec", "joint_state_stamp_simulation_sec", "arm", "status", "error_code", "accepted")
SPAN_FIELDS = ("name", "start_wall_sec", "status", "error", "end_wall_sec", "duration_wall_sec", "reason")


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
    result = _records(_select(data, fields[kind]))
    if kind in ("trace", "setup"):
        result["command"] = _command(result["command"])
    if kind == "session" and "model_manifest" in result:
        result["model_manifest"] = _select(result["model_manifest"], MODEL_FIELDS)
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
