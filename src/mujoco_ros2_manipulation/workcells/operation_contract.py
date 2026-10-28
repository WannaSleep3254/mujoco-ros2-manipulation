"""Read-only operation observations; never admit, schedule or transmit commands.

This additive presentation contract preserves source stages and clock domains.
It does not replace the v1 motion command, a controller adapter or admission rules.
"""
import math
import re


MAX_EVENTS = 128
MODES = frozenset(("automatic", "plan_only", "approved_plan"))
TERMINAL_STATUSES = frozenset(("succeeded", "failed", "cancelled", "rejected"))
STAGE_PHASES = {
    "submitted": "admission", "received": "admission", "queued": "admission",
    "state_snapshot": "admission",
    "computation_started": "planning", "endpoint_clearance": "planning",
    "endpoint_validity": "planning", "planning_both_arms": "planning",
    "planning_left_arm": "planning", "planning_right_arm": "planning",
    "pose_ik": "planning",
    "spatial_candidate_search": "planning", "departure_search": "planning",
    "safety_checks": "planning", "computation_done": "planning", "state_recheck": "planning",
    "plan_ready": "review", "plan_revoked": "review", "approval_validating": "review",
    "approved_plan_recheck": "review", "final_state_recheck": "dispatch",
    "dispatch": "dispatch", "goal_sent": "dispatch", "goal_acceptance": "dispatch",
    "first_motion": "executing", "first_motion_left": "executing", "first_motion_right": "executing",
    "goal_result": "executing", "actions_done": "executing",
    "settling": "settling", "settled": "settling",
    "terminal": "terminal", "plan_not_required": "terminal",
    # A cancellation event alone does not identify which operation phase was interrupted.
    "cancel_received": "unknown", "cancel_requested": "unknown",
    "cancel_submitting": "unknown", "cancel_submission_unconfirmed": "unknown",
}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _text(value, field, *, maximum=128, identifier=False):
    if value is None:
        return None
    _require(type(value) is str and 0 < len(value) <= maximum
             and not any(ord(char) < 32 for char in value), "Invalid bounded " + field)
    if identifier:
        _require(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.:-]*", value) is not None,
                 "Invalid observation identifier " + field)
    return value


def _seconds(value, field):
    if value is None:
        return None
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and 0 <= value <= 1e12
    except OverflowError:
        valid = False
    _require(valid, "Invalid finite seconds " + field)
    return value


def _optional_bool(value, field):
    _require(value is None or type(value) is bool, "Invalid boolean " + field)
    return value


def _event(source):
    _require(type(source) is dict, "Expected an operation event object")
    name = _text(source.get("name"), "event name")
    _require(name is not None, "Expected an operation event name")
    event = dict(name=name, phase=STAGE_PHASES.get(name, "unknown"),
                 wall_sec=_seconds(source.get("wall_sec"), "event wall_sec"),
                 simulation_sec=_seconds(source.get("simulation_sec"), "event simulation_sec"))
    if "joint_state_stamp_simulation_sec" in source:
        event["joint_state_stamp_simulation_sec"] = _seconds(
            source["joint_state_stamp_simulation_sec"], "joint state source stamp")
    if "arm" in source:
        event["arm"] = _text(source["arm"], "event arm", identifier=True)
    if "controller" in source:
        event["controller"] = _text(source["controller"], "event controller", identifier=True)
    if "goal_uuid" in source:
        value = source["goal_uuid"]
        _require(type(value) is str and re.fullmatch("[0-9a-f]{32}", value) is not None,
                 "Invalid confirmed goal UUID")
        event["goal_uuid"] = value
    if "accepted" in source:
        event["accepted"] = _optional_bool(source["accepted"], "goal accepted")
    for field in ("status", "error_code"):
        if field in source:
            value = source[field]
            _require(type(value) is int and -(2**31) <= value < 2**31,
                     "Invalid event integer " + field)
            event[field] = value
    return event


def _guard(source):
    if source is None:
        return None
    _require(type(source) is dict, "Expected a guard observation object")
    phase = _text(source.get("phase"), "guard phase", maximum=64)
    # The native guard initially reports an empty reason before any latch.
    reason = source.get("reason")
    reason = "" if reason == "" else _text(reason, "guard reason")
    latched = _optional_bool(source.get("latched"), "guard latched")
    return dict(phase=phase, reason=reason, latched=latched,
                stop_confirmation=("reported_stopped" if latched is True and phase == "stopped"
                    and reason in ("contact", "clock_stall") else "not_established"),
                source="provided_guard_report")


def describe_operation(record):
    """Return a bounded detached JSON observation from a trace or status summary.

    An explicit ``stage`` takes precedence over the last event name. Unknown
    stages stay unknown, even with a terminal status. Up to the last 128 events
    are projected; omitted prefixes are counted. Event clock values are copied,
    never subtracted across domains. Missing ages establish no freshness claim.
    ``cancel_submission`` is optional presentation state, not a motion v1 field.
    """
    _require(type(record) is dict, "Expected an operation record object")
    source_events = record.get("events", [])
    _require(type(source_events) is list, "Expected an operation event list")
    events = [_event(source) for source in source_events[-MAX_EVENTS:]]
    stage = _text(record.get("stage", events[-1]["name"] if events else None), "source stage")
    status = _text(record.get("status"), "request status", maximum=64)
    request = _text(record.get("request_id"), "request_id", identifier=True)
    session = _text(record.get("session_id"), "session_id", identifier=True)
    mode = _text(record.get("mode", "automatic"), "operation mode", maximum=64)
    approval = record.get("approval")
    _require(approval is None or type(approval) is dict, "Expected approval link object")
    linked_plan = _text((approval or {}).get("plan_request_id"), "approval plan_request_id", identifier=True)
    plan = _text(record.get("plan_request_id"), "plan_request_id", identifier=True)
    _require(plan is None or linked_plan is None or plan == linked_plan, "Conflicting plan identifiers")
    plan = plan or linked_plan or (request if mode == "plan_only" else None)
    execution = _text(record.get("execution_request_id"), "execution_request_id", identifier=True)
    execution = execution or (request if mode in ("automatic", "approved_plan") else None)
    submission = record.get("cancel_submission")
    _require(submission in (None, "submitting", "submitted", "unconfirmed"), "Invalid cancel submission observation")
    acknowledged = stage == "cancel_received" or any(event["name"] == "cancel_received" for event in events)
    # A discarded event prefix cannot prove there was no acknowledgement.
    history_provided = "events" in record
    ack_known_absent = history_provided and len(source_events) <= MAX_EVENTS
    if status == "cancelled":
        cancel_state = "request_cancelled"
    elif acknowledged and status in ("failed", "rejected"):
        cancel_state = "request_failed_after_cancel"
    elif acknowledged and status == "succeeded":
        cancel_state = "request_completed_after_cancel"
    elif acknowledged:
        cancel_state = "server_acknowledged"
    elif submission == "unconfirmed" or stage == "cancel_submission_unconfirmed":
        cancel_state = "submission_unconfirmed"
    elif submission == "submitting" or stage == "cancel_submitting":
        cancel_state = "submission_in_progress"
    elif submission == "submitted" or stage == "cancel_requested" or status == "cancel_requested":
        cancel_state = "submitted"
    else:
        cancel_state = "not_observed"
    guard = _guard(record.get("contact_guard", record.get("guard")))
    return dict(schema_version=1, kind="operation-observation-v1", source_stage=stage,
        phase=STAGE_PHASES.get(stage, "unknown"), request_id=request, session_id=session,
        mode=mode, mode_known=mode in MODES, mode_origin="record" if "mode" in record else "legacy_default",
        plan_request_id=plan, execution_request_id=execution,
        status=status, request_terminal=status in TERMINAL_STATUSES,
        failure=dict(kind=_text(record.get("failure_kind"), "failure kind"),
                     source_stage=_text(record.get("failure_stage"), "failure stage")),
        cancellation=dict(state=cancel_state, server_acknowledged=True if acknowledged else False if ack_known_absent else None,
                          implies_physical_stop=False),
        physical_guard=guard,
        owner_cleanup=dict(clean=_optional_bool(record.get("cleanup_clean"), "owner cleanup clean"),
            shutdown_clean=_optional_bool(record.get("shutdown_clean"), "owner shutdown clean"),
            process_group_gone=_optional_bool(record.get("process_group_gone"), "owner process group gone")),
        clocks=dict(wall="recorder_elapsed_monotonic_seconds", simulation="ros_simulation_seconds",
                    cross_domain_durations_valid=False),
        freshness=dict(scope="recorded_snapshot_ages_only",
            joint_state_source_stamp_simulation_sec=_seconds(record.get("joint_state_stamp_simulation_sec"), "joint state stamp"),
            joint_state_age_wall_sec=_seconds(record.get("state_receive_age_wall_sec", record.get("state_age_sec")), "joint state age"),
            clock_age_wall_sec=_seconds(record.get("clock_receive_age_wall_sec", record.get("clock_age_sec")), "clock age")),
        events=dict(items=events, history_provided=history_provided,
                    source_count=len(source_events) if history_provided else None, retained_count=len(events),
                    omitted_prefix_count=max(0, len(source_events)-MAX_EVENTS)),
        admission_owned_by="existing_command_server", read_only=True)
