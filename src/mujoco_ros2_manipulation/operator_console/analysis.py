"""Bounded read-only request and telemetry observations, without time alignment.

Only named public fields leave this projection. A latest receiver observation is
not a sample synchronized with the selected historical request. No interpolation,
commands, filesystem lookup, host details or arbitrary error strings are exposed.
"""
import math
import re

from ..workcells.operation_contract import describe_operation, MAX_EVENTS


SOURCE_KINDS = ("scene", "joints", "image", "clock")
CLOCK_DOMAINS = frozenset(("ros_simulation", "recorder_elapsed_monotonic",
                           "source_elapsed_monotonic", "unknown"))
MAX_SOURCES = 64
_TAG = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}")
_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}")
_SHA = re.compile(r"[0-9a-f]{64}")


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _seconds(value, label):
    if value is None:
        return None
    _require(type(value) in (int, float) and 0 <= value <= 1e12
             and math.isfinite(value), "Invalid analysis " + label)
    return value


def _tag(value):
    return value if type(value) is str and _TAG.fullmatch(value) else None


def _identifier(value):
    return value if type(value) is str and _ID.fullmatch(value) else None


def _digest(value):
    if value is None:
        return None
    _require(type(value) is str and _SHA.fullmatch(value) is not None,
             "Invalid analysis scene identity")
    return value


def _timeline(record):
    """Retain counts when a status poll carries a compact operation timeline."""
    clock = record.get("clock")
    _require(clock is None or type(clock) is dict, "Invalid analysis recorder clock")
    if clock is not None and "origin_ns" in clock:
        origin = clock["origin_ns"]
        _require(type(origin) is int and 0 <= origin <= 2**63-1,
                 "Invalid analysis recorder monotonic origin")
    projected = describe_operation(record)
    timeline = projected["events"]
    if "events" not in record and isinstance(record.get("operation"), dict):
        compact = record["operation"].get("events")
        if compact is not None:
            _require(type(compact) is dict and type(compact.get("items")) is list
                     and len(compact["items"]) <= MAX_EVENTS,
                     "Invalid compact analysis timeline")
            count, retained = compact.get("source_count"), len(compact["items"])
            known = compact.get("history_provided")
            _require(type(known) is bool and type(compact.get("retained_count")) is int
                     and compact["retained_count"] == retained,
                     "Invalid compact analysis event count")
            _require((not known and count is None and retained == 0)
                     or (known and type(count) is int and retained <= count <= 2**53-1),
                     "Invalid compact analysis source count")
            omitted = compact.get("omitted_prefix_count")
            _require(type(omitted) is int and omitted == (count-retained if known else 0),
                     "Invalid compact analysis omitted count")
            checked = describe_operation(dict(record, events=compact["items"]))["events"]
            timeline = dict(checked, history_provided=known, source_count=count,
                            retained_count=retained, omitted_prefix_count=omitted)
    items = []
    last_wall = None
    for event in timeline["items"]:
        if event["wall_sec"] is not None:
            _require(last_wall is None or event["wall_sec"] >= last_wall,
                     "Analysis recorder monotonic timeline moved backwards")
            last_wall = event["wall_sec"]
        # Stage and controller labels are tokens, never arbitrary free text.
        name = _tag(event["name"])
        item = dict(name=name, phase=event["phase"] if name else "unknown",
                    label_redacted=name is None, wall_sec=event["wall_sec"],
                    simulation_sec=event["simulation_sec"])
        for field in ("arm", "controller"):
            if field in event:
                item[field] = _tag(event[field])
        for field in ("goal_uuid", "accepted", "status", "error_code",
                      "joint_state_stamp_simulation_sec"):
            if field in event:
                item[field] = event[field]
        items.append(item)
    return projected, dict(timeline, items=items,
        clock_domains=dict(wall_sec="recorder_elapsed_monotonic",
            simulation_sec="ros_simulation",
            joint_state_stamp_simulation_sec="ros_simulation"),
        cross_domain_durations_valid=False, interpolation=False)


def _source(value):
    if value is None:
        return dict(available=False, stamp_sec=None, clock_domain="unknown", age_sec=None,
            age_clock_domain="console_receiver_elapsed_monotonic", stale_after_sec=None,
            freshness="unknown", age_freshness="unknown", sequence=None)
    _require(type(value) is dict, "Expected analysis source observation")
    stamp = _seconds(value.get("stamp_sec"), "source stamp")
    domain = value.get("clock_domain", "unknown")
    _require(type(domain) is str and domain in CLOCK_DOMAINS, "Invalid analysis clock domain")
    age = _seconds(value.get("age_sec"), "receiver age")
    limit = _seconds(value.get("stale_after_sec"), "stale threshold")
    _require(limit is None or 0 < limit <= 3600, "Invalid analysis stale threshold")
    sequence = value.get("sequence")
    _require(sequence is None or type(sequence) is int and 0 <= sequence <= 2**53-1,
             "Invalid analysis source sequence")
    result = dict(available=True, stamp_sec=stamp, clock_domain=domain,
        age_sec=age, age_clock_domain="console_receiver_elapsed_monotonic",
        stale_after_sec=limit,
        freshness="unknown" if age is None or limit is None else "stale" if age > limit else "fresh",
        sequence=sequence)
    result["age_freshness"] = result["freshness"]
    # Fresh receipt alone cannot rehabilitate an ambiguous/disconnected/invalid
    # acquisition. Encoding is an observed raw sample, not an encoded-frame claim.
    status = value.get("status")
    if status is not None:
        if status == "stale":
            result["freshness"] = "stale"
        elif status not in ("ready", "encoding"):
            result["freshness"] = "unknown"
    for field in ("source_id", "binding_id", "mode", "status", "generation"):
        if field in value:
            result[field] = _identifier(value[field])
    for field in ("scene_sha256", "model_sha256"):
        if field in value:
            result[field] = _digest(value[field])
    if "preview_time_sec" in value:
        result["preview_time_sec"] = _seconds(value["preview_time_sec"], "preview position")
        result["preview_clock_domain"] = "saved_plan_elapsed"
    if value.get("kind") == "rgb8-image-source-v1":
        result["stamp_semantics"] = "latest_received_acquisition_not_encoded_frame"
    return result


def summarize_analysis(record, *, sources=None):
    """Project a raw trace or compact history record; no admission is implied.

    ``sources`` contains latest scene/joints/image/clock observations with
    explicit ``stamp_sec``, ``clock_domain``, ``age_sec`` and optionally an
    owner-provided ``stale_after_sec``. Missing ages never become fresh. Unknown
    source kinds are counted without exposing their names or arbitrary payloads.
    """
    _require(type(record) is dict, "Expected analysis request record")
    _require(sources is None or type(sources) is dict and len(sources) <= MAX_SOURCES,
             "Invalid bounded analysis source collection")
    sources = {} if sources is None else sources
    operation, timeline = _timeline(record)
    stage = _tag(operation["source_stage"])
    return dict(schema_version=1, kind="console-analysis-v1", read_only=True,
        request_id=operation["request_id"], session_id=operation["session_id"],
        plan_request_id=operation["plan_request_id"], execution_request_id=operation["execution_request_id"],
        mode=_tag(operation["mode"]), status=_tag(operation["status"]),
        source_stage=stage, phase=operation["phase"] if stage else "unknown",
        request_terminal=operation["request_terminal"], timeline=timeline,
        sources={kind:_source(sources.get(kind)) for kind in SOURCE_KINDS},
        source_counts=dict(provided=len(sources), supported=sum(kind in sources for kind in SOURCE_KINDS),
                           unknown=sum(kind not in SOURCE_KINDS for kind in sources)),
        sources_observed_at="current_console_snapshot",
        source_association="Latest receiver observations; not synchronized with the selected request",
        command_authority=False)
