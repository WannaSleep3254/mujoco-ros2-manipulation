"""Bounded optional timing metadata. No diagnostic value authorizes a command."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import time

STAMP_FIELDS = (
    "http_handler_enter_ns", "console_submit_enter_ns", "reserved_ns", "enqueued_ns",
    "publish_begin_ns", "publish_returned_ns", "first_status_received_ns", "first_terminal_received_ns",
    "cancel_http_handler_enter_ns", "cancel_enqueued_ns", "cancel_publish_begin_ns", "cancel_publish_returned_ns")


def decimal_ns(value):
    """Keep original integer nanoseconds as JSON-safe strings, without rounding."""
    return str(value) if type(value) is int and 0 <= value <= 2**63 - 1 else None


def sample_ns():
    try:
        return decimal_ns(time.monotonic_ns())
    except Exception:
        return None


def capture_clock_context():
    """Sample only at process startup. Self-observation is not peer verification."""
    value = dict(schema_version=1, context_id=None, clock_source="CLOCK_MONOTONIC",
        monotonic_implementation=None, boot_id_sha256=None, time_namespace_identity=None,
        process_pid=None, process_start_ticks=None, proof_status="unavailable", unavailable_reason=None)
    try:
        implementation = time.get_clock_info("monotonic").implementation
        if implementation != "clock_gettime(CLOCK_MONOTONIC)":
            raise ValueError("unsupported_clock")
        with Path("/proc/sys/kernel/random/boot_id").open("rb") as stream:
            boot = stream.read(129)
        if len(boot) > 128 or not re.fullmatch(rb"[0-9a-f-]{36}\n?", boot):
            raise ValueError("invalid_boot_identity")
        with Path("/proc/self/stat").open() as stream:
            process = stream.read(8193)
        if len(process) > 8192:
            raise ValueError("invalid_process_identity")
        start = int(process[process.rfind(")") + 2:].split()[19])
        namespace = os.readlink("/proc/self/ns/time")
        if not re.fullmatch(r"time:\[[0-9]{1,20}\]", namespace) or start < 0:
            raise ValueError("invalid_time_namespace")
        value.update(monotonic_implementation=implementation,
            boot_id_sha256=hashlib.sha256(boot.strip()).hexdigest(), time_namespace_identity=namespace,
            process_pid=os.getpid(), process_start_ticks=str(start), proof_status="self_observed")
        value["context_id"] = hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    except Exception:
        value["unavailable_reason"] = "clock_context_unavailable"
    return value


def console_timing(context, *, session_id, generation, selection_id, expected_session_id,
                   kind, plan_request_id, http_received_ns, submit_enter_ns):
    """One fixed-size local record, created only after a new reservation."""
    try:
        value = dict(schema_version=1, clock_context_id=(context or {}).get("context_id"),
            console_session_id=session_id, console_generation=generation,
            workspace_selection_id=selection_id, expected_motion_session_id=expected_session_id,
            kind=kind, plan_request_id=plan_request_id,
            stamps=dict.fromkeys(STAMP_FIELDS), observed_fields=[], diagnostic_status="partial", missing_reasons=[])
        record = dict(console_timing=value)
        mark_timing(record, "http_handler_enter_ns", http_received_ns)
        mark_timing(record, "console_submit_enter_ns", submit_enter_ns)
        return value
    except Exception:
        return None


def mark_timing(record, field, value):
    """Keep each original mark once. Optional metadata failures stay local."""
    try:
        timing = record.get("console_timing")
        if not isinstance(timing, dict) or field not in STAMP_FIELDS:
            return
        stamps = timing["stamps"]
        observed = timing["observed_fields"]
        if field in observed:
            return
        observed.append(field)
        if type(value) is str and value.isascii() and value.isdecimal() and len(value) <= 19 and int(value) <= 2**63 - 1:
            stamps[field] = value
        else:
            reason = field + "_unavailable"
            reasons = timing["missing_reasons"]
            if reason not in reasons and len(reasons) < 8:
                reasons.append(reason)
        complete = timing.get("clock_context_id") is not None and all(stamps[name] is not None for name in STAMP_FIELDS[:6])
        timing["diagnostic_status"] = "available" if complete else "partial"
    except Exception:
        return


def trace_clock_context(context):
    """Detach startup metadata without retaining arbitrary caller structures."""
    try:
        fields = ("schema_version", "context_id", "clock_source", "monotonic_implementation", "boot_id_sha256",
                  "time_namespace_identity", "process_pid", "process_start_ticks", "proof_status", "unavailable_reason")
        if type(context) is not dict or set(context) != set(fields):
            return None
        encoded = json.dumps(context, allow_nan=False)
        if len(encoded.encode()) > 2048:
            return None
        return deepcopy(context)
    except Exception:
        return None
