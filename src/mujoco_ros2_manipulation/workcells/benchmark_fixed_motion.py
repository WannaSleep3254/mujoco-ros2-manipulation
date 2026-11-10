"""Run selected fixed-suite rows in fresh, sequential owned ROS simulations.

Every declared row is recorded before execution, including unselected/not-run rows.
The raw output stays local under runtime/validation; this command never builds
models, retries scored requests, or publishes evidence. Native physics coverage
and shutdown checks remain the standard run_verification lifecycle's job.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
from statistics import median
import time
import uuid

import rclpy
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.timing import sample_ns, trace_clock_context
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import diagnostic_environment, finalize_physics
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.fixed_execution import FixedCaseExecution
from mujoco_ros2_manipulation.workcells.fixed_results import aggregate_results, summarize_row
from mujoco_ros2_manipulation.workcells.fixed_suite import (
    BASE_ID, POST_ID, check_suite, make_suite, planned_rows, suite_hash,
)
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.verify import require
from mujoco_ros2_manipulation.workcells.verify_motion_reproduction import require_unused_domain


OUTCOMES = frozenset(("motion_succeeded", "planning_failed", "expected_invalid_goal"))

TIMING_EVENTS = ("received", "first_motion", "actions_done", "settled", "terminal")
TIMING_METRICS = ("publish_call",) + tuple("publish_to_" + name for name in TIMING_EVENTS)


class _ObservedPublisher:
    """Observe the existing publisher once, without introducing a transport."""
    def __init__(self, publisher, observation):
        self.publisher = publisher
        self.observation = observation

    def get_subscription_count(self):
        return self.publisher.get_subscription_count()

    def publish(self, message):
        record = None
        try:
            payload = json.loads(message.data)
            identifier = payload.get("request_id")
            if isinstance(identifier, str) and re.fullmatch(r"[0-9a-f]{32}", identifier):
                records = self.observation["publications"]
                prior = next((item for item in records if item["request_id"] == identifier), None)
                if prior is not None:
                    prior["duplicate_calls"] += 1
                elif len(records) < 32:
                    record = dict(request_id=identifier,
                        payload_sha256=hashlib.sha256(message.data.encode()).hexdigest(),
                        command_sha256=hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
                        publish_begin_ns=sample_ns(), publish_returned_ns=None,
                        publish_outcome="running", exception_type=None, duplicate_calls=0)
                    records.append(record)
                else:
                    self.observation["dropped_publications"] += 1
        except Exception:
            # The underlying publisher remains authoritative for malformed input.
            pass
        try:
            result = self.publisher.publish(message)
        except BaseException as error:
            if record is not None:
                record.update(publish_returned_ns=sample_ns(), publish_outcome="raised",
                              exception_type=type(error).__name__)
            raise
        if record is not None:
            record.update(publish_returned_ns=sample_ns(), publish_outcome="returned")
        return result


def _observe_publications(node):
    observation = dict(schema_version=1, transport="same_process_ros_topic",
        session_id=getattr(node, "session_id", None),
        clock_context=trace_clock_context(getattr(node, "timing_clock_context", None)),
        publications=[], dropped_publications=0, scored=None,
        unavailable_reason=None,
        http_browser_timing=dict(value_sec=None, unavailable_reason="fixed_suite_uses_ros_topic"))
    publisher = getattr(node, "command_publisher", None)
    if publisher is not None and callable(getattr(publisher, "publish", None)):
        node.command_publisher = _ObservedPublisher(publisher, observation)
    else:
        observation["unavailable_reason"] = "local_publisher_unavailable"
    return observation


def _original_ns(value):
    if type(value) is str and value.isascii() and value.isdecimal() and len(value) <= 19:
        parsed = int(value)
        if parsed <= 2**63 - 1:
            return parsed
    return None


def _timing_interval(begin, end, missing_reason):
    start, finish = _original_ns(begin), _original_ns(end)
    if start is None or finish is None:
        return dict(value_sec=None, unavailable_reason=missing_reason)
    if finish < start:
        return dict(value_sec=None, unavailable_reason="clock_order_invalid")
    return dict(value_sec=(finish - start) / 1e9, unavailable_reason=None)


def _scored_timing(observation, result, trace):
    identifier = (result.get("scored") or {}).get("request_id")
    publication = next((item for item in observation["publications"]
                        if item["request_id"] == identifier), None)
    source = trace.get("clock") or {}
    context = observation.get("clock_context")
    reason = None
    if publication is None:
        reason = "local_publication_unavailable"
    elif publication["duplicate_calls"]:
        reason = "duplicate_publication_identity"
    elif publication["publish_outcome"] != "returned":
        reason = "publication_did_not_return"
    elif publication["command_sha256"] != hashlib.sha256(
            json.dumps(trace.get("command"), sort_keys=True, separators=(",", ":")).encode()).hexdigest():
        reason = "published_command_mismatch"
    elif (not isinstance(context, dict) or context.get("proof_status") != "self_observed"
          or context.get("clock_source") != "CLOCK_MONOTONIC"
          or not isinstance(context.get("context_id"), str)
          or not re.fullmatch(r"[0-9a-f]{64}", context["context_id"])):
        reason = "local_clock_context_unavailable"
    elif (source.get("context") != context or source.get("context_id") != context["context_id"]):
        reason = "source_clock_context_unproven"
    elif (trace.get("request_id") != identifier or trace.get("session_id") != observation["session_id"]
          or result.get("session_id") != observation["session_id"] or not observation["session_id"]):
        reason = "request_session_mismatch"
    stamps = dict(publish_begin_ns=(publication or {}).get("publish_begin_ns"),
                  publish_returned_ns=(publication or {}).get("publish_returned_ns"),
                  received_ns=source.get("origin_monotonic_ns"))
    for name in TIMING_EVENTS[1:]:
        stamps[name + "_ns"] = next((event.get("monotonic_ns") for event in trace.get("events", [])
                                    if isinstance(event, dict) and event.get("name") == name), None)
    metrics = dict(publish_call=_timing_interval(stamps["publish_begin_ns"], stamps["publish_returned_ns"],
                                                "local_publication_stamps_unavailable"))
    received = _original_ns(stamps["received_ns"])
    begin = _original_ns(stamps["publish_begin_ns"])
    if reason is None and (received is None or begin is None):
        reason = "original_receive_stamp_unavailable"
    if reason is None and (received < begin or str(source.get("origin_ns")) != stamps["received_ns"]):
        reason = "source_clock_order_invalid"
    returned = _original_ns(stamps["publish_returned_ns"])
    if reason is None and (returned is None or returned < begin):
        reason = "local_publish_return_stamp_invalid"
    for name in TIMING_EVENTS:
        end = stamps[name + "_ns"]
        event_reason = reason
        if event_reason is None and _original_ns(end) is not None and _original_ns(end) < received:
            event_reason = "source_clock_order_invalid"
        metrics["publish_to_" + name] = (dict(value_sec=None, unavailable_reason=event_reason)
            if event_reason else _timing_interval(stamps["publish_begin_ns"], end, "original_" + name + "_stamp_unavailable"))
    return dict(request_id=identifier, source_session_id=trace.get("session_id"),
                source_clock=deepcopy(source), stamps=stamps, metrics=metrics,
                clock_join=dict(status="same_process" if reason is None else "unavailable", unavailable_reason=reason),
                first_motion_semantics="first observed joint displacement above the existing threshold")


def _retain_timing(row, result):
    observation = row.get("timing")
    if observation is None or result is None or row.get("trace_path") is None:
        return
    try:
        observation["scored"] = _scored_timing(observation, result, _read_json(Path(row["trace_path"])))
    except Exception as error:
        observation["scored"] = None
        observation["unavailable_reason"] = "timing_evidence_unavailable"
        observation["diagnostic_error_type"] = type(error).__name__


def _timing_summary(rows):
    groups = {}
    for row in rows:
        if not row["selected"]:
            continue
        key = row["case_id"], row["variant"]
        group = groups.setdefault(key, dict(case_id=key[0], variant=key[1], planned=0,
            completed=0, aborted=0, not_run=0, metrics={name: [] for name in TIMING_METRICS}))
        group["planned"] += 1
        group[row["status"]] = group.get(row["status"], 0) + 1
        metrics = ((row.get("timing") or {}).get("scored") or {}).get("metrics", {})
        for name, values in group["metrics"].items():
            value = metrics.get(name, {}).get("value_sec")
            if row["status"] == "completed" and value is not None:
                values.append(value)
    for group in groups.values():
        group["metrics"] = {name: dict(count=len(values), median_sec=median(values) if values else None,
                                      max_sec=max(values) if values else None)
                            for name, values in group["metrics"].items()}
    return dict(groups=list(groups.values()), statistics="individual rows, count, median and maximum only",
                scope="fixed inputs and selected repeats; ROS topic publication, without browser or HTTP timing")


def _numeric_host_load():
    try:
        load = list(os.getloadavg())
    except (AttributeError, OSError):
        load = None
    return dict(cpu_count=os.cpu_count(), load_average_1_5_15_min=load)


def _unique_pairs(items):
    value = {}
    for key, item in items:
        if key in value:
            raise ValueError("Duplicate suite field: " + key)
        value[key] = item
    return value


def _load_declared_suite(path, cells):
    raw = path.read_bytes()
    declared = json.loads(raw, object_pairs_hook=_unique_pairs)
    expected = make_suite(cells[BASE_ID], cells[POST_ID])
    require(isinstance(declared, dict) and declared == expected
            and suite_hash(declared) == suite_hash(expected),
            "Suite file differs from the immutable factory declarations or pinned sources")
    return declared, hashlib.sha256(raw).hexdigest()


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


def _row_id(row):
    return f'r{row["repeat"]:02d}-{row["case_id"]}-{row["variant"]}'


def _requested_cases(values):
    if values is None:
        return None
    return [part for value in values for part in value.split(",") if part]


def _full_rows(suite, repeats, case_ids, *, domain_start=None):
    cases = {case["case_id"]: case for case in suite["cases"]}
    rows = planned_rows(suite, repeats=repeats, case_ids=case_ids)
    selected_count = sum(row["selected"] for row in rows)
    if domain_start is not None:
        require(type(domain_start) is int and 0 <= domain_start <= 232
                and selected_count > 0 and domain_start + selected_count - 1 <= 232,
                "Selected rows require unique ROS domains within 0..232; lower --domain or reduce selected rows/repeats")
    ordinal = 0
    for row in rows:
        row["domain"] = domain_start + ordinal if row["selected"] and domain_start is not None else None
        ordinal += bool(row["selected"])
        row.update(row_id=_row_id(row), strategy="coordinated", expectation=cases[row["case_id"]]["expectation"],
            outcome=None, actual_request_status=None, fixed_case_result_path=None, fixed_case_result_sha256=None,
            verification_result_path=None, verification_result_sha256=None, trace_path=None, trace_sha256=None,
            unindexed_trace_present=False, unindexed_trace_path=None, unindexed_trace_sha256=None,
            not_run_reason=None if row["selected"] else "case_not_selected", error=None, timing=None)
    return rows


def _read_json(path):
    value = json.loads(path.read_text())
    require(isinstance(value, dict), "Expected an evidence JSON object: " + str(path))
    return value


def _collect_row(row, case, directory, report_path, expected_suite_sha256):
    """Retain available raw evidence even when preparation or infrastructure fails."""
    verification = None
    if report_path is not None and report_path.is_file():
        row.update(verification_result_path=str(report_path), verification_result_sha256=file_sha256(report_path))
        verification = _read_json(report_path)
        row["verification"] = {name: deepcopy(verification.get(name)) for name in (
            "run_id", "ros_domain_id", "status", "motion_checks_passed", "physics_checks_passed",
            "shutdown_clean", "launch_return_code", "process_group_gone", "error", "failure_stage")}
    path = directory / "fixed-case.json"
    if not path.is_file():
        return verification, None
    row.update(fixed_case_result_path=str(path), fixed_case_result_sha256=file_sha256(path))
    result = _read_json(path)
    require(result.get("schema_version") == 1 and result.get("case_id") == row["case_id"]
            and result.get("variant") == row["variant"] and result.get("repeat") == row["repeat"]
            and result.get("suite_sha256") == expected_suite_sha256,
            "Fixed case result belongs to another suite row")
    row["fixed_case_status"] = result.get("status")
    row["maintenance_requests"] = len(result.get("maintenance", []))
    row["preparation_error"] = deepcopy(result.get("error"))
    scored = result.get("scored")
    if scored is None and result.get("scored_attempted") is True:
        declared = result.get("scored_request_id")
        if isinstance(declared, str) and re.fullmatch(r"[0-9a-f]{32}", declared):
            orphan = (directory / (declared + ".json")).resolve()
            require(orphan.is_relative_to(directory.resolve()), "Unindexed trace escapes its row directory")
            if orphan.is_file():
                # A raw file is insufficient to establish indexed request
                # provenance or successful delivery. Keep it separate, never
                # feed it to the scored summary or retry the declared request.
                row.update(unindexed_trace_present=True, unindexed_trace_path=str(orphan),
                           unindexed_trace_sha256=file_sha256(orphan))
    if scored is not None:
        require(isinstance(scored, dict), "Invalid scored result object")
        row.update(request_id=scored.get("request_id"), actual_request_status=scored.get("status"),
                   outcome=scored.get("outcome"), failure_kind=scored.get("failure_kind"),
                   failure_stage=scored.get("failure_stage"))
        filename = scored.get("trace_file")
        require(isinstance(row["request_id"], str) and filename == row["request_id"] + ".json",
                "Scored trace filename does not match its request ID")
        trace_path = (directory / filename).resolve()
        require(trace_path.is_relative_to(directory.resolve()) and trace_path.is_file(),
                "Scored raw trace is missing or escapes its row directory")
        row.update(trace_path=str(trace_path), trace_sha256=file_sha256(trace_path))
        require(row["trace_sha256"] == scored.get("trace_sha256"), "Scored trace SHA256 mismatch")
        trace = _read_json(trace_path)
        require(trace.get("request_id") == row["request_id"] and trace.get("status") == row["actual_request_status"]
                and trace.get("command", {}).get("strategy") == "coordinated"
                and trace.get("command", {}).get("target_rad") == case["target_rad"],
                "Scored trace identity, strategy, status or fixed target mismatch")
    return verification, result


def _completed_evaluation(verification, result, row):
    require(verification is not None and result is not None, "Missing row lifecycle or node result")
    require(verification.get("status") == "passed" and verification.get("motion_checks_passed") is True
            and verification.get("physics_checks_passed") is True and verification.get("shutdown_clean") is True
            and verification.get("launch_return_code") == 0 and verification.get("process_group_gone") is True,
            "Preparation, infrastructure, physics or cleanup failed; later rows must not run")
    require(result.get("status") == "completed" and result.get("scored") is not None
            and row.get("outcome") in OUTCOMES,
            "Fixed row did not finish exactly one judged scored request")
    if row["expectation"] == "invalid_goal":
        require(row["outcome"] == "expected_invalid_goal" and row["actual_request_status"] == "failed",
                "Invalid-goal row did not finish with its expected rejection")
    else:
        require((row["outcome"], row["actual_request_status"]) in
                (("motion_succeeded", "succeeded"), ("planning_failed", "failed")),
                "Valid-goal row has an unexpected result category")


def _counts(rows):
    return dict(planned=len(rows), selected=sum(row["selected"] for row in rows),
        completed=sum(row["status"] == "completed" for row in rows),
        aborted=sum(row["status"] == "aborted" for row in rows),
        not_run=sum(row["status"] == "not_run" for row in rows),
        motion_succeeded=sum(row.get("outcome") == "motion_succeeded" and row["status"] == "completed" for row in rows),
        planning_failed=sum(row.get("outcome") == "planning_failed" and row["status"] == "completed" for row in rows),
        expected_invalid_goal=sum(row.get("outcome") == "expected_invalid_goal" and row["status"] == "completed" for row in rows))


def run_suite(suite, cells, args, directory):
    """Execute each selected row once; stop on the first non-evaluation failure."""
    digest = suite_hash(suite)
    rows = _full_rows(suite, args.repeats, args.cases, domain_start=args.domain)
    require(any(row["selected"] for row in rows), "Select at least one fixed case")
    raw = dict(schema_version=1, id="fixed-motion-raw-v1", status="running", run_id=uuid.uuid4().hex,
        started_utc=_utc_now(), completed_utc=None, suite_sha256=digest, suite_file="suite.json",
        domain_start=args.domain, domain_range=dict(start=args.domain,
            end=args.domain + sum(row["selected"] for row in rows) - 1, count=sum(row["selected"] for row in rows)),
        repeats=args.repeats, requested_case_ids=deepcopy(args.cases),
        input_suite_file=str(args.suite), input_suite_file_sha256=args.suite_file_sha256,
        compute_budget_wall_sec=suite["compute_budget_wall_sec"], initial_tolerance_rad=suite["initial_tolerance_rad"],
        source_sha256={name: file_sha256(Path(__file__).parent / name)
                       for name in ("benchmark_fixed_motion.py", "fixed_execution.py", "fixed_suite.py", "fixed_results.py")},
        timing_source_sha256=file_sha256(Path(__file__).parents[1] / "timing.py"),
        rows=rows, counts=_counts(rows), current_row_id=None, error=None,
        note="Raw local suite run. Every selected row gets a fresh simulator; preparation is unscored. Expected planning failures continue; infrastructure/action/physics failures abort remaining rows. No result is automatically published.")
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "rows").mkdir()
    atomic_json(directory / "suite.json", suite)
    path = directory / "fixed-benchmark.json"

    def save():
        raw["counts"] = _counts(rows)
        raw["timing_summary"] = _timing_summary(rows)
        atomic_json(path, raw)

    save()  # Persist the complete catalog before any row preflight or launch.
    cases = {case["case_id"]: case for case in suite["cases"]}
    planned_catalog = deepcopy(rows)
    compact_rows = [summarize_row(suite, row, fixed_case_raw=None, trace_raw=None, verification_raw=None)
                    for row in planned_catalog]
    raw["results"] = compact_rows
    raw["summary"] = aggregate_results(suite, planned_catalog, compact_rows)
    raw["numeric_host_load_start"] = _numeric_host_load()
    save()

    def refresh_summary(row, output, report_path):
        case_path = output / "fixed-case.json"
        trace_path = Path(row["trace_path"]) if row.get("trace_path") else None
        compact_rows[row["index"]] = summarize_row(suite, planned_catalog[row["index"]],
            fixed_case_raw=case_path.read_bytes() if case_path.is_file() else None,
            trace_raw=trace_path.read_bytes() if trace_path is not None and trace_path.is_file() else None,
            verification_raw=report_path.read_bytes() if report_path is not None and report_path.is_file() else None)
        raw["summary"] = aggregate_results(suite, planned_catalog, compact_rows)

    exit_code = 0
    try:
        for row in rows:
            if not row["selected"]:
                continue
            raw["current_row_id"] = row["row_id"]
            row["status"], row["started_utc"] = "running", _utc_now()
            outer_started = time.perf_counter()
            row["numeric_host_load_start"] = _numeric_host_load()
            save()
            case = cases[row["case_id"]]
            cell = cells[case["workcell"]]
            output = directory / "rows" / f'{row["index"]:04d}-{row["row_id"]}'
            capture = {}
            prefix = "fixed_suite_" + raw["run_id"] + "_" + str(row["index"]) + "_"
            alias = ROOT / "runtime/validation" / (prefix + cell.id + "_headless.json")
            report_path = None
            row_code = None
            try:
                # Never reuse a row domain: destroyed DDS nodes can remain in
                # discovery briefly after a clean shutdown. Do not ignore them.
                require_unused_domain(row["domain"])
                check_workcell(cell)  # Existing generated models only; no runtime build.
                launch_args = argparse.Namespace(robot=cell.id, scene="default", gui=False,
                    stop_mode="interrupt", domain=row["domain"], require_gpu=False, screenshot_dir=None)

                def on_launch(report, profile, child_args, process):
                    capture["report"] = report
                    row["launch_created_wall_offset_sec"] = time.perf_counter() - outer_started

                def factory(profile, progress):
                    def measured_progress(stage):
                        if stage == "fixed_case_scored_request":
                            row["unscored_scoring_ready_wall_offset_sec"] = time.perf_counter() - outer_started
                        progress(stage)
                    node = FixedCaseExecution(profile, measured_progress, case, row, output=output, suite_sha256=digest)
                    row["node_created_wall_offset_sec"] = time.perf_counter() - outer_started
                    row["timing"] = _observe_publications(node)
                    return node

                row_code = run_verification(launch_args, ROOT, rclpy, factory, Trigger,
                    profile_loader=lambda *a, **kw: load_workcell(cell.id, root=kw["root"]),
                    validator=check_workcell,
                    command_builder=lambda profile, child_args: ["bash", str(ROOT / "run_scene.sh"),
                        "workcell:=" + profile.id, "gui:=false", "rviz:=false",
                        "planning_case:=" + ("eval_post" if profile.id == POST_ID else "dual_arm_crossing")],
                    report_prefix=prefix, launch_environment=diagnostic_environment,
                    finalize=finalize_physics, on_launch=on_launch)
                report_path = capture["report"].path if "report" in capture else alias
                verification, result = _collect_row(row, case, output, report_path, digest)
                _retain_timing(row, result)
                row["verification_exit_code"] = row_code
                refresh_summary(row, output, report_path)
                save()  # Preserve any scored request before checking the outer lifecycle.
                require(row_code == 0, "Owned row lifecycle failed; raw evidence retained")
                _completed_evaluation(verification, result, row)
                require(compact_rows[row["index"]]["status"] == "completed",
                        "Independent evidence summary did not accept this row as a completed evaluation")
                row.update(status="completed", completed_utc=_utc_now(),
                           outer_wall_sec=time.perf_counter() - outer_started,
                           numeric_host_load_end=_numeric_host_load())
                save()
            except BaseException as error:
                # run_verification normally handles failures and cleans up itself.
                # Collect its report even when launch/preparation never reached the
                # callback or node result. The alias is unique to this suite row.
                report_path = report_path or (capture["report"].path if "report" in capture else alias)
                try:
                    _, result = _collect_row(row, case, output, report_path, digest)
                    _retain_timing(row, result)
                except Exception as evidence_error:
                    row["evidence_error"] = dict(type=type(evidence_error).__name__, message=str(evidence_error))
                try:
                    refresh_summary(row, output, report_path)
                except Exception as summary_error:
                    row["summary_error"] = dict(type=type(summary_error).__name__, message=str(summary_error))
                row.update(status="aborted", completed_utc=_utc_now(), verification_exit_code=row_code,
                           outer_wall_sec=time.perf_counter() - outer_started,
                           numeric_host_load_end=_numeric_host_load(),
                           error=dict(type=type(error).__name__, message=str(error)))
                raw.update(status="aborted", error=dict(row_id=row["row_id"], **row["error"]))
                exit_code = 130 if isinstance(error, KeyboardInterrupt) or row_code == 130 else 1
                for remaining in rows:
                    if remaining["selected"] and remaining["status"] == "not_run":
                        remaining["not_run_reason"] = "suite_aborted_after_" + row["row_id"]
                save()
                break
        else:
            raw["status"] = "completed"
    except BaseException as error:
        # Also cover interruption or bookkeeping failure between owned row runs.
        current = next((row for row in rows if row["row_id"] == raw["current_row_id"]), None)
        detail = dict(type=type(error).__name__, message=str(error))
        if current is not None and current["status"] == "running":
            current.update(status="aborted", completed_utc=_utc_now(), error=detail)
        raw.update(status="aborted", error=dict(row_id=raw["current_row_id"], **detail))
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        for remaining in rows:
            if remaining["selected"] and remaining["status"] == "not_run":
                remaining["not_run_reason"] = "suite_aborted"
    finally:
        raw["completed_utc"] = _utc_now()
        raw["current_row_id"] = None
        raw["numeric_host_load_end"] = _numeric_host_load()
        save()
    print(f'{raw["status"].upper()}: {path}', flush=True)
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="List fixed cases and variants without launching ROS")
    parser.add_argument("--suite", type=Path, default=ROOT / "config/evaluations/fixed_motion_v1.json",
                        help="Immutable fixed-suite JSON or an identical copy")
    parser.add_argument("--cases", nargs="+", help="Selected case IDs, separated by spaces or commas; retain all others as not_run")
    parser.add_argument("--repeats", type=int, default=3, help="Fresh paired repetitions, 1..10 (default: 3)")
    parser.add_argument("--domain", type=int, default=171, help="Starting unused ROS domain; each selected row uses the next unique domain (default: 171)")
    parser.add_argument("--output", type=Path, help="New output directory under runtime/; existing paths are refused")
    args = parser.parse_args()
    if not 1 <= args.repeats <= 10 or not 0 <= args.domain <= 232:
        parser.error("Repeats must be 1..10 and ROS domain 0..232")
    args.cases = _requested_cases(args.cases)
    try:
        cells = {name: load_workcell(name) for name in (BASE_ID, POST_ID)}
        args.suite = args.suite.resolve()
        declared, args.suite_file_sha256 = _load_declared_suite(args.suite, cells)
        _full_rows(declared, args.repeats, args.cases, domain_start=args.domain)  # Validate range before output creation.
        if args.list:
            print(json.dumps(dict(suite_id=declared["id"], suite_sha256=suite_hash(declared),
                input_suite_file_sha256=args.suite_file_sha256,
                compute_budget_wall_sec=declared["compute_budget_wall_sec"], variants=declared["variants"],
                validation="Declarations and pinned sources only; generated-model and live checks happen before execution",
                cases=[{name: case[name] for name in ("case_id", "label", "category", "workcell", "expectation")}
                       for case in declared["cases"]]), ensure_ascii=False, indent=2))
            return 0
        metadata = {name: check_workcell(cell) for name, cell in cells.items()}
        suite = check_suite(declared, cells[BASE_ID], cells[POST_ID], metadata)
        default = ROOT / "runtime/validation/fixed-suites" / (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "-" + uuid.uuid4().hex[:8])
        directory = (args.output or default).resolve()
        if directory.exists() or not directory.is_relative_to((ROOT / "runtime").resolve()):
            parser.error("Output must be a new directory under runtime/")
        return run_suite(suite, cells, args, directory)
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
