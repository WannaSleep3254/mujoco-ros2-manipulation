"""Recompute private fixed-suite evidence and emit a strict public projection.

Only original input bytes are accepted. The public artifact contains the complete
planned catalog, measured compact values, source digests and a curated measurement
configuration. Raw paths, process/environment telemetry and error text never pass
through this adapter. Report generation is a separate operation on this artifact.
"""
import argparse
from copy import deepcopy
import getpass
import gzip
import hashlib
import json
from pathlib import Path
import re
import socket

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.evidence_export import audit_public_evidence, encoded_evidence
from mujoco_ros2_manipulation.workcells.fixed_results import aggregate_results, summarize_row
from mujoco_ros2_manipulation.workcells.fixed_suite import BASE_ID, POST_ID, make_suite, planned_rows, suite_hash


SOURCE_KEYS = ("fixed_case_raw", "trace_raw", "verification_raw", "session_raw", "model_manifest_raw", "unindexed_trace_raw")
CODE_FILES = ("benchmark_fixed_motion.py", "fixed_execution.py", "fixed_suite.py", "fixed_results.py")
IMPLEMENTATION_FILES = {"motion_server.py", "motion_client.py", "motion_request.py", "request_trace.py",
    "verify.py", "timing.py", "path_quality.py", "planning_budget.py", "contracts.py", "motion_diagnostics.py",
    "plan_selection.py", "spatial_candidates.py"}
ROW_FIELDS = ("schema_version", "suite_id", "suite_sha256", "row_id", "index", "case_id", "workcell", "workcell_sha256",
    "variant", "repeat", "max_plan_candidates", "strategy", "expectation", "selected", "status", "attempted", "scored",
    "declared_request_id", "request_id", "request_status", "outcome", "fixed_status", "verification_status",
    "delivery_unknown", "initial_error_rad", "not_run_reason")
METRIC_FIELDS = ("compute_wall_sec", "compute_limit_wall_sec", "budget_exceeded", "planning_rpc_recorded_calls",
    "planning_rpc_wall_sec", "server_planning_sec", "candidate_count", "selected_attempt", "selected_proposal_kind",
    "planned_candidate_attempt", "candidate_duration_sec", "dispatched_goals", "first_motion_wall_sec",
    "actions_done_wall_sec", "settled_wall_sec", "terminal_wall_sec", "settling_duration_wall_sec")
PATH_FIELDS = ("status", "samples", "minimum_distance_m", "prefix_minimum_distance_m", "duration_sec",
               "sample_dt_max_sec", "joint_travel_rad", "tip_frame_path_m")
CHECK_FIELDS = ("run_id", "motion_checks_passed", "physics_checks_passed", "shutdown_clean", "process_group_gone",
    "physics_complete", "contact_steps", "physics_steps", "invalid_values", "model_manifest_sha256", "physics_report_sha256")
FAILURE_KINDS = {None, "cancelled", "budget_exhausted", "candidate_exhausted", "departure_exhausted", "invalid_start",
                 "invalid_goal", "planner_failed", "state_changed", "execution_failed", "planning_error"}
FAILURE_STAGES = {None, "state_snapshot", "endpoint_clearance", "endpoint_validity", "spatial_candidate_search",
    "safety_checks", "state_recheck", "final_state_recheck", "dispatch_to_action_results", "settling", "observed_clearance",
    "planning:both_arms", "planning:left_arm", "planning:right_arm", "computation_started", "dispatch"}


def _require(value, message):
    if not value:
        raise ValueError(message)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _digest(value, *, length=64):
    _require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{" + str(length) + r"}", value), "Invalid source digest")
    return value


def _pairs(items):
    value = {}
    for key, item in items:
        _require(key not in value, "Duplicate source JSON field")
        value[key] = item
    return value


def _load(raw):
    if raw is None:
        return None
    _require(type(raw) is bytes, "Source records must be original bytes")
    value = json.loads(raw, object_pairs_hook=_pairs)
    _require(isinstance(value, dict), "Expected source JSON object")
    return value


def _select(value, fields):
    _require(isinstance(value, dict), "Expected whitelisted source object")
    return {key: deepcopy(value[key]) for key in fields if key in value}


def _row_id(row):
    return f'r{row["repeat"]:02d}-{row["case_id"]}-{row["variant"]}'


def _catalog(suite, raw):
    _require(type(raw.get("schema_version")) is int and raw["schema_version"] == 1 and raw.get("id") == "fixed-motion-raw-v1"
             and raw.get("status") in ("completed", "aborted"), "Only terminal fixed-suite runs can be exported")
    _require(raw.get("suite_sha256") == suite_hash(suite) and raw.get("compute_budget_wall_sec") == 5.0
             and raw.get("initial_tolerance_rad") == .0015, "Run suite/budget/tolerance binding mismatch")
    _require(type(raw.get("domain_start")) is int and 0 <= raw["domain_start"] <= 232, "Invalid starting domain")
    rows = planned_rows(suite, repeats=raw["repeats"], case_ids=raw.get("requested_case_ids"))
    ordinal = 0
    for row in rows:
        case = next(case for case in suite["cases"] if case["case_id"] == row["case_id"])
        row.update(row_id=_row_id(row), strategy="coordinated", expectation=case["expectation"],
                   domain=raw["domain_start"] + ordinal if row["selected"] else None)
        ordinal += bool(row["selected"])
    _require(ordinal > 0 and raw["domain_start"] + ordinal - 1 <= 232, "Invalid selected domain range")
    _require(raw.get("domain_range") == dict(start=raw["domain_start"], end=raw["domain_start"] + ordinal - 1, count=ordinal),
             "Run domain range changed")
    _require(isinstance(raw.get("rows"), list) and len(raw["rows"]) == len(rows), "Dropped or extra planned rows")
    for expected, original in zip(rows, raw["rows"]):
        _require(all(original.get(key) == expected[key] and type(original.get(key)) is type(expected[key])
                     for key in ("case_id", "variant", "max_plan_candidates", "repeat", "index", "selected", "row_id", "strategy", "expectation", "domain")),
                 "Planned catalog identity/order/domain mismatch")
        _require(original.get("status") in ("completed", "aborted", "not_run"), "Nonterminal raw row")
    return rows


def _bound_source(original, source, raw_key, digest_key, *, parse_json=True):
    value = source[raw_key]
    declared = original.get(digest_key)
    _require((value is None) == (declared is None), "Missing or unexpected row source bytes")
    if value is not None:
        _require(_sha(value) == _digest(declared), "Row source SHA256 mismatch")
    return _load(value) if parse_json else None


def _session_configuration(suite, case, fixed, verification, session, manifest, source, code_sources):
    if fixed is None:
        _require(session is None and manifest is None, "Unattributed session/model evidence")
        return None
    _require(session is not None and manifest is not None, "Measured node lacks original session/model bytes")
    _require(isinstance(session.get("session_id"), str) and re.fullmatch(r"[0-9a-f]{32}", session["session_id"])
             and session.get("session_id") == fixed.get("session_id") and session.get("workcell") == case["workcell"]
             and session.get("model_manifest") == manifest
             and manifest.get("workcell_sha256") == suite["workcell_fingerprints"][case["workcell"]]
             and isinstance(manifest.get("joints"), list) and set(manifest["joints"]) == set(case["initial_rad"]),
             "Session/model/workcell identity mismatch")
    _require(session.get("policy") == fixed.get("policy") and session.get("compute_budget_sec") == 5.0
             and session.get("max_plan_candidates") == fixed["scored_settings"]["max_plan_candidates"],
             "Session/row policy, scored budget or candidate cap mismatch")
    if verification is not None:
        _require(verification.get("model_manifest_sha256") == _sha(source["model_manifest_raw"]),
                 "Measured verification/model raw SHA256 mismatch")
    _require(fixed.get("source_sha256") == code_sources["fixed_execution.py"], "Measured execution implementation mismatch")
    commit = _digest(session.get("code_commit"), length=40)
    _require(type(session.get("code_worktree_dirty")) is bool, "Missing measured worktree provenance")
    implementation = session.get("implementation_sha256")
    _require(isinstance(implementation, dict) and IMPLEMENTATION_FILES.issubset(implementation),
             "Missing measured implementation digests")
    for name, digest in implementation.items():
        _require(isinstance(name, str) and re.fullmatch(r"[a-z][a-z0-9_]*\.py", name), "Invalid measured implementation name")
        _digest(digest)
    runtime, host, evaluator = session.get("runtime", {}), session.get("host", {}), session.get("evaluator", {})
    environment = dict(os=runtime.get("os"), cpu_model=host.get("cpu"), python=runtime.get("python"),
                       middleware=runtime.get("middleware"), native_engine=evaluator.get("engine"))
    _require(all(isinstance(value, str) and value and len(value) <= 300 for value in environment.values()),
             "Missing or malformed curated measured environment")
    if verification is not None and verification.get("physics", {}).get("mujoco_version") is not None:
        _require(environment["native_engine"] == "MuJoCo " + str(verification["physics"]["mujoco_version"]),
                 "Native engine version differs across measured records")
    return dict(code_commit=commit, measurement_worktree_dirty=session["code_worktree_dirty"],
                implementation_sha256=deepcopy(implementation), evaluation_source_sha256=deepcopy(code_sources),
                environment=environment, native_distance_source_sha256=_digest(evaluator.get("source_sha256")),
                native_library_sha256=_digest(evaluator.get("library_sha256")))


def _public_row(compact, source, original, case):
    result = _select(compact, ROW_FIELDS)
    result["source_sha256"] = deepcopy(compact["source_sha256"])
    for label, key in (("session", "session_raw"), ("model_manifest", "model_manifest_raw")):
        result["source_sha256"][label] = None if source[key] is None else _sha(source[key])
    result["failure_kind"] = compact["failure_kind"] if compact["failure_kind"] in FAILURE_KINDS else "other"
    result["failure_stage"] = compact["failure_stage"] if compact["failure_stage"] in FAILURE_STAGES else "other"
    result["verification"] = None if compact["verification"] is None else _select(compact["verification"], CHECK_FIELDS)
    result["metrics"] = None
    if compact["metrics"] is not None:
        metrics = result["metrics"] = _select(compact["metrics"], METRIC_FIELDS)
        metrics["action_results"] = [_select(row, ("status", "error_code")) for row in compact["metrics"]["action_results"]]
        _require(metrics.get("selected_proposal_kind") in (None, "coordinated_rrt", "arm_order_waypoint"),
                 "Unknown selected proposal kind")
        owners = {name.split("_", 1)[0] for name in case["initial_rad"]}
        for key in ("planned", "observed"):
            metrics[key] = _select(compact["metrics"][key], PATH_FIELDS)
            tips = metrics[key].get("tip_frame_path_m")
            _require(tips is None or (isinstance(tips, dict) and set(tips) == owners), "Unexpected robot names in path metrics")
    orphan = source["unindexed_trace_raw"]
    result["unindexed_trace_present"] = orphan is not None
    result["unindexed_trace_sha256"] = None if orphan is None else _sha(orphan)
    _require(result["unindexed_trace_present"] == original.get("unindexed_trace_present", False), "Unindexed presence mismatch")
    return result


def export_fixed_motion(suite_raw, benchmark_raw, sources, *, expected_suite,
                        report_id="2026-10-23-fixed-motion", forbidden_tokens=()):
    """Return a detached safe artifact after exact source/summary recomputation.

    ``sources`` maps every stable row ID to all six SOURCE_KEYS, with bytes or
    null values. ``expected_suite`` comes from the trusted public factory. No
    caller-supplied summary, report date or current Git HEAD is trusted here.
    """
    _require(isinstance(report_id, str) and len(report_id) <= 100
             and re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", report_id), "Invalid public report ID")
    suite, raw = _load(suite_raw), _load(benchmark_raw)
    _require(suite == expected_suite and suite_hash(suite) == suite_hash(expected_suite), "Source suite differs from the public factory")
    catalog = _catalog(suite, raw)
    _require(isinstance(sources, dict) and set(sources) == {_row_id(row) for row in catalog}, "Missing/extra row source bundle")
    code_sources = raw.get("source_sha256")
    _require(isinstance(code_sources, dict) and set(code_sources) == set(CODE_FILES), "Evaluation source digests are incomplete")
    for digest in code_sources.values():
        _digest(digest)
    compact_rows, public_rows, configurations, tokens, session_ids = [], [], [], list(forbidden_tokens), []
    cases = {case["case_id"]: case for case in suite["cases"]}
    for row, original in zip(catalog, raw["rows"]):
        source = sources[row["row_id"]]
        _require(isinstance(source, dict) and set(source) == set(SOURCE_KEYS), "Unexpected/missing row source fields")
        for value in source.values():
            _require(value is None or type(value) is bytes, "Row sources must be original bytes or null")
        _require(row["selected"] or all(value is None for value in source.values()), "Unselected row has private execution sources")
        fixed = _bound_source(original, source, "fixed_case_raw", "fixed_case_result_sha256")
        verification = _bound_source(original, source, "verification_raw", "verification_result_sha256")
        _bound_source(original, source, "trace_raw", "trace_sha256")
        _bound_source(original, source, "unindexed_trace_raw", "unindexed_trace_sha256", parse_json=False)
        compact = summarize_row(suite, row, fixed_case_raw=source["fixed_case_raw"],
                                trace_raw=source["trace_raw"], verification_raw=source["verification_raw"])
        compact_rows.append(compact)
        _require(original.get("request_id") == compact["request_id"]
                 and original.get("actual_request_status") == compact["request_status"]
                 and (original.get("outcome") == compact["outcome"] if compact["scored"]
                      else original.get("outcome") in (None, compact["outcome"])),
                 "Raw/checked row request outcome mismatch")
        configuration = _session_configuration(suite, cases[row["case_id"]], fixed, verification,
            _load(source["session_raw"]), _load(source["model_manifest_raw"]), source, code_sources)
        if configuration is not None:
            configurations.append(configuration)
            session_ids.append(fixed["session_id"])
            host = _load(source["session_raw"]).get("host", {})
            tokens.extend(host.get(key) for key in ("hostname", "username", "user"))
        public_rows.append(_public_row(compact, source, original, cases[row["case_id"]]))
    aggregate = aggregate_results(suite, catalog, compact_rows)
    _require(raw.get("results") == compact_rows and raw.get("summary") == aggregate,
             "Stored rows/summary differ from a full raw-byte recomputation")
    counts = dict(planned=len(catalog), selected=sum(row["selected"] for row in catalog),
        completed=sum(row["status"] == "completed" for row in raw["rows"]), aborted=sum(row["status"] == "aborted" for row in raw["rows"]),
        not_run=sum(row["status"] == "not_run" for row in raw["rows"]),
        motion_succeeded=sum(row["status"] == "completed" and row.get("outcome") == "motion_succeeded" for row in raw["rows"]),
        planning_failed=sum(row["status"] == "completed" and row.get("outcome") == "planning_failed" for row in raw["rows"]),
        expected_invalid_goal=sum(row["status"] == "completed" and row.get("outcome") == "expected_invalid_goal" for row in raw["rows"]))
    _require(raw.get("counts") == counts, "Stored run counts do not match its complete catalog")
    if raw["status"] == "completed":
        _require(all(not row["selected"] or row["status"] == "completed" for row in compact_rows), "Completed run contains an incomplete selected row")
    _require(len(session_ids) == len(set(session_ids)), "A simulator session was reused across fixed rows")
    _require(not configurations or all(value == configurations[0] for value in configurations),
             "Measured code/environment changed between rows; do not pool this run")
    public_catalog = [_select(row, ("row_id", "case_id", "variant", "repeat", "index", "selected", "max_plan_candidates", "strategy", "expectation")) for row in catalog]
    artifact = dict(schema_version=1, artifact_kind="fixed-motion-public-evidence-v1", report_id=report_id,
        status=raw["status"], suite=deepcopy(suite), suite_sha256=suite_hash(suite), planned_catalog=public_catalog,
        rows=public_rows, summary=deepcopy(aggregate), measurement=None if not configurations else configurations[0],
        source_sha256=dict(suite=_sha(suite_raw), benchmark=_sha(benchmark_raw)),
        scope=dict(fixed_cases=len(suite["cases"]), valid_goal_cases=sum(case["expectation"] == "valid_goal" for case in suite["cases"]),
            expected_invalid_goal_cases=sum(case["expectation"] == "invalid_goal" for case in suite["cases"]), repeats=raw["repeats"],
            candidate_caps=[1, 12], compute_budget_wall_sec=5., initial_tolerance_rad=.0015,
            sample_role="full_evaluation" if raw["repeats"] == 3 and all(row["selected"] for row in catalog) and raw["status"] == "completed" else "partial_or_diagnostic"),
        limitations=["Fixed paired inputs; OMPL internal randomness and OS scheduling are not fixed.",
            "Variants do not share a sampled RRT proposal. First-candidate path/time differences cannot establish a causal effect of the candidate cap.",
            "Per-case fixed repeats do not establish tail percentiles; no p95/p99 is reported.",
            "Cases share models and preparation and are correlated; this is not a general robot or planner benchmark.",
            "Expected-invalid goals have a separate denominator. Not-run and delivery-unknown rows remain visible.",
            "Partial sampled minima are prefix evidence, never a complete-path minimum or a continuous safety certificate."],
        export_note="Strict public projection; complete original bytes remain local. Raw paths, process/environment telemetry and error messages omitted.")
    artifact["privacy_audit"] = audit_public_evidence(artifact, forbidden_tokens=tokens)
    return artifact


def _private_file(path, root):
    resolved = Path(path).resolve()
    _require(resolved.is_relative_to(root.resolve()) and resolved.is_file(), "Private source file is missing or outside the allowed runtime root")
    return resolved.read_bytes()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="Terminal local fixed-benchmark.json")
    parser.add_argument("--output", type=Path, required=True, help="New .json or deterministic .json.gz artifact")
    parser.add_argument("--report-id", help="Plain publication ID; default derives from completed_utc")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output exists; public export never overwrites source or prior evidence")
    private_root = ROOT / "runtime"
    benchmark_raw = _private_file(args.source, private_root)
    raw = _load(benchmark_raw)
    _require(raw.get("suite_file") == "suite.json", "Unexpected local suite filename")
    suite_raw = _private_file(args.source.resolve().parent / "suite.json", private_root)
    suite = _load(suite_raw)
    expected = make_suite(load_workcell(BASE_ID), load_workcell(POST_ID))
    source_bundle = {}
    for row in raw["rows"]:
        source = dict.fromkeys(SOURCE_KEYS)
        for key, field in (("fixed_case_raw", "fixed_case_result_path"), ("trace_raw", "trace_path"),
                           ("verification_raw", "verification_result_path"), ("unindexed_trace_raw", "unindexed_trace_path")):
            if row.get(field) is not None:
                source[key] = _private_file(row[field], private_root)
        if row.get("fixed_case_result_path") is not None:
            source["session_raw"] = _private_file(Path(row["fixed_case_result_path"]).parent / "session.json", private_root)
            case = next(case for case in suite["cases"] if case["case_id"] == row["case_id"])
            source["model_manifest_raw"] = _private_file(private_root / "generated/workcells" / case["workcell"] / "source.json", private_root)
        source_bundle[_row_id(row)] = source
    completed = raw.get("completed_utc")
    report_id = args.report_id
    if report_id is None:
        from datetime import date
        _require(isinstance(completed, str), "Pass --report-id when terminal UTC is absent")
        report_id = date.fromisoformat(completed[:10]).isoformat() + "-fixed-motion"
    result = export_fixed_motion(suite_raw, benchmark_raw, source_bundle, expected_suite=expected, report_id=report_id,
                                 forbidden_tokens=(getpass.getuser(), socket.gethostname()))
    encoded = encoded_evidence(result)
    if args.output.suffix == ".gz":
        encoded = gzip.compress(encoded, mtime=0)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as stream:
        stream.write(encoded)
    print("Wrote one audited public projection; complete source files left unchanged")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
