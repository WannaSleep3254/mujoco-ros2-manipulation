"""Evidence-driven summaries for the separate planner/cache experiment."""
from copy import deepcopy
import math
import statistics

from .planner_cache_experiment import (CONDITIONS, GOAL_TOLERANCE_RAD, COMPUTE_BUDGET_SEC,
                                       COMPUTE_SCOPE, PRIMARY_LATENCY_METRIC)


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def require_validated_cache_hit(trace):
    """Functional reuse proof excludes a rejected hit followed by fresh fallback."""
    cache = trace.get("trajectory_cache") or {}
    lookup = cache.get("lookup") or {}
    _require(trace.get("status") == "succeeded" and lookup.get("hit") is True
        and cache.get("reason") == "hit" and not cache.get("fallback")
        and cache.get("store") is None and lookup.get("full_revalidation_required") is True
        and _finite(cache.get("full_revalidation_wall_sec"))
        and trace.get("planned_clearance", {}).get("complete") is True
        and trace.get("moveit_safety_samples", 0) > 0,
        "Functional reuse requires an accepted cache hit with full revalidation; fresh fallback is separate")


def summarize_trace(experiment, row, trace):
    """Retain failures; refuse foreign goals, dispatches and claimed planner IDs."""
    condition = next(item for item in CONDITIONS if item["id"] == row["condition"])
    _require(type(trace) is dict and trace.get("mode") == "plan_only"
        and trace.get("command", {}).get("target_rad") == experiment["case"]["target_rad"]
        and trace.get("command", {}).get("strategy") == "coordinated"
        and trace.get("dispatched_goals") == 0, "Scored trace is foreign or dispatched an action")
    _require(trace.get("status") in ("succeeded", "failed"), "Cancellation/rejection cannot be hidden as a scored planning result")
    configuration = trace.get("planning_configuration", {})
    _require(configuration.get("requested_planner_id") == condition["planner_id"], "Requested planner evidence differs")
    _require(configuration.get("trajectory_cache_enabled") is condition["cache"], "Cache condition differs from the actual server")
    _require(configuration.get("max_plan_candidates") == 1 and configuration.get("compute_budget_wall_sec") == COMPUTE_BUDGET_SEC
        and configuration.get("goal_tolerance_rad") == GOAL_TOLERANCE_RAD,
        "Scored settings differ from the shared comparison contract")
    _require(configuration.get("budget_includes_review") is True
        and configuration.get("compute_scope") == COMPUTE_SCOPE
        and experiment["comparison_policy"].get("budget_includes_review") is True
        and experiment["comparison_policy"].get("compute_scope") == COMPUTE_SCOPE,
        "Scored computation scope must include final review identity in every condition")
    actual = trace.get("initial_state_rad")
    initial = experiment["case"]["initial_rad"]
    _require(type(actual) is dict and set(actual) == set(initial)
        and all(type(value) in (int, float) and math.isfinite(value) for value in actual.values()),
        "Scored trace lacks its actual complete observed start")
    start_error = max(abs(actual[name]-initial[name]) for name in initial)
    _require(start_error <= experiment["comparison_policy"]["initial_tolerance_rad"], "Actual start differs from the declared paired baseline")
    cache = deepcopy(trace.get("trajectory_cache"))
    if condition["cache"] and trace["status"] == "succeeded":
        _require(type(cache) is dict and cache.get("enabled") is True and cache.get("eligible") is True,
                 "Successful cache condition lacks its complete cache receipt")
        hit = cache.get("lookup", {}).get("hit")
        _require(hit is (condition["expected_cache"] == "hit"), "Cold/warm result does not match its declared cache condition")
        if hit:
            _require(_finite(cache.get("full_revalidation_wall_sec"))
                and cache.get("lookup", {}).get("full_revalidation_required") is True,
                "Warm candidate skipped complete validation")
            if cache.get("fallback"):
                _require(cache.get("reason") == "validation_rejected_fallback"
                    and type(cache.get("rejected_candidate")) is dict
                    and cache.get("eviction", {}).get("evicted") is True
                    and _finite(cache.get("fallback_planning_wall_sec")) and trace.get("plans"),
                    "Warm fallback lacks a rejected candidate, eviction and fresh planning evidence")
            else:
                _require(cache.get("store") is None and cache.get("reason") == "hit",
                         "Accepted warm hit refreshed its original source entry")
    events = {item["name"]: item["wall_sec"] for item in trace.get("events", [])}
    started, done = events.get("computation_started"), events.get("computation_done")
    plan_ready = events.get("plan_ready")
    if trace["status"] == "succeeded":
        _require(_finite(plan_ready) and _finite(done) and plan_ready >= done,
                 "Successful scored plan lacks its common plan_ready endpoint after computation")
    computation = done-started if _finite(started) and _finite(done) and done >= started else None
    if computation is None and _finite(trace.get("compute_budget", {}).get("elapsed_wall_sec")):
        computation = trace["compute_budget"]["elapsed_wall_sec"]
    rpc = [item["wall_sec"] for item in trace.get("plans", []) if _finite(item.get("wall_sec"))]
    plans = deepcopy(trace.get("plans", []))
    _require(all(item.get("requested_planner_id") == condition["planner_id"]
        and item.get("selected_planner_id") is None for item in plans if item.get("source") != "trajectory_cache"),
        "RPC requested planner differs, or an unsupported selected planner identity was claimed")
    cache_active = type(cache) is dict and cache.get("enabled") is True
    fallback = cache_active and cache.get("fallback") is True
    cache_hit = cache_active and (cache.get("lookup") or {}).get("hit") is True
    return dict(row_id=row["row_id"], condition=row["condition"], repeat=row["repeat"],
        measured=True, status=trace["status"], request_id=trace.get("request_id"),
        initial_max_error_rad=start_error, actual_initial_state_rad=deepcopy(actual),
        error=deepcopy(trace.get("error")), failure_kind=trace.get("failure_kind"),
        computation_wall_sec=computation, planning_rpc_wall_sec=sum(rpc) if rpc else 0. if cache_hit and not fallback else None,
        received_to_plan_ready_wall_sec=plan_ready if _finite(plan_ready) else None,
        cache_lookup_hit=cache_hit, cached_path_reused=cache_hit and not fallback and trace["status"] == "succeeded",
        cache_validation_fallback=fallback,
        cache_lookup_wall_sec=(cache.get("lookup") or {}).get("lookup_wall_sec") if cache_active else None,
        cache_source_validation_wall_sec=cache.get("source_validation_wall_sec") if cache_active else None,
        cache_full_revalidation_wall_sec=cache.get("full_revalidation_wall_sec") if cache_active else None,
        cache_fallback_planning_wall_sec=cache.get("fallback_planning_wall_sec") if cache_active else None,
        received_to_terminal_wall_sec=trace.get("metrics", {}).get("received_to_terminal_wall_sec"),
        trajectory_cache=cache, requested_planner_id=condition["planner_id"], selected_planner_id=None,
        planning_configuration=deepcopy(configuration), plans=plans,
        planned_clearance=deepcopy(trace.get("planned_clearance")),
        path_quality=deepcopy(trace.get("planned_clearance")),
        dispatched_goals=0, review=deepcopy(trace.get("review")))


def aggregate_results(experiment, rows):
    """Aggregate only declared scored rows; all unsuccessful outcomes remain."""
    scored = [row for row in rows if row.get("kind") == "performance" and row.get("scored") is True]
    groups = {condition["id"]: [row for row in scored if row["condition"] == condition["id"]]
              for condition in CONDITIONS}
    valid = {key: [row["summary"] for row in value if row.get("status") == "completed"
                  and row.get("summary", {}).get("measured") is True
                  and _finite(row["summary"].get("computation_wall_sec"))]
             for key, value in groups.items()}
    publish_p95 = all(sum(_finite(item.get(PRIMARY_LATENCY_METRIC)) for item in items) >= 20
                      for items in valid.values())
    result = []
    for condition in CONDITIONS:
        name, values = condition["id"], valid[condition["id"]]
        item = dict(condition=name, requested_planner_id=condition["planner_id"],
            planned=len(groups[name]), measured=len(values),
            succeeded=sum(row["status"] == "succeeded" for row in values),
            failed=sum(row["status"] == "failed" for row in values),
            cache_lookup_hits=sum(row.get("cache_lookup_hit") is True for row in values),
            cached_paths_reused=sum(row.get("cached_path_reused") is True for row in values),
            cache_validation_fallbacks=sum(row.get("cache_validation_fallback") is True for row in values),
            aborted=sum(row.get("status") == "aborted" for row in groups[name]),
            not_run=sum(row.get("status") == "not_run" for row in groups[name]),
            samples=deepcopy(values), statistics={})
        for field in (PRIMARY_LATENCY_METRIC, "computation_wall_sec", "planning_rpc_wall_sec", "received_to_terminal_wall_sec",
                      "cache_lookup_wall_sec", "cache_source_validation_wall_sec", "cache_full_revalidation_wall_sec",
                      "cache_fallback_planning_wall_sec"):
            numbers = [row[field] for row in values if _finite(row.get(field))]
            stats = dict(count=len(numbers), median=statistics.median(numbers) if numbers else None,
                         maximum=max(numbers) if numbers else None)
            if publish_p95 and len(numbers) >= 20:
                stats["p95"] = sorted(numbers)[math.ceil(.95*len(numbers))-1]
                stats["p95_method"] = "nearest-rank"
            item["statistics"][field] = stats
        result.append(item)
    return dict(schema_version=1, experiment_id=experiment["id"], conditions=result,
        primary_latency_metric=PRIMARY_LATENCY_METRIC,
        primary_latency_scope=experiment["comparison_policy"]["primary_latency_scope"],
        compute_scope=COMPUTE_SCOPE, budget_includes_review=True,
        p95_published=publish_p95, p95_rule="At least 20 valid plan_ready endpoint samples in every condition",
        aggregation_scope="Terminal measured planning attempts including planning failures; functional tests and warm-fill preparation excluded",
        comparison_scope="RRT-off vs RRTConnect-off is separate from RRTConnect cold vs warm reuse; no combined global improvement claim")
