"""ROS/engine-independent diagnostics; measurements never relax execution gates."""
from copy import deepcopy
import math


def measure_endpoints(initial, target, evaluator, threshold_m, *, checkpoint=None):
    """Use the injected evaluator at both complete named states, including failures."""
    if type(threshold_m) not in (int, float) or not math.isfinite(threshold_m) or threshold_m <= 0:
        raise ValueError("Expected a positive finite clearance threshold")
    result = dict(threshold_m=threshold_m, endpoints={}, endpoint_violations=[],
                  scope="Sampled cross-arm geometry only; separate from MoveIt validity")
    for name, state in (("start", initial), ("goal", target)):
        if checkpoint:
            checkpoint()
        value = deepcopy(evaluator.at(state))
        if not math.isfinite(value["distance_m"]):
            raise ValueError("Nonfinite endpoint distance")
        value["below_policy"] = value["distance_m"] < threshold_m
        result["endpoints"][name] = value
        if value["below_policy"]:
            result["endpoint_violations"].append(name)
        if checkpoint:
            checkpoint()
    return result


def path_clearance_diagnostic(endpoints, evaluation, duration_sec):
    """Classify the first subthreshold sample, without claiming a full-path minimum."""
    witness = evaluation["minimum"]
    location = None
    if not evaluation["complete"]:
        stamp = witness["time_sec"]
        location = "start" if abs(stamp) <= 1e-9 else "goal" if abs(stamp-duration_sec) <= 1e-9 else "intermediate"
    return dict(endpoint_violations=list(endpoints["endpoint_violations"]),
                sampled_path_complete=evaluation["complete"], violation_location=location,
                witness=deepcopy(witness),
                witness_semantics="Minimum of evaluated samples; sampling stops at first policy violation")


def timing_breakdown(trace):
    """Expose wall spans and simulation events separately; never infer DDS latency."""
    return dict(
        wall_clock=deepcopy(trace["clock"]),
        request_metrics=deepcopy(trace["metrics"]),
        wall_spans=deepcopy(trace["spans"]),
        planning_calls=deepcopy(trace.get("plans", [])),
        simulation_reservation=dict(scheduled_start_sec=trace.get("scheduled_start_simulation_sec"),
                                    state_stamp_sec=trace.get("joint_state_stamp_simulation_sec")),
        events=deepcopy(trace["events"]),
        failure_stage=trace.get("failure_stage"),
        note="Wall spans may overlap; simulation reservation is not added to wall time. RPC minus planning_time is not pure transport latency.")
