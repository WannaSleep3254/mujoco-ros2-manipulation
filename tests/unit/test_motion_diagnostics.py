"""Clearance and failure evidence must keep policy and clock distinctions."""
from copy import deepcopy
import unittest

from mujoco_ros2_manipulation.workcells.motion_diagnostics import measure_endpoints, path_clearance_diagnostic, timing_breakdown
from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace


class Evaluator:
    def at(self, state):
        return dict(distance_m=state["joint"], bodies=["left", "right"])


class MotionDiagnosticsTests(unittest.TestCase):
    def test_endpoints_distinguish_start_goal_and_boundary_without_changing_inputs(self):
        start, goal = {"joint": .03}, {"joint": .029}
        before = deepcopy((start, goal))
        result = measure_endpoints(start, goal, Evaluator(), .03)
        self.assertEqual(result["endpoint_violations"], ["goal"])
        self.assertFalse(result["endpoints"]["start"]["below_policy"])
        self.assertEqual((start, goal), before)
        self.assertEqual(measure_endpoints(goal, start, Evaluator(), .03)["endpoint_violations"], ["start"])

    def test_early_path_refusal_is_not_a_global_minimum_or_an_endpoint_failure(self):
        endpoints = measure_endpoints({"joint": .1}, {"joint": .2}, Evaluator(), .03)
        evaluation = dict(complete=False, minimum=dict(time_sec=1, distance_m=.029))
        result = path_clearance_diagnostic(endpoints, evaluation, 2)
        self.assertEqual(result["violation_location"], "intermediate")
        self.assertEqual(result["endpoint_violations"], [])
        evaluation["minimum"]["time_sec"] = 2
        self.assertEqual(path_clearance_diagnostic(endpoints, evaluation, 2)["violation_location"], "goal")
        evaluation["minimum"]["time_sec"] = 0
        self.assertEqual(path_clearance_diagnostic(endpoints, evaluation, 2)["violation_location"], "start")
        evaluation["complete"] = True
        self.assertIsNone(path_clearance_diagnostic(endpoints, evaluation, 2)["violation_location"])

    def test_cancel_checkpoint_prevents_further_endpoint_work(self):
        def cancelled():
            raise RuntimeError("cancelled")
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            measure_endpoints({}, {}, Evaluator(), .03, checkpoint=cancelled)
        for bad in (0, -1, True, float("nan")):
            with self.subTest(threshold=bad), self.assertRaises(ValueError):
                measure_endpoints({}, {}, Evaluator(), bad)

    def test_failed_stage_and_missing_motion_remain_distinct_from_terminal(self):
        now = [1000000000]
        trace = RequestTrace("a"*32, clock=lambda: now[0])
        trace.event("state_snapshot")
        trace.event("computation_started", simulation_sec=40)
        with self.assertRaises(ValueError):
            with trace.span("safety_checks"):
                now[0] += 100000000
                raise ValueError("clearance")
        trace.finish("failed", dict(type="ValueError", message="clearance"))
        self.assertEqual(trace.data["failure_stage"], "safety_checks")
        self.assertIsNone(trace.data["metrics"]["received_to_first_motion_wall_sec"])
        summary = timing_breakdown(trace.data)
        self.assertEqual(summary["wall_spans"][0]["duration_wall_sec"], .1)
        self.assertEqual(summary["events"][2]["simulation_sec"], 40)
        self.assertEqual(trace.summary("trace.json", "session")["stage"], "terminal")
        self.assertEqual(trace.summary("trace.json", "session")["failure_stage"], "safety_checks")
        summary["events"].clear()
        self.assertEqual(len(trace.data["events"]), 4)


if __name__ == "__main__":
    unittest.main()
