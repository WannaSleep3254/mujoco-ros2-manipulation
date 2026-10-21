"""Protocol boundaries and clock/terminal semantics without ROS dependencies."""
import json
from pathlib import Path
import tempfile
import unittest

from mujoco_ros2_manipulation.workcells.motion_request import command, decode_message, request_id, validate_command, validate_cancel
from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.limits = {"joint": dict(lower=-1, upper=1)}

    def test_valid_complete_command_and_cancellation(self):
        value = command({"joint": 0.2}, "departure_timing")
        self.assertEqual(validate_command(decode_message(json.dumps(value)), self.limits), value)
        self.assertEqual(validate_cancel({"request_id": value["request_id"]}), value["request_id"])

    def test_unknown_partial_nonfinite_boolean_and_out_of_limit_goals_are_rejected(self):
        for target in ({}, {"unknown": 0}, {"joint": True}, {"joint": 2}, {"joint": float("nan")}, {"joint": 10**1000}):
            with self.subTest(target=target), self.assertRaises(ValueError):
                validate_command(command(target), self.limits)
        value = command({"joint": 0})
        value["schema_version"] = True
        with self.assertRaises(ValueError):
            validate_command(value, self.limits)
        value["schema_version"] = 1
        value["extra"] = "not allowed"
        with self.assertRaises(ValueError):
            validate_command(value, self.limits)

    def test_duplicate_keys_overflow_and_excessive_messages_are_rejected(self):
        for text in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":1e309}', '{"a":'+"9"*100+'}', '[]', " "*65537):
            with self.subTest(text=text[:30]), self.assertRaises(ValueError):
                decode_message(text)

    def test_request_ids_cannot_escape_trace_paths(self):
        for value in ("../result", "A"*32, "a"*31, ["a"*32], None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                request_id(value)
        with self.assertRaises(ValueError):
            validate_cancel({"request_id":"a"*32,"other":True})


class TraceTests(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.trace = RequestTrace("a"*32, clock=lambda:self.now)

    def advance(self, seconds):
        self.now += round(seconds*1e9)

    def test_single_recorder_wall_latency_does_not_add_simulation_wait(self):
        with self.trace.span("planning"):
            self.advance(0.2)
        self.trace.event("goal_acceptance", simulation_sec=100)
        self.advance(1)
        self.trace.event("first_motion", simulation_sec=101)
        self.advance(2)
        self.trace.event("actions_done", simulation_sec=103)
        self.advance(0.1)
        self.trace.event("settled", simulation_sec=103.1)
        self.trace.finish("succeeded")
        self.assertAlmostEqual(self.trace.data["metrics"]["received_to_first_motion_wall_sec"],1.2)
        self.assertAlmostEqual(self.trace.data["metrics"]["received_to_actions_done_wall_sec"],3.2)
        self.assertEqual(self.trace.data["events"][2]["simulation_sec"],101)
        self.assertAlmostEqual(self.trace.data["spans"][0]["duration_wall_sec"],0.2)

    def test_failed_and_cancelled_requests_have_null_movement_not_zero(self):
        for status in ("failed","cancelled","rejected"):
            trace=RequestTrace("a"*32,clock=lambda:self.now)
            trace.skip("departure_search","not used by this strategy")
            self.advance(0.3)
            trace.finish(status,dict(type="test",message="test-only failure"))
            self.assertIsNone(trace.data["metrics"]["received_to_first_motion_wall_sec"])
            self.assertIsNone(trace.data["spans"][0]["duration_wall_sec"])
            self.assertAlmostEqual(trace.data["metrics"]["received_to_terminal_wall_sec"],0.3)

    def test_failed_span_and_atomic_trace_preserve_primary_error(self):
        with self.assertRaisesRegex(RuntimeError,"planner unavailable"):
            with self.trace.span("planning"):
                self.advance(0.4)
                raise RuntimeError("planner unavailable")
        self.trace.finish("failed",dict(type="RuntimeError",message="planner unavailable"))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"request.json"
            self.trace.save(path)
            saved=json.loads(path.read_text())
        self.assertEqual(saved["spans"][0]["status"],"failed")
        self.assertEqual(saved["error"]["message"],"planner unavailable")
        self.assertEqual(saved["metrics"]["received_to_first_motion_wall_sec"],None)

    def test_backwards_recorder_clock_is_rejected(self):
        self.now-=1
        with self.assertRaisesRegex(ValueError,"backwards"):
            self.trace.event("invalid")
