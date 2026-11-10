"""Pure production benchmark timing checks, with no ROS initialization."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from tests.unit.test_fixed_runner import load_runner_functions
from tests.unit.test_fixed_execution import Execution


class FixedBenchmarkTimingTests(unittest.TestCase):
    def fixture(self, *, stamps=None, publisher=None):
        self.start = 9_007_199_254_740_991
        self.session = "b" * 32
        context = dict(schema_version=1, context_id="c" * 64, clock_source="CLOCK_MONOTONIC",
            monotonic_implementation="clock_gettime(CLOCK_MONOTONIC)", boot_id_sha256="d" * 64,
            time_namespace_identity="time:[123]", process_pid=123, process_start_ticks="456",
            proof_status="self_observed", unavailable_reason=None)
        functions = load_runner_functions(sample_ns=Mock(side_effect=stamps or [str(self.start), str(self.start + 10_000_000)]))
        self.payload = dict(request_id="a" * 32, strategy="coordinated", target_rad={"j": 0.})
        self.message = SimpleNamespace(data=json.dumps(self.payload))
        self.publisher = publisher or Mock()
        node = SimpleNamespace(command_publisher=self.publisher, session_id=self.session, timing_clock_context=context)
        observation = functions["_observe_publications"](node)
        result = dict(session_id=self.session, scored=dict(request_id=self.payload["request_id"]))
        origin = self.start + 2_000_000
        trace = dict(session_id=self.session, request_id=self.payload["request_id"], command=deepcopy(self.payload),
            clock=dict(origin_ns=origin, origin_monotonic_ns=str(origin), context_id=context["context_id"], context=deepcopy(context)),
            events=[dict(name=name, monotonic_ns=str(self.start + offset)) for name, offset in
                    (("received", 2_000_100), ("first_motion", 3_000_000), ("actions_done", 11_000_000),
                     ("settled", 12_000_000), ("terminal", 13_000_000))])
        return functions, node, observation, result, trace

    def test_existing_execution_startup_subscription_admission_uses_original_publisher(self):
        for subscriptions in (0, 1):
            with self.subTest(subscriptions=subscriptions):
                _, observed_node, _, _, _ = self.fixture()
                self.publisher.get_subscription_count.return_value = subscriptions
                node = object.__new__(Execution)
                node.command_publisher = observed_node.command_publisher
                node.prepare = Mock()
                def until(predicate, timeout, label):
                    self.assertEqual((timeout, label), (5, "fixed case command subscription"))
                    if not predicate():
                        raise TimeoutError("No subscription")
                node.until = until
                node.joint_names = ["j"]
                node.cell = SimpleNamespace(home=[0.])
                node.drive_to_state = Mock(side_effect=RuntimeError("Stop before movement"))
                node.result_record = {}
                node.save_result = Mock()
                expected = "No subscription" if subscriptions == 0 else "Stop before movement"
                with self.assertRaisesRegex((RuntimeError, TimeoutError), expected):
                    node.run()
                self.publisher.get_subscription_count.assert_called_once_with()
                self.publisher.publish.assert_not_called()
                node.prepare.assert_called_once_with()
                self.assertEqual(node.result_record["status"], "aborted")

    def test_exact_ns_join_accepts_receive_before_publish_return_and_keeps_legacy_trace(self):
        functions, node, observation, result, trace = self.fixture()
        original = deepcopy(trace)
        returned = node.command_publisher.publish(self.message)
        self.assertIs(returned, self.publisher.publish.return_value)
        self.publisher.publish.assert_called_once_with(self.message)
        timing = functions["_scored_timing"](observation, result, trace)
        self.assertEqual(trace, original)
        self.assertEqual(timing["clock_join"]["status"], "same_process")
        self.assertEqual(timing["stamps"]["publish_begin_ns"], str(self.start))
        self.assertEqual(timing["metrics"]["publish_call"]["value_sec"], .01)
        self.assertEqual(timing["metrics"]["publish_to_received"]["value_sec"], .002)
        self.assertEqual(timing["metrics"]["publish_to_first_motion"]["value_sec"], .003)
        self.assertEqual(timing["metrics"]["publish_to_terminal"]["value_sec"], .013)
        self.assertEqual(observation["publications"][0]["payload_sha256"], hashlib.sha256(self.message.data.encode()).hexdigest())
        self.assertIsNone(observation["http_browser_timing"]["value_sec"])

    def test_legacy_cross_context_session_or_payload_cannot_be_joined(self):
        changes = {
            "legacy": lambda trace: trace.update(clock=dict(origin_ns=self.start)),
            "context": lambda trace: trace["clock"]["context"].update(boot_id_sha256="e" * 64),
            "context_id": lambda trace: trace["clock"].update(context_id="e" * 64),
            "session": lambda trace: trace.update(session_id="e" * 32),
            "request": lambda trace: trace.update(request_id="e" * 32),
            "payload": lambda trace: trace["command"]["target_rad"].update(j=.5),
            "origin_rounding": lambda trace: trace["clock"].update(origin_ns=self.start),
            "predates_publish": lambda trace: trace["clock"].update(origin_ns=self.start - 1, origin_monotonic_ns=str(self.start - 1)),
        }
        for name, change in changes.items():
            with self.subTest(name=name):
                functions, node, observation, result, trace = self.fixture()
                node.command_publisher.publish(self.message)
                change(trace)
                timing = functions["_scored_timing"](observation, result, trace)
                self.assertEqual(timing["clock_join"]["status"], "unavailable")
                for metric in functions["TIMING_METRICS"][1:]:
                    self.assertIsNone(timing["metrics"][metric]["value_sec"])
                    self.assertTrue(timing["metrics"][metric]["unavailable_reason"])
                self.assertEqual(timing["metrics"]["publish_call"]["value_sec"], .01)

    def test_duplicate_publication_keeps_first_marks_and_never_suppresses_actual_publish(self):
        functions, node, observation, result, trace = self.fixture()
        node.command_publisher.publish(self.message)
        first = deepcopy(observation["publications"][0])
        node.command_publisher.publish(self.message)
        self.assertEqual(self.publisher.publish.call_count, 2)
        self.assertEqual(len(observation["publications"]), 1)
        current = observation["publications"][0]
        self.assertEqual(current["publish_begin_ns"], first["publish_begin_ns"])
        self.assertEqual(current["publish_returned_ns"], first["publish_returned_ns"])
        self.assertEqual(current["duplicate_calls"], 1)
        self.assertEqual(functions["_scored_timing"](observation, result, trace)["clock_join"]["unavailable_reason"],
                         "duplicate_publication_identity")

    def test_missing_clock_is_diagnostic_only_and_real_publish_exception_still_propagates(self):
        functions, node, observation, result, trace = self.fixture(stamps=[None, None])
        node.command_publisher.publish(self.message)
        self.publisher.publish.assert_called_once()
        timing = functions["_scored_timing"](observation, result, trace)
        self.assertIsNone(timing["metrics"]["publish_call"]["value_sec"])
        self.assertEqual(timing["clock_join"]["unavailable_reason"], "original_receive_stamp_unavailable")
        for error in (RuntimeError("Transport failed"), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__):
                functions, node, observation, result, trace = self.fixture(publisher=Mock())
                self.publisher.publish.side_effect = error
                with self.assertRaises(type(error)):
                    node.command_publisher.publish(self.message)
                self.publisher.publish.assert_called_once()
                self.assertEqual(observation["publications"][0]["exception_type"], type(error).__name__)
                self.assertEqual(functions["_scored_timing"](observation, result, trace)["clock_join"]["unavailable_reason"],
                                 "publication_did_not_return")

    def test_invalid_publish_return_stamp_never_produces_source_latency(self):
        for returned in (None, str(9_007_199_254_740_990)):
            with self.subTest(returned=returned):
                functions, node, observation, result, trace = self.fixture(
                    stamps=[str(9_007_199_254_740_991), returned])
                node.command_publisher.publish(self.message)
                timing = functions["_scored_timing"](observation, result, trace)
                self.assertEqual(timing["clock_join"]["unavailable_reason"], "local_publish_return_stamp_invalid")
                self.assertTrue(all(timing["metrics"][name]["value_sec"] is None
                                    for name in functions["TIMING_METRICS"][1:]))

    def test_observation_limit_does_not_drop_or_repeat_actual_commands(self):
        functions, node, observation, _, _ = self.fixture(stamps=["100", "101"] * 32)
        for number in range(33):
            node.command_publisher.publish(SimpleNamespace(data=json.dumps(dict(request_id=f"{number:032x}"))))
        self.assertEqual(self.publisher.publish.call_count, 33)
        self.assertEqual(len(observation["publications"]), 32)
        self.assertEqual(observation["dropped_publications"], 1)
        node.command_publisher.publish(SimpleNamespace(data="invalid JSON"))
        self.assertEqual(self.publisher.publish.call_count, 34)

    def test_missing_or_backward_event_has_explicit_unavailable_interval(self):
        functions, node, observation, result, trace = self.fixture()
        node.command_publisher.publish(self.message)
        trace["events"] = [dict(name="first_motion", monotonic_ns=str(self.start + 1))]
        timing = functions["_scored_timing"](observation, result, trace)
        self.assertIsNone(timing["metrics"]["publish_to_first_motion"]["value_sec"])
        self.assertEqual(timing["metrics"]["publish_to_first_motion"]["unavailable_reason"], "source_clock_order_invalid")
        self.assertEqual(timing["metrics"]["publish_to_settled"]["unavailable_reason"], "original_settled_stamp_unavailable")
        self.assertEqual(timing["metrics"]["publish_to_received"]["value_sec"], .002)

    def test_retention_is_read_only_and_malformed_optional_timing_does_not_raise(self):
        functions, node, observation, result, trace = self.fixture()
        node.command_publisher.publish(self.message)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "original.json"
            original = json.dumps(trace, indent=1).encode()
            path.write_bytes(original)
            row = dict(timing=observation, trace_path=str(path))
            functions["_retain_timing"](row, result)
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(row["timing"]["scored"]["clock_join"]["status"], "same_process")
            path.write_text('{"clock": "malformed"}')
            functions["_retain_timing"](row, result)
            self.assertEqual(row["timing"]["unavailable_reason"], "timing_evidence_unavailable")
            self.assertIsNone(row["timing"]["scored"])

    def test_small_n_summary_is_case_variant_scoped_and_excludes_aborted_observations(self):
        functions, node, observation, result, trace = self.fixture()
        node.command_publisher.publish(self.message)
        observation["scored"] = functions["_scored_timing"](observation, result, trace)
        rows = [dict(selected=True, case_id="local_small", variant="single", status=status, timing=deepcopy(observation))
                for status in ("completed", "aborted", "not_run")]
        rows[2]["timing"] = None
        summary = functions["_timing_summary"](rows)
        group = summary["groups"][0]
        self.assertEqual((group["planned"], group["completed"], group["aborted"], group["not_run"]), (3, 1, 1, 1))
        self.assertEqual(group["metrics"]["publish_to_terminal"], dict(count=1, median_sec=.013, max_sec=.013))
        self.assertNotIn("p95", json.dumps(summary))


if __name__ == "__main__":
    unittest.main()
