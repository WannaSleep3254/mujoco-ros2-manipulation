"""Read-only selected-request timeline and independently aged receiver data."""
from copy import deepcopy
import json
import unittest

from mujoco_ros2_manipulation.operator_console.analysis import summarize_analysis
from mujoco_ros2_manipulation.workcells.operation_contract import describe_operation


def record(count=3):
    return dict(request_id="a"*32, session_id="b"*32, plan_request_id="c"*32,
        execution_request_id="a"*32, mode="approved_plan", status="running",
        clock=dict(origin_ns=123456789, wall="time.monotonic_ns in this recorder process"),
        events=[dict(name="future_stage", wall_sec=index/10., simulation_sec=40.+index,
            joint_state_stamp_simulation_sec=39.+index) for index in range(count)])


class ConsoleAnalysisTests(unittest.TestCase):
    def test_selected_ids_unknown_phase_and_bounded_full_timeline_preserve_source_counts(self):
        source = record(140)
        value = summarize_analysis(source)
        self.assertEqual(value["request_id"], "a"*32)
        self.assertEqual(value["plan_request_id"], "c"*32)
        self.assertEqual(value["execution_request_id"], "a"*32)
        self.assertEqual(value["phase"], "unknown")
        self.assertEqual(value["timeline"]["source_count"], 140)
        self.assertEqual(value["timeline"]["retained_count"], 128)
        self.assertEqual(value["timeline"]["omitted_prefix_count"], 12)
        self.assertEqual(value["timeline"]["items"][0]["wall_sec"], 1.2)
        self.assertFalse(value["command_authority"])
        self.assertFalse(value["timeline"]["interpolation"])

    def test_compact_poll_counts_are_not_reinvented_as_one_complete_event(self):
        source = record(140)
        operation = describe_operation(source)
        operation["events"]["items"] = operation["events"]["items"][-1:]
        operation["events"].update(retained_count=1, omitted_prefix_count=139)
        source.pop("events")
        source["operation"] = operation
        value = summarize_analysis(source)
        self.assertEqual(value["timeline"]["source_count"], 140)
        self.assertEqual(value["timeline"]["retained_count"], 1)
        self.assertEqual(value["timeline"]["omitted_prefix_count"], 139)
        source["operation"]["events"]["source_count"] = 0
        with self.assertRaises(ValueError):
            summarize_analysis(source)

    def test_source_domains_remain_explicit_and_current_observations_are_not_historical_samples(self):
        value = summarize_analysis(record(), sources=dict(
            joints=dict(stamp_sec=41., age_sec=.1, stale_after_sec=.5, clock_domain="ros_simulation"),
            clock=dict(stamp_sec=42., age_sec=.8, stale_after_sec=.5, clock_domain="ros_simulation"),
            image=dict(stamp_sec=200., age_sec=.5, clock_domain="source_elapsed_monotonic"),
            scene=dict(sequence=7, preview_time_sec=1.2, mode="plan_preview")))
        self.assertEqual(value["sources"]["joints"]["freshness"], "fresh")
        self.assertEqual(value["sources"]["clock"]["freshness"], "stale")
        self.assertEqual(value["sources"]["image"]["freshness"], "unknown")
        self.assertEqual(value["sources"]["scene"]["freshness"], "unknown")
        self.assertEqual(value["sources"]["image"]["clock_domain"], "source_elapsed_monotonic")
        self.assertEqual(value["sources"]["image"]["age_clock_domain"], "console_receiver_elapsed_monotonic")
        self.assertEqual(value["sources_observed_at"], "current_console_snapshot")
        self.assertFalse(value["timeline"]["cross_domain_durations_valid"])
        self.assertIn("not synchronized", value["source_association"])
        self.assertEqual(value["sources"]["scene"]["preview_clock_domain"], "saved_plan_elapsed")

    def test_missing_sources_and_history_are_unknown_instead_of_fresh_or_complete(self):
        value = summarize_analysis(dict(request_id="a"*32))
        self.assertFalse(value["timeline"]["history_provided"])
        self.assertIsNone(value["timeline"]["source_count"])
        for source in value["sources"].values():
            self.assertEqual(source["freshness"], "unknown")
            self.assertFalse(source["available"])

    def test_recent_image_receipt_does_not_override_invalid_or_ambiguous_source_status(self):
        for status in ("invalid", "ambiguous", "disconnected", "closed", "waiting_frame", "error"):
            with self.subTest(status=status):
                source = dict(kind="rgb8-image-source-v1", status=status, stamp_sec=41., age_sec=.1,
                    stale_after_sec=.5, clock_domain="unknown", generation="d"*32)
                image = summarize_analysis(record(), sources=dict(image=source))["sources"]["image"]
                self.assertEqual(image["age_freshness"], "fresh")
                self.assertEqual(image["freshness"], "unknown")
                self.assertEqual(image["status"], status)
                self.assertEqual(image["stamp_semantics"], "latest_received_acquisition_not_encoded_frame")

    def test_whitelist_removes_paths_hosts_arbitrary_details_and_unknown_source_names(self):
        source = record(1)
        source.update(trace_path="/home/private/result.json", host="secret-host", error=dict(message="secret-token"))
        source["events"][0].update(name="/home/private/file", token="secret-token", hostname="secret-host")
        value = summarize_analysis(source, sources={
            "joints":dict(stamp_sec=1., clock_domain="ros_simulation", host="secret-host", path="/private/raw"),
            "/private/camera":dict(raw="secret-token")})
        encoded = json.dumps(value, allow_nan=False)
        for private in ("/home/private", "/private", "secret-token", "secret-host"):
            self.assertNotIn(private, encoded)
        self.assertTrue(value["timeline"]["items"][0]["label_redacted"])
        self.assertEqual(value["source_counts"], dict(provided=2, supported=1, unknown=1))

    def test_malformed_times_domains_counts_and_monotonic_origins_fail_closed(self):
        for observation in (dict(age_sec=float("nan")), dict(stamp_sec=True),
                            dict(clock_domain="utc"), dict(stale_after_sec=0), dict(sequence=True)):
            with self.subTest(observation=observation), self.assertRaises(ValueError):
                summarize_analysis(record(), sources=dict(joints=observation))
        source = record()
        source["events"][2]["wall_sec"] = 0.
        with self.assertRaisesRegex(ValueError, "backwards"):
            summarize_analysis(source)
        for invalid in (True, -1, float("inf"), 2**80):
            source = record()
            source["clock"]["origin_ns"] = invalid
            with self.subTest(origin=invalid), self.assertRaises(ValueError):
                summarize_analysis(source)

    def test_projection_is_detached_and_does_not_mutate_original_record(self):
        source = record()
        old = deepcopy(source)
        value = summarize_analysis(source, sources=dict(image=dict(age_sec=.8)))
        value["timeline"]["items"][0]["wall_sec"] = 99.
        self.assertEqual(source, old)
        self.assertEqual(json.loads(json.dumps(value, allow_nan=False)), value)
