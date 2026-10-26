"""Execution journaling never serializes the large planning snapshot again."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import mock_open, patch

from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace


class RequestJournalTests(unittest.TestCase):
    def trace(self):
        trace = RequestTrace("a" * 32, clock=lambda: 1_000_000_000)
        trace.data.update(status="running", scheduled_start_simulation_sec=42.0,
                          compute_budget=dict(limit_wall_sec=30.0, elapsed_wall_sec=0.9,
                                              exceeded=False, enforcement="Cooperative checks"))
        return trace

    def test_cursor_appends_each_new_index_once_and_noop_never_opens_file(self):
        trace = self.trace()
        trace.event("dispatch", simulation_sec=41.0)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "events.jsonl"
            cursor = trace.append_events(path)
            self.assertEqual(cursor, 2)
            trace.event("goal_sent", arm="fr5", simulation_sec=41.1)
            cursor = trace.append_events(path, cursor)
            self.assertEqual(cursor, 3)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([row["event_index"] for row in rows], [0, 1, 2])
            self.assertEqual([row["event"]["name"] for row in rows], ["received", "dispatch", "goal_sent"])
            self.assertEqual(rows[-1]["event"]["arm"], "fr5")
            before = (path.read_bytes(), path.stat().st_mtime_ns)
            with patch("builtins.open", side_effect=AssertionError("No-op must not open")):
                self.assertEqual(trace.append_events(path, cursor), cursor)
            self.assertEqual((path.read_bytes(), path.stat().st_mtime_ns), before)

    def test_large_or_unserializable_planning_geometry_and_extra_event_detail_are_ignored(self):
        trace = self.trace()
        private_geometry = object()
        trace.data.update(planned_trajectories=private_geometry, spans=private_geometry,
                          geometry=private_geometry, plan_selection=private_geometry)
        event = trace.event("goal_result", arm="ur5e", status=4, error_code=0, geometry=private_geometry)
        trace.data["compute_budget"]["geometry"] = private_geometry
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "events.jsonl"
            trace.append_events(path)
            row = json.loads(path.read_text().splitlines()[-1])
        self.assertEqual(set(row), {"schema_version", "request_id", "event_index", "event", "status",
                                    "scheduled_start_simulation_sec", "compute_budget"})
        self.assertEqual(row["event"], dict(name="goal_result", wall_sec=0.0, simulation_sec=None,
                                           arm="ur5e", status=4, error_code=0))
        self.assertNotIn("geometry", row["compute_budget"])
        self.assertIs(event["geometry"], private_geometry)
        self.assertIs(trace.data["planned_trajectories"], private_geometry)

    def test_appended_snapshot_is_unchanged_by_later_mutation_and_final_json_keeps_full_events(self):
        trace = self.trace()
        event = trace.event("goal_acceptance", arm="fr5", accepted=True, note="Full diagnostic detail")
        original_events = deepcopy(trace.data["events"])
        with tempfile.TemporaryDirectory() as folder:
            journal = Path(folder) / "events.jsonl"
            cursor = trace.append_events(journal)
            saved = journal.read_bytes()
            self.assertEqual(trace.data["events"], original_events)
            event.update(accepted=False, name="changed")
            trace.data["compute_budget"]["elapsed_wall_sec"] = 99.0
            trace.data["status"] = "failed"
            self.assertEqual(journal.read_bytes(), saved)
            row = json.loads(saved.splitlines()[-1])
            self.assertTrue(row["event"]["accepted"])
            self.assertEqual(row["status"], "running")
            self.assertEqual(row["compute_budget"]["elapsed_wall_sec"], 0.9)
            trace.finish("failed", dict(type="RuntimeError", message="Failure"))
            trace.append_events(journal, cursor)
            final = Path(folder) / "trace.json"
            trace.save(final)
            final_data = json.loads(final.read_text())
        self.assertEqual(final_data["events"], trace.data["events"])
        self.assertEqual(final_data["events"][1]["note"], "Full diagnostic detail")
        self.assertEqual(final_data["events"][-1]["name"], "terminal")

    def test_multiple_new_events_use_one_open_and_one_compact_write(self):
        trace = self.trace()
        trace.event("goal_sent", arm="fr5")
        trace.event("goal_sent", arm="ur5e")
        opened = mock_open()
        with patch("builtins.open", opened):
            self.assertEqual(trace.append_events("events.jsonl"), 3)
        opened.assert_called_once_with("events.jsonl", "a", encoding="utf-8")
        opened().write.assert_called_once()
        text = opened().write.call_args.args[0]
        self.assertEqual(len(text.splitlines()), 3)
        self.assertNotIn('": ', text)
        self.assertTrue(text.endswith("\n"))

    def test_invalid_cursor_is_rejected_before_file_access(self):
        trace = self.trace()
        for cursor in (True, False, -1, 2, 0.0, "0", None):
            with self.subTest(cursor=cursor), patch("builtins.open") as opened:
                with self.assertRaisesRegex(ValueError, "Event cursor"):
                    trace.append_events("events.jsonl", cursor)
                opened.assert_not_called()

    def test_open_and_write_errors_are_propagated_without_advancing_a_cursor(self):
        trace = self.trace()
        error = OSError("Storage unavailable")
        with patch("builtins.open", side_effect=error), self.assertRaises(OSError) as caught:
            trace.append_events("events.jsonl")
        self.assertIs(caught.exception, error)
        opened = mock_open()
        opened().write.side_effect = error
        with patch("builtins.open", opened), self.assertRaises(OSError) as caught:
            trace.append_events("events.jsonl")
        self.assertIs(caught.exception, error)
        self.assertEqual(len(trace.data["events"]), 1)

    def test_nonfinite_compact_values_fail_before_any_append(self):
        trace = self.trace()
        trace.data["events"][0]["wall_sec"] = float("nan")
        with patch("builtins.open") as opened, self.assertRaises(ValueError):
            trace.append_events("events.jsonl")
        opened.assert_not_called()
