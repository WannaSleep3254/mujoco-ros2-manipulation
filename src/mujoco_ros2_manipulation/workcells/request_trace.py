"""One recorder's monotonic timeline; simulation stamps are separate observations."""
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import json
import time

from mujoco_ros2_manipulation.verification.reports import atomic_json


class RequestTrace:
    def __init__(self, identifier, *, clock=time.monotonic_ns, received_ns=None):
        self.clock = clock
        self.origin = clock() if received_ns is None else received_ns
        self.data = dict(schema_version=1, request_id=identifier, status="received",
            received_utc=datetime.now(timezone.utc).isoformat(),
            clock=dict(wall="time.monotonic_ns in this recorder process", origin_ns=self.origin,
                       simulation="ROS joint-state /clock seconds; never added to wall durations"),
            events=[], spans=[], metrics={}, error=None, failure_kind=None, dispatched_goals=0)
        self.event("received")

    def elapsed(self):
        return (self.clock() - self.origin) / 1e9

    def event(self, name, *, simulation_sec=None, **details):
        record = dict(name=name, wall_sec=self.elapsed(), simulation_sec=simulation_sec, **details)
        if self.data["events"] and record["wall_sec"] < self.data["events"][-1]["wall_sec"]:
            raise ValueError("Recorder monotonic clock moved backwards")
        self.data["events"].append(record)
        return record

    @contextmanager
    def span(self, name):
        record = dict(name=name, start_wall_sec=self.elapsed(), status="running")
        self.data["spans"].append(record)
        try:
            yield record
        except BaseException as error:
            record.update(status="failed", error=type(error).__name__)
            raise
        else:
            record["status"] = "completed"
        finally:
            record["end_wall_sec"] = self.elapsed()
            record["duration_wall_sec"] = record["end_wall_sec"] - record["start_wall_sec"]

    def skip(self, name, reason):
        self.data["spans"].append(dict(name=name, status="not_applicable", duration_wall_sec=None, reason=reason))

    def finish(self, status, error=None):
        if status not in ("succeeded", "failed", "cancelled", "rejected"):
            raise ValueError("Expected a terminal command status")
        failed = next((s["name"] for s in reversed(self.data["spans"]) if s["status"] == "failed"), None)
        self.data["failure_stage"] = (failed or self.data["events"][-1]["name"]) if status != "succeeded" else None
        self.event("terminal")
        self.data.update(status=status, error=error)
        first = lambda name: next((e["wall_sec"] for e in self.data["events"] if e["name"] == name), None)
        self.data["metrics"] = dict(received_to_first_motion_wall_sec=first("first_motion"),
            received_to_actions_done_wall_sec=first("actions_done"),
            received_to_settled_wall_sec=first("settled"),
            received_to_terminal_wall_sec=first("terminal"),
            received_to_state_snapshot_wall_sec=first("state_snapshot"),
            received_to_computation_started_wall_sec=first("computation_started"),
            received_to_computation_done_wall_sec=first("computation_done"))

    def save(self, path):
        atomic_json(path, self.data)


    def append_events(self, path, since=0):
        """Append only new compact event envelopes; leave the full trace intact.

        The cursor is an index into ``data["events"]``. Geometry, plans and spans
        are excluded even if an event accidentally carries a large extra detail.
        Serializing all lines before opening also rejects nonfinite values before
        any append. This raw diagnostic journal does not replace the final JSON.
        """
        events = self.data["events"]
        count = len(events)
        if type(since) is not int or not 0 <= since <= count:
            raise ValueError("Event cursor must be an integer within the recorded events")
        if since == count:
            return count
        event_fields = ("name", "wall_sec", "simulation_sec", "joint_state_stamp_simulation_sec",
                        "arm", "status", "error_code", "accepted")
        budget_fields = ("limit_wall_sec", "elapsed_wall_sec", "exceeded", "enforcement")
        budget = self.data.get("compute_budget")
        if budget is not None:
            budget = {name: budget[name] for name in budget_fields if name in budget}
        lines = []
        for index, event in enumerate(events[since:count], start=since):
            record = dict(schema_version=1, request_id=self.data["request_id"], event_index=index,
                          event={name: event[name] for name in event_fields if name in event},
                          status=self.data["status"],
                          scheduled_start_simulation_sec=self.data.get("scheduled_start_simulation_sec"),
                          compute_budget=budget)
            lines.append(json.dumps(record, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
        with open(path, "a", encoding="utf-8") as stream:
            stream.write("".join(lines))
        return count

    def summary(self, path, session_id):
        return dict(schema_version=1, session_id=session_id, request_id=self.data["request_id"],
            status=self.data["status"], stage=self.data["events"][-1]["name"],
            trace_path=str(path), metrics=deepcopy(self.data["metrics"]), error=self.data["error"],
            failure_stage=self.data.get("failure_stage"), failure_kind=self.data.get("failure_kind"))
