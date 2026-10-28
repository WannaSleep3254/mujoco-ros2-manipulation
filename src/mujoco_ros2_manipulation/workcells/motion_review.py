"""Single-use simulation plan reviews, independent of ROS and engine imports.

Only one review is retained. The command owner claims it under a lock before
queueing execution; a failed/cancelled claim cannot be approved a second time.
The executor must still recheck identity, state and all trajectory safety gates.
"""
from copy import deepcopy
import hashlib
import json
import math
import threading
import time

from .motion_request import request_id, validate_approval


REVIEW_TTL_SEC = 60.0
INITIAL_TOLERANCE_RAD = 0.002


class ReviewRejected(ValueError):
    """A review is absent, expired, changed or already consumed."""


def _state(value):
    if (not isinstance(value, dict) or not value
            or any(not isinstance(name, str) or not name for name in value)
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value.values())):
        raise ReviewRejected("Review requires a complete finite named state")
    return value


def _copy(value):
    return json.loads(json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":")))


class PlanReviewStore:
    def __init__(self, session_id, *, clock=time.monotonic, ttl_sec=REVIEW_TTL_SEC):
        self.session_id = request_id(session_id)
        if type(ttl_sec) not in (int, float) or not math.isfinite(ttl_sec) or not 1 <= ttl_sec <= 60:
            raise ValueError("Review lifetime must be 1..60 wall seconds")
        self.clock, self.ttl_sec = clock, float(ttl_sec)
        self.lock = threading.Lock()
        self.record = None

    def _refresh(self, now):
        record = self.record
        if record and record["state"] == "awaiting_approval" and now >= record["expires_at"]:
            record.update(state="expired", reason="Review expired; create a new plan")

    def _summary(self, now):
        self._refresh(now)
        if self.record is None:
            return None
        record = self.record
        return dict(schema_version=1, plan_request_id=record["plan_request_id"],
                    session_id=self.session_id, plan_sha256=record["plan_sha256"],
                    state=record["state"], reason=record.get("reason"),
                    expires_after_sec=self.ttl_sec,
                    remaining_sec=max(0.0, record["expires_at"] - now),
                    execution_request_id=record.get("execution_request_id"))

    def summary(self):
        with self.lock:
            return self._summary(self.clock())

    def payload_for(self, plan_request_id, plan_sha256):
        """Return the exact detached digest input for a verified preview trace."""
        with self.lock:
            if (self.record is None or self.record["plan_request_id"] != plan_request_id
                    or self.record["plan_sha256"] != plan_sha256):
                raise ReviewRejected("Preview does not identify the stored plan")
            return deepcopy(self.record["payload"])

    def issue(self, plan_request_id, *, identity, initial_state_rad, simulation_sec,
              command, trajectories, no_motion_required=False, goal_context=None):
        request_id(plan_request_id)
        _state(initial_state_rad)
        if type(simulation_sec) not in (int, float) or not math.isfinite(simulation_sec):
            raise ValueError("Expected finite simulation time")
        if not isinstance(identity, dict) or not identity:
            raise ValueError("Review requires model/environment/policy identity")
        payload = _copy(dict(schema_version=1, plan_request_id=plan_request_id,
            session_id=self.session_id, identity=identity, initial_state_rad=initial_state_rad,
            simulation_sec=simulation_sec, command=command, trajectories=trajectories,
            no_motion_required=no_motion_required))
        if goal_context is not None:
            from .pose_tools import validate_context_identity
            validate_context_identity(goal_context, plan_request_id=plan_request_id)
            payload["goal_context"] = _copy(goal_context)
        digest = hashlib.sha256(json.dumps(payload, sort_keys=True, allow_nan=False,
            separators=(",", ":")).encode()).hexdigest()
        with self.lock:
            if self.record and self.record["plan_request_id"] == plan_request_id:
                raise ReviewRejected("A plan request identity may not be issued twice")
            now = self.clock()
            self.record = dict(payload=payload, plan_request_id=plan_request_id,
                plan_sha256=digest, expires_at=now+self.ttl_sec, state="awaiting_approval")
            return self._summary(now)

    def invalidate(self, reason):
        with self.lock:
            now = self.clock()
            self._refresh(now)
            if self.record and self.record["state"] == "awaiting_approval":
                self.record.update(state="invalidated", reason=str(reason))
            return self._summary(now)

    def claim(self, approval, *, identity, state_rad, simulation_sec):
        validate_approval(approval)
        _state(state_rad)
        with self.lock:
            now = self.clock()
            self._refresh(now)
            record = self.record
            if record is None or record["state"] != "awaiting_approval":
                raise ReviewRejected("Review is absent, expired, invalidated or already consumed; replan")
            if (approval["session_id"] != self.session_id
                    or approval["plan_request_id"] != record["plan_request_id"]
                    or approval["plan_sha256"] != record["plan_sha256"]):
                raise ReviewRejected("Approval does not identify this server's exact plan")
            payload = record["payload"]
            reason = self._changed(payload, identity, state_rad, simulation_sec)
            if reason:
                record.update(state="invalidated", reason=reason)
                raise ReviewRejected(reason)
            # Consume before returning data, even when the eventual executor fails.
            record.update(state="consumed", execution_request_id=approval["request_id"],
                          reason="Approved once; execution still requires current safety checks")
            return deepcopy(payload), self._summary(now)

    @staticmethod
    def _changed(payload, identity, state_rad, simulation_sec):
        if payload["identity"] != identity:
            return "Model, environment or policy changed; replan"
        initial = payload["initial_state_rad"]
        if (set(initial) != set(state_rad)
                or max(abs(state_rad[n]-initial[n]) for n in initial) >= INITIAL_TOLERANCE_RAD):
            return "Initial robot state changed; replan"
        if (type(simulation_sec) not in (int, float) or not math.isfinite(simulation_sec)
                or simulation_sec < payload["simulation_sec"]):
            return "Simulation time changed backwards; replan"
        return None

    def recheck_claim(self, payload, *, identity, state_rad, simulation_sec):
        """Check the claimed snapshot again immediately before dispatch.

        User review time is bounded by wall time through the final gate, rather
        than stopping the expiry clock when an approval packet is received.
        """
        _state(state_rad)
        with self.lock:
            record = self.record
            if (record is None or record["state"] != "consumed"
                    or record["payload"] != payload or self.clock() >= record["expires_at"]):
                raise ReviewRejected("Approved plan expired or was replaced; replan")
            reason = self._changed(payload, identity, state_rad, simulation_sec)
            if reason:
                raise ReviewRejected(reason)
