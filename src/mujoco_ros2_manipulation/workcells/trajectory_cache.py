"""Bounded, opt-in structural trajectory reuse; never an execution authority.

The owner supplies SHA-256 identities of its checked source, model, environment,
tool, policy and planner configuration. This module has no ROS, engine, filesystem
or review-token access. A hit is only a detached candidate: the owner must check
current identity, cancellation/budget, limits and the entire native/MoveIt path,
then issue a new single-use review. Those failures must never become cache misses.

The initial scope is one coordinated spline with every configured joint in its
declared order. Start anchors are exact by default. An explicit bounded policy
may choose the nearest same-identity/exact-goal anchor and replace only the first
position in a detached candidate. Original anchors, paths and TTL never change;
there is no rounding, retiming, partial-arm path or old approval reuse.
"""
from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import json
import math
import re
import threading
import time

from .interference import Spline


IDENTITY_FIELDS = ("source_sha256", "model_sha256", "environment_sha256",
                   "tool_sha256", "policy_sha256", "planner_sha256")
MAX_JOINTS = 64
MAX_POINTS = 2000
MAX_ENTRY_BYTES = 4 * 1024 * 1024
MAX_TOTAL_BYTES = 64 * 1024 * 1024
MAX_ENTRIES = 128
MISS_REASONS = frozenset(("cold", "identity_mismatch", "start_mismatch", "goal_mismatch",
                          "start_and_goal_mismatch", "expired", "invalid_entry"))
EVICTION_REASONS = frozenset(("validation_rejected", "identity_changed", "manual"))
_REQUEST_FIELDS = {"schema_version", "joint_names", "identity", "initial_rad", "target_rad",
                   "goal_tolerance_rad", "start_tolerance_rad"}


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _pairs(items):
    value = {}
    for key, item in items:
        _require(key not in value, "Duplicate cached JSON key")
        value[key] = item
    return value


@dataclass(frozen=True)
class _Entry:
    raw: bytes
    sha256: str
    created_ns: int


class TrajectoryCache:
    """Small thread-safe LRU of complete, already owner-validated joint paths.

    ``identity`` has exactly the six ``IDENTITY_FIELDS``. Every value is a lower
    SHA-256 digest, except ``tool_sha256=None`` explicitly declares no tool.
    State dictionary iteration order is normalized by ``joint_names``; the spline
    names themselves must match that complete order exactly. ``start_tolerance_rad``
    is zero by default and never exceeds 0.001rad. An opt-in near-start hit only
    changes a returned copy's first positions to the *actual supplied* start.

    ``store`` accepts ``[Spline(...).record()]`` and ``lookup`` returns the same
    record layout in ``trajectories``. Successful lookup does not consume it.
    The owner validates that candidate before making a *new* PlanReviewStore
    issue; an old plan/session/approval identifier is never stored here.
    """
    def __init__(self, joint_names, *, max_entries=16, max_total_bytes=16 * 1024 * 1024,
                 ttl_sec=300., goal_tolerance_rad=.003, start_tolerance_rad=0., clock=time.monotonic_ns):
        _require(type(joint_names) in (list, tuple) and 1 <= len(joint_names) <= MAX_JOINTS,
                 "Expected bounded ordered cache joint names")
        _require(all(type(name) is str and re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,127}", name)
                     for name in joint_names) and len(set(joint_names)) == len(joint_names),
                 "Expected unique named cache joints")
        _require(type(max_entries) is int and 1 <= max_entries <= MAX_ENTRIES,
                 "Expected cache entry limit within 1..128")
        _require(type(max_total_bytes) is int and 1 <= max_total_bytes <= MAX_TOTAL_BYTES,
                 "Expected cache byte limit within 1..64MiB")
        _require(_finite(ttl_sec) and .001 <= ttl_sec <= 3600,
                 "Expected cache TTL within 0.001..3600 wall seconds")
        _require(_finite(goal_tolerance_rad) and 0 <= goal_tolerance_rad <= .01,
                 "Expected declared cache endpoint tolerance within 0..0.01rad")
        _require(_finite(start_tolerance_rad) and 0 <= start_tolerance_rad <= .001,
                 "Expected explicit cache start tolerance within 0..0.001rad")
        _require(callable(clock), "Expected monotonic nanosecond clock")
        self.joint_names = tuple(joint_names)
        self.max_entries, self.max_total_bytes = max_entries, max_total_bytes
        self.ttl_ns = round(float(ttl_sec) * 1e9)
        self.goal_tolerance_rad = float(goal_tolerance_rad)
        self.start_tolerance_rad = float(start_tolerance_rad)
        self.clock = clock
        self.lock = threading.RLock()
        self._entries = OrderedDict()
        self._last_clock_ns = None

    def _now(self):
        value = self.clock()
        _require(type(value) is int and value >= 0, "Cache clock must return monotonic integer nanoseconds")
        _require(self._last_clock_ns is None or value >= self._last_clock_ns,
                 "Cache monotonic clock moved backwards")
        self._last_clock_ns = value
        return value

    def _identity(self, identity):
        _require(type(identity) is dict and set(identity) == set(IDENTITY_FIELDS),
                 "Expected all six declared cache source identities")
        for name, value in identity.items():
            if name == "tool_sha256" and value is None:
                continue
            _require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value),
                     "Cache identities must be lowercase SHA-256; only no-tool may be null")
        return {name: identity[name] for name in IDENTITY_FIELDS}

    def _state(self, state):
        _require(type(state) is dict and set(state) == set(self.joint_names),
                 "Cache state must cover every configured joint exactly once")
        _require(all(_finite(value) and abs(value) <= 1e6 for value in state.values()),
                 "Expected finite bounded cache joint radians")
        # Normalize signed zero and integer/float forms without rounding a state.
        return {name: 0. if state[name] == 0 else float(state[name]) for name in self.joint_names}

    def _request(self, identity, initial_rad, target_rad):
        return dict(schema_version=1, joint_names=list(self.joint_names),
                    identity=self._identity(identity), initial_rad=self._state(initial_rad),
                    target_rad=self._state(target_rad), goal_tolerance_rad=self.goal_tolerance_rad,
                    start_tolerance_rad=self.start_tolerance_rad)

    def _trajectories(self, trajectories, request):
        _require(type(trajectories) is list and len(trajectories) == 1,
                 "Cache initially supports one complete coordinated spline")
        curve = trajectories[0]
        _require(type(curve) is dict and set(curve) == {"names", "points"}
                 and type(curve["names"]) is list and curve["names"] == list(self.joint_names),
                 "Cached spline must retain the complete declared joint order")
        points = curve["points"]
        _require(type(points) is list and 2 <= len(points) <= MAX_POINTS,
                 "Expected 2..2000 bounded cached spline points")
        normalized = []
        for point in points:
            _require(type(point) is dict and set(point) == {"t", "p", "v", "a"}
                     and _finite(point["t"]), "Expected exact finite cached spline point fields")
            result = dict(t=float(point["t"]))
            for name in ("p", "v", "a"):
                values = point[name]
                _require(type(values) is list and len(values) in ((len(self.joint_names),) if name == "p"
                         else (0, len(self.joint_names)))
                         and all(_finite(value) and abs(value) <= 1e6 for value in values),
                         "Expected complete finite cached spline vectors")
                result[name] = [0. if value == 0 else float(value) for value in values]
            normalized.append(result)
        # Reuse the configured controller spline's shape/time/derivative contract.
        result = Spline(list(self.joint_names), normalized).record()
        first, last = normalized[0], normalized[-1]
        _require(first["p"] == [request["initial_rad"][name] for name in self.joint_names],
                 "Cached path must start at the exact full state; reanchoring is unsupported")
        _require(all(value == 0 for name in ("v", "a") for value in first[name]),
                 "Cached path must start with anchored zero derivatives")
        _require(all(abs(value) <= 1e-8 for value in last["v"]),
                 "Cached path must finish with zero velocity")
        _require(max(abs(last["p"][index] - request["target_rad"][name])
                     for index, name in enumerate(self.joint_names)) <= self.goal_tolerance_rad + 1e-9,
                 "Cached path endpoint exceeds its declared goal tolerance")
        return [result]

    def _load(self, key, entry, now):
        _require(type(entry) is _Entry and type(entry.raw) is bytes
                 and 0 < len(entry.raw) <= min(MAX_ENTRY_BYTES, self.max_total_bytes)
                 and type(entry.created_ns) is int and 0 <= entry.created_ns <= now
                 and type(entry.sha256) is str and _sha(entry.raw) == entry.sha256,
                 "Invalid cached entry envelope or checksum")
        try:
            payload = json.loads(entry.raw, object_pairs_hook=_pairs)
        except (ValueError, UnicodeError, RecursionError) as error:
            raise ValueError("Invalid cached entry JSON") from error
        _require(type(payload) is dict and set(payload) == {"request", "trajectories"},
                 "Invalid cached entry fields")
        request = payload["request"]
        _require(type(request) is dict and set(request) == _REQUEST_FIELDS
                 and type(request["schema_version"]) is int and request["schema_version"] == 1
                 and request["joint_names"] == list(self.joint_names)
                 and type(request["goal_tolerance_rad"]) is float
                 and request["goal_tolerance_rad"] == self.goal_tolerance_rad
                 and type(request["start_tolerance_rad"]) is float
                 and request["start_tolerance_rad"] == self.start_tolerance_rad,
                 "Cached request differs from this cache contract")
        checked = self._request(request["identity"], request["initial_rad"], request["target_rad"])
        curves = self._trajectories(payload["trajectories"], checked)
        _require(_canonical(dict(request=checked, trajectories=curves)) == entry.raw
                 and _sha(_canonical(checked)) == key, "Cached entry content or request identity changed")
        return checked, curves

    def _cost(self, started, **details):
        return dict(details, elapsed_wall_sec=(self._now() - started) / 1e9,
                    entries=len(self._entries), stored_bytes=sum(len(item.raw) for item in self._entries.values()),
                    start_tolerance_rad=self.start_tolerance_rad)

    def lookup(self, identity, initial_rad, target_rad):
        """Return a detached candidate and measured cost, never approval to move.

        Invalid caller input or clock failures raise ValueError. Only malformed
        *stored* entries are evicted as cache misses; no cancellation/budget or
        environment-validation callback is called or caught by this module.
        """
        with self.lock:
            started = self._now()
            request = self._request(identity, initial_rad, target_rad)
            key = _sha(_canonical(request))
            valid = OrderedDict()
            invalid, expired = [], []
            expired_requests = []
            inspected = len(self._entries)
            for candidate, entry in list(self._entries.items()):
                try:
                    checked, curves = self._load(candidate, entry, started)
                except (ValueError, TypeError, KeyError, OverflowError):
                    invalid.append(candidate)
                    del self._entries[candidate]
                    continue
                if started - entry.created_ns >= self.ttl_ns:
                    expired.append(candidate)
                    # Retain only checked request metadata for this lookup's
                    # diagnosis. Expired paths are deleted and never reused.
                    expired_requests.append(checked)
                    del self._entries[candidate]
                else:
                    valid[candidate] = (checked, curves)
            source_key = key if key in self._entries else None
            start_error = 0.
            if source_key is None and self.start_tolerance_rad > 0:
                # The actual observation is never changed. Nearest lookup uses
                # the maximum full-state error, with most recent LRU breaking
                # equal-distance ties. Only an exact goal/identity may match.
                best = float("inf")
                for candidate in reversed(valid):
                    source = valid[candidate][0]
                    if source["identity"] != request["identity"] or source["target_rad"] != request["target_rad"]:
                        continue
                    difference = max(abs(source["initial_rad"][name] - request["initial_rad"][name])
                                     for name in self.joint_names)
                    if difference <= self.start_tolerance_rad and difference < best:
                        source_key, start_error, best = candidate, difference, difference
            entry = self._entries.get(source_key)
            reason, mismatch_fields = "cold", []
            if entry is not None:
                self._entries.move_to_end(source_key)
                source, curves = valid[source_key]
                reanchored = start_error > 0
                if reanchored:
                    curves[0]["points"][0]["p"] = [request["initial_rad"][name] for name in self.joint_names]
                    # Changing a first knot changes the first interpolation
                    # segment. Repeat structural checks; the owner must still
                    # revalidate the *whole* resulting curve with both engines.
                    curves = self._trajectories(curves, request)
                candidate_sha = _sha(_canonical(dict(request=request, trajectories=curves)))
                return self._cost(started, hit=True, reason="hit", trajectories=curves,
                    key_sha256=source_key, request_key_sha256=key, entry_sha256=entry.sha256,
                    source_entry_sha256=entry.sha256, candidate_sha256=candidate_sha,
                    source_initial_rad=dict(source["initial_rad"]), current_initial_rad=dict(request["initial_rad"]),
                    start_max_error_rad=start_error, reanchored=reanchored,
                    entry_age_wall_sec=(started - entry.created_ns) / 1e9,
                    lookup_wall_sec=(self._now() - started) / 1e9,
                    inspected_entries=inspected, evicted_invalid=len(invalid), evicted_expired=len(expired),
                    mismatch_fields=[], full_revalidation_required=True)
            if key in invalid:
                reason = "invalid_entry"
            elif any(candidate["identity"] == request["identity"]
                     and candidate["target_rad"] == request["target_rad"]
                     and max(abs(candidate["initial_rad"][name] - request["initial_rad"][name])
                             for name in self.joint_names) <= self.start_tolerance_rad
                     for candidate in expired_requests):
                reason = "expired"
            elif valid:
                # Diagnostic comparisons never select an approximate path. Prefer
                # a same-identity entry, then report the most recently used one.
                candidates = [item for item, _ in valid.values() if item["identity"] == request["identity"]]
                same_identity = bool(candidates)
                if not candidates:
                    candidates = [item for item, _ in valid.values()]
                candidate = candidates[-1]
                if not same_identity:
                    reason = "identity_mismatch"
                    mismatch_fields = [name for name in IDENTITY_FIELDS
                                       if candidate["identity"][name] != request["identity"][name]]
                else:
                    # Diagnose the declared start policy just as lookup does.
                    # This does not select a candidate: a changed goal still
                    # misses, even when harmless observed-state drift is within
                    # the explicitly configured near-start tolerance.
                    start = max(abs(candidate["initial_rad"][name] - request["initial_rad"][name])
                                for name in self.joint_names) <= self.start_tolerance_rad
                    goal = candidate["target_rad"] == request["target_rad"]
                    reason = "goal_mismatch" if start else "start_mismatch" if goal else "start_and_goal_mismatch"
            return self._cost(started, hit=False, reason=reason, trajectories=None,
                key_sha256=key, request_key_sha256=key, entry_sha256=None, entry_age_wall_sec=None,
                source_entry_sha256=None, candidate_sha256=None, source_initial_rad=None,
                current_initial_rad=dict(request["initial_rad"]), start_max_error_rad=None, reanchored=False,
                lookup_wall_sec=(self._now() - started) / 1e9,
                inspected_entries=inspected, evicted_invalid=len(invalid), evicted_expired=len(expired),
                mismatch_fields=mismatch_fields, full_revalidation_required=True)

    def store(self, identity, initial_rad, target_rad, trajectories):
        """Store an owner-validated *fresh* path; structural checks repeat here.

        The owner must never store a hit/reanchored candidate. Reusing it is not
        a fresh plan and cannot extend the source TTL or accumulate anchor drift.
        """
        with self.lock:
            started = self._now()
            request = self._request(identity, initial_rad, target_rad)
            key = _sha(_canonical(request))
            curves = self._trajectories(trajectories, request)
            raw = _canonical(dict(request=request, trajectories=curves))
            _require(len(raw) <= min(MAX_ENTRY_BYTES, self.max_total_bytes),
                     "Cached candidate exceeds the per-entry or total byte bound")
            old = self._entries.pop(key, None)
            evicted = 0
            while self._entries and (len(self._entries) >= self.max_entries
                    or sum(len(item.raw) for item in self._entries.values()) + len(raw) > self.max_total_bytes):
                self._entries.popitem(last=False)
                evicted += 1
            entry = _Entry(raw=raw, sha256=_sha(raw), created_ns=started)
            self._entries[key] = entry
            return self._cost(started, stored=True, replaced=old is not None, key_sha256=key,
                entry_sha256=entry.sha256, entry_bytes=len(raw), evicted_lru=evicted,
                full_revalidation_required=True)

    def evict(self, key_sha256, reason="validation_rejected"):
        """Discard a candidate rejected by the owner's current full validators."""
        _require(type(key_sha256) is str and re.fullmatch(r"[0-9a-f]{64}", key_sha256),
                 "Expected exact cache key digest for eviction")
        _require(type(reason) is str and reason in EVICTION_REASONS,
                 "Expected bounded explicit cache eviction reason")
        with self.lock:
            started = self._now()
            entry = self._entries.pop(key_sha256, None)
            return self._cost(started, evicted=entry is not None, reason=reason, key_sha256=key_sha256)
