"""Bounded, identity-checked observations from the native grasp plugin.

This module does not reconstruct contacts or modify physics.  Native timestamps
are rounded ``mjData.time`` values, not exact ROS integer timestamps.  Receiving
the same physics time again must never make a held simulation look fresh.
"""
from dataclasses import dataclass, field
import json
import math
import re
import threading


_KIND = "ros-grasp-observation-v1"
_MAX_BYTES = 256 * 1024
_MAX_NUMBER = 1e6
_CONTROLLERS = ("left_controller", "right_controller", "gripper_controller")
_FINGERS = ("fr5_left_finger_slide", "fr5_right_finger_slide")
_ARMS = tuple("left_j" + str(i) for i in range(1, 7)) + (
    "right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_elbow_joint",
    "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint",
)


class ObservationUnavailable(ValueError):
    """No checked observation is eligible for the requested use."""


@dataclass(frozen=True)
class IngestResult:
    accepted: bool
    reason: str = ""


@dataclass(frozen=True)
class GraspObservation:
    """An immutable observation; nested public payloads are detached copies."""

    receipt_sec: float
    clock_progress_receipt_sec: float
    clock_has_advanced: bool
    _json: bytes = field(repr=False)

    @property
    def payload(self):
        return json.loads(self._json)

    @property
    def sequence(self):
        return self.payload["sequence"]

    @property
    def time_sec(self):
        return self.payload["time_sec"]

    @property
    def stamp_ns(self):
        """Rounded native time as a decimal string; it is not exact ROS time."""
        return self.payload["stamp_ns"]

    @property
    def qpos(self):
        return tuple(self.payload["qpos"])

    @property
    def qvel(self):
        return tuple(self.payload["qvel"])

    @property
    def arm_positions_rad(self):
        value = self.payload["arm_positions_rad"]
        return tuple(value[name] for name in _ARMS)

    @property
    def finger_positions_m(self):
        value = self.payload["finger_positions_m"]
        return tuple(value[name] for name in _FINGERS)

    @property
    def force_timing_verified(self):
        return False


def _hex(value, length, label):
    if type(value) is not str or re.fullmatch("[0-9a-f]{%d}" % length, value) is None:
        raise ValueError(label + " must be lowercase hexadecimal")
    return value


def _number(value, label, maximum=_MAX_NUMBER, minimum=None):
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > maximum:
        raise ValueError(label + " must be a bounded finite number")
    if minimum is not None and value < minimum:
        raise ValueError(label + " is below its allowed minimum")
    return float(value)


def _integer(value, label, low=0, high=2**53 - 1):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(label + " must be a bounded integer")
    return value


def _vector(value, width, label):
    if type(value) is not list or len(value) != width:
        raise ValueError(label + " has the wrong width")
    return tuple(_number(item, label) for item in value)


def _quaternion(value, label):
    result = _vector(value, 4, label)
    if abs(sum(item * item for item in result) - 1) > 1e-6:
        raise ValueError(label + " must be a unit wxyz quaternion")
    return result


def _rotation(value, label):
    result = _vector(value, 9, label)
    rows = tuple(result[i:i + 3] for i in (0, 3, 6))
    for i in range(3):
        for j in range(3):
            if abs(sum(a * b for a, b in zip(rows[i], rows[j])) - (i == j)) > 1e-6:
                raise ValueError(label + " must be orthonormal")
    a, b, c, d, e, f, g, h, i = result
    if abs(a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g) - 1) > 1e-6:
        raise ValueError(label + " must be a proper rotation")
    return result


def _no_duplicate_keys(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError("duplicate JSON key: " + key)
        value[key] = item
    return value


def _decode(payload):
    if type(payload) is dict:
        encoded = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode()
    elif type(payload) is str:
        encoded = payload.encode("utf-8")
    elif type(payload) is bytes:
        encoded = payload
    else:
        raise ValueError("observation must be a JSON object or UTF-8 JSON")
    if len(encoded) > _MAX_BYTES:
        raise ValueError("observation exceeds the native JSON bound")
    value = json.loads(encoded, object_pairs_hook=_no_duplicate_keys)
    if type(value) is not dict:
        raise ValueError("observation must be a JSON object")
    return value


class GraspObservationStore:
    """Validate one fixed native run and gate physical use with its guard.

    ``manifest`` is the already SHA-checked execution target manifest, and
    ``geom_names`` is the compiled model's full ID-ordered geometry catalogue.
    A clock reversal permanently invalidates this store.  A new process needs a
    new run ID and a new store; callers cannot clear that fault in place.
    """

    def __init__(self, *, run_id, model_sha256, plugin_sha256, manifest, geom_names):
        self.run_id = _hex(run_id, 32, "run_id")
        self.model_sha256 = _hex(model_sha256, 64, "model_sha256")
        self.plugin_sha256 = _hex(plugin_sha256, 64, "plugin_sha256")
        if type(manifest) is not dict:
            raise ValueError("a checked grasp manifest is required")
        if manifest.get("artifact_kind") != "grasp-execution-target-v1":
            raise ValueError("unexpected execution target kind")
        if manifest.get("arm_joint_names") != list(_ARMS) or manifest.get("finger_joint_names") != list(_FINGERS):
            raise ValueError("unexpected scalar joint names or units")
        units = {name: "rad" for name in _ARMS}
        units.update({name: "m" for name in _FINGERS})
        if manifest.get("scalar_joint_units") != units:
            raise ValueError("unexpected scalar joint names or units")
        layout = manifest.get("qpos_state_contract", {})
        if any(layout.get(key) != value for key, value in {"nq": 21, "nv": 20, "nu": 14, "njnt": 15}.items()):
            raise ValueError("unexpected full-state dimensions")
        rows = layout.get("layout")
        scalar_names = _ARMS[:6] + _FINGERS + _ARMS[6:]
        expected = [dict(name=name, type="slide" if name in _FINGERS else "hinge",
                         qpos_address=i, qpos_width=1, velocity_address=i)
                    for i, name in enumerate(scalar_names)]
        expected.append(dict(name="fr5_block_free", type="free", qpos_address=14,
                             qpos_width=7, velocity_address=14))
        if rows != expected:
            raise ValueError("unexpected named qpos/velocity layout")
        self._addresses = {row["name"]: row["qpos_address"] for row in expected}
        if type(geom_names) not in (tuple, list) or not 1 <= len(geom_names) <= 256:
            raise ValueError("compiled geometry catalogue is required")
        if any(type(name) is not str or len(name) > 128 for name in geom_names):
            raise ValueError("invalid compiled geometry names")
        named = [name for name in geom_names if name]
        if len(set(named)) != len(named) or not {
            "fr5_block_geom", "fr5_table_geom", "fr5_left_finger_geom", "fr5_right_finger_geom"
        }.issubset(named):
            raise ValueError("incomplete or ambiguous compiled geometry catalogue")
        self._geom_names = tuple(geom_names)
        self._lock = threading.RLock()
        self._latest = None
        self._error = "no native grasp observation received"
        self._fault = ""
        self._guard = None
        self._guard_receipt = None
        self._guard_error = "no contact guard observation received"

    def _validate(self, value):
        if type(value.get("schema_version")) is not int or value["schema_version"] != 1 or value.get("kind") != _KIND:
            raise ValueError("unsupported native grasp observation schema")
        for key, expected in (("run_id", self.run_id), ("model_sha256", self.model_sha256),
                              ("plugin_sha256", self.plugin_sha256)):
            if value.get(key) != expected:
                raise ValueError("native grasp " + key + " does not match the selected run")
        version = value.get("native_version")
        if type(version) is not str or re.fullmatch(r"\d+\.\d+\.\d+", version) is None or len(version) > 32:
            raise ValueError("invalid native version")
        _integer(value.get("sequence"), "sequence", low=1)
        time_sec = _number(value.get("time_sec"), "time_sec", maximum=2147483647., minimum=0)
        stamp = value.get("stamp_ns")
        if type(stamp) is not str or re.fullmatch(r"0|[1-9][0-9]{0,18}", stamp) is None:
            raise ValueError("native stamp_ns must be a canonical decimal string")
        rounded = time_sec * 1e9
        if abs(int(stamp) - rounded) > max(.501, math.ulp(rounded)):
            raise ValueError("native stamp_ns does not match rounded native time")
        if value.get("stamp_semantics") != "rounded_native_mjData_time":
            raise ValueError("native time must retain its rounded-time semantics")
        if _number(value.get("pose_sample_sec"), "pose_sample_sec", maximum=2147483647., minimum=0) != time_sec:
            raise ValueError("pose sample time does not match native time")
        force_time = _number(value.get("force_evaluation_sec"), "force_evaluation_sec", maximum=2147483647., minimum=0)
        if force_time > time_sec or time_sec - force_time > .010001:
            raise ValueError("invalid nominal force evaluation time")
        if value.get("force_timing_verified") is not False or value.get("force_time_semantics") != (
            "nominal last mj_step evaluation; invalid when native physics is held"
        ):
            raise ValueError("force timing must remain explicitly unverified")
        phase = _integer(value.get("phase"), "phase", low=-1, high=8)
        armed, heartbeat = value.get("armed"), value.get("heartbeat_fresh")
        if type(armed) is not bool or type(heartbeat) is not bool:
            raise ValueError("armed and heartbeat_fresh must be booleans")
        if heartbeat != (armed and phase >= 0) or (not armed and phase != 0):
            raise ValueError("inconsistent native phase/heartbeat state")
        _integer(value.get("local_overwrites_or_skips"), "local_overwrites_or_skips")
        qpos = _vector(value.get("qpos"), 21, "qpos")
        _vector(value.get("qvel"), 20, "qvel")
        for key, names in (("arm_positions_rad", _ARMS), ("finger_positions_m", _FINGERS)):
            positions = value.get(key)
            if type(positions) is not dict or set(positions) != set(names):
                raise ValueError(key + " must contain exactly the declared joints")
            for name in names:
                number = _number(positions[name], key + "." + name)
                if number != qpos[self._addresses[name]]:
                    raise ValueError(key + " disagrees with the named qpos layout")
        _quaternion(list(qpos[17:21]), "free block qpos quaternion")
        geoms = value.get("geoms")
        if type(geoms) is not list or len(geoms) != len(self._geom_names):
            raise ValueError("geometry count does not match the compiled model")
        for index, geom in enumerate(geoms):
            if type(geom) is not dict or type(geom.get("id")) is not int or geom["id"] != index:
                raise ValueError("geometry IDs must match the compiled order")
            _vector(geom.get("position"), 3, "geometry position")
            _rotation(geom.get("rotation"), "geometry rotation")
        for name in ("tcp", "block"):
            pose = value.get(name)
            if type(pose) is not dict:
                raise ValueError(name + " pose is missing")
            _vector(pose.get("position"), 3, name + " position")
            _quaternion(pose.get("quaternion_wxyz"), name + " quaternion")
        block = value["block"]
        if any(abs(a - b) > 1e-9 for a, b in zip(block["position"], qpos[14:17])):
            raise ValueError("free block position disagrees with qpos")
        bq = block["quaternion_wxyz"]
        if min(sum((a - b) ** 2 for a, b in zip(bq, qpos[17:21])),
               sum((a + b) ** 2 for a, b in zip(bq, qpos[17:21]))) > 1e-12:
            raise ValueError("free block orientation disagrees with qpos")
        _vector(block.get("linear_velocity_m_sec"), 3, "block linear velocity")
        _vector(block.get("angular_velocity_rad_sec"), 3, "block angular velocity")
        contacts = value.get("contacts")
        if type(contacts) is not list or len(contacts) > 256:
            raise ValueError("invalid native contact count")
        for contact in contacts:
            if type(contact) is not dict:
                raise ValueError("invalid native contact")
            for side in ("geom1", "geom2"):
                index = _integer(contact.get(side), side, high=len(self._geom_names) - 1)
                if contact.get(side + "_name") != self._geom_names[index]:
                    raise ValueError("contact geometry name does not match its compiled ID")
            if contact["geom1"] == contact["geom2"]:
                raise ValueError("contact cannot join a geometry to itself")
            _number(contact.get("normal_force_n"), "normal contact force", minimum=0)

    def ingest(self, payload, *, receipt_sec):
        """Accept a checked update; rejected duplicates never refresh any time."""
        with self._lock:
            if self._fault:
                return IngestResult(False, self._fault)
            try:
                receipt = _number(receipt_sec, "receipt_sec", maximum=1e12, minimum=0)
                value = _decode(payload)
                self._validate(value)
                previous = self._latest
                if previous is not None:
                    if receipt < previous.receipt_sec:
                        raise ValueError("observation receipt clock moved backwards")
                    if value["sequence"] <= previous.sequence:
                        return IngestResult(False, "duplicate or reordered native sequence")
                    if value["time_sec"] < previous.time_sec:
                        self._fault = "native grasp clock reversed; a new run is required"
                        self._error = self._fault
                        return IngestResult(False, self._fault)
                progressed = previous is not None and value["time_sec"] > previous.time_sec
                progress_receipt = receipt if previous is None or progressed else previous.clock_progress_receipt_sec
                has_advanced = progressed or (previous is not None and previous.clock_has_advanced)
                encoded = json.dumps(value, allow_nan=False, separators=(",", ":"), sort_keys=True).encode()
                self._latest = GraspObservation(receipt, progress_receipt, has_advanced, encoded)
                self._error = ""
                return IngestResult(True)
            except (ValueError, TypeError, OverflowError, RecursionError, UnicodeError) as exc:
                self._error = str(exc)
                return IngestResult(False, self._error)

    def update_guard(self, payload, *, receipt_sec):
        """Record contact-guard receipt separately from the native pose clock."""
        with self._lock:
            try:
                receipt = _number(receipt_sec, "guard receipt_sec", maximum=1e12, minimum=0)
                guard = _decode(payload)
                for key, expected in (("run_id", self.run_id), ("model_sha256", self.model_sha256)):
                    if key in guard and guard[key] != expected:
                        raise ValueError("contact guard " + key + " does not match the selected run")
                if type(guard.get("latched")) is not bool:
                    raise ValueError("contact guard latched state is missing")
                if type(guard.get("phase")) is not str or type(guard.get("reason")) is not str:
                    raise ValueError("contact guard phase/reason is missing")
                controllers = guard.get("controllers")
                if type(controllers) is not dict or any(type(controllers.get(name)) is not str for name in _CONTROLLERS):
                    raise ValueError("contact guard must observe all three controllers")
                if self._guard_receipt is not None and receipt < self._guard_receipt:
                    raise ValueError("contact guard receipt clock moved backwards")
                self._guard = json.dumps(guard, allow_nan=False, separators=(",", ":")).encode()
                self._guard_receipt = receipt
                self._guard_error = ""
                return IngestResult(True)
            except (ValueError, TypeError, OverflowError, RecursionError, UnicodeError) as exc:
                self._guard_error = str(exc)
                return IngestResult(False, self._guard_error)

    def snapshot(self, *, now_sec, max_age_sec=.5, require_physical=False):
        """Get one detached checked sample, optionally eligible for grasp checks.

        Physical eligibility requires a progressing native clock, phase
        heartbeat, and a fresh ready/unlatched guard observing three active
        controllers.  Force evaluation timing remains nominal even then.
        """
        now = _number(now_sec, "now_sec", maximum=1e12, minimum=0)
        max_age = _number(max_age_sec, "max_age_sec", maximum=5, minimum=0)
        if type(require_physical) is not bool:
            raise ValueError("require_physical must be boolean")
        with self._lock:
            if self._fault or self._error or self._latest is None:
                raise ObservationUnavailable(self._fault or self._error or "no native observation")
            latest = self._latest
            if now < latest.receipt_sec or now < latest.clock_progress_receipt_sec:
                raise ObservationUnavailable("snapshot receipt clock moved backwards")
            if now - latest.clock_progress_receipt_sec > max_age:
                raise ObservationUnavailable("native grasp clock is stale or held")
            if now - latest.receipt_sec > max_age:
                raise ObservationUnavailable("native grasp observation is stale")
            if require_physical:
                value = latest.payload
                if not latest.clock_has_advanced:
                    raise ObservationUnavailable("native grasp clock has not advanced")
                if not value["armed"] or not value["heartbeat_fresh"] or value["phase"] not in range(8):
                    raise ObservationUnavailable("native grasp phase heartbeat is not eligible")
                if self._guard_error or self._guard is None or self._guard_receipt is None:
                    raise ObservationUnavailable(self._guard_error or "no contact guard")
                if now < self._guard_receipt or now - self._guard_receipt > max_age:
                    raise ObservationUnavailable("contact guard observation is stale")
                guard = json.loads(self._guard)
                if guard["latched"] or guard["phase"] != "ready" or guard.get("error"):
                    raise ObservationUnavailable("contact guard is latched or not ready")
                if any(guard["controllers"][name] != "active" for name in _CONTROLLERS):
                    raise ObservationUnavailable("grasp requires three active controllers")
            return latest
