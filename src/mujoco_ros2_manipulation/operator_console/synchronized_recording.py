"""Immutable, bounded camera/3D recordings on one declared ROS acquisition epoch.

Acquisition timestamps remain decimal integer nanoseconds. Playback uses one
cursor to select both tracks; it never interpolates poses, fills image gaps,
imports ROS, or dispatches robot commands. Identity declarations are validated,
but only the capture owner can establish their association with a live source.
"""
from bisect import bisect_left, bisect_right
from copy import deepcopy
import hashlib
import math
import os
from pathlib import Path
import re
import stat
import threading
import time
from types import MappingProxyType
import uuid

from .display_contract import validate_manifest, validate_pose
from .scene_host import _canonical, _json, _require, MAX_MANIFEST_BYTES, MAX_TOTAL_ASSET_BYTES
from .view_binding import _checked_binding, make_binding, binding_id, envelope

NS_PER_SEC = 1_000_000_000
MAX_STAMP_NS = (2**31) * NS_PER_SEC - 1
MAX_DURATION_NS = 3600 * NS_PER_SEC
MAX_RECORDING_BYTES = 64 * 1024 * 1024
MAX_POSES, MAX_IMAGES = 20_000, 20_000
MAX_JPEG_BYTES = 512 * 1024
MAX_TOTAL_IMAGE_BYTES = 128 * 1024 * 1024
MAX_SAMPLE_AGE_NS = NS_PER_SEC // 2
MAX_SEQUENCE = 2**53 - 1
SPEEDS = (.25, .5, 1., 2.)
_SHA = re.compile(r"[0-9a-f]{64}")
_UUID = re.compile(r"[0-9a-f]{32}")
_NS = re.compile(r"0|[1-9][0-9]{0,18}")
_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_/]*")
_DOCUMENT_FIELDS = {"schema_version", "kind", "label", "manifest", "identity", "origin_ns",
                    "poses", "images", "ended_reason"}
_IDENTITY_FIELDS = {"binding", "operator_session_id", "motion_session_id", "app_generation",
                    "image_generation", "image_source_id", "topic", "publisher_endpoint_sha256", "clock_domain"}


def _decimal_ns(value):
    _require(type(value) is str and _NS.fullmatch(value) is not None, "Invalid decimal nanosecond stamp")
    number = int(value)
    _require(number <= MAX_STAMP_NS, "Nanosecond stamp exceeds ROS time range")
    return number


def _sequence(value):
    _require(type(value) is int and 1 <= value <= MAX_SEQUENCE, "Invalid recording sequence")
    return value


def _identity(value):
    _require(type(value) is dict and set(value) == _IDENTITY_FIELDS, "Invalid recording identity fields")
    result = deepcopy(value)
    binding = _checked_binding(result["binding"])
    _require(binding["mode"] == "live", "Only observed live poses may be recorded with sensor images")
    for name in ("operator_session_id", "image_generation"):
        _require(type(result[name]) is str and _UUID.fullmatch(result[name]) is not None,
                 "Invalid recording " + name)
    motion = result["motion_session_id"]
    _require(type(motion) is str and _UUID.fullmatch(motion) is not None, "Missing recording motion session")
    _require(binding["operator_session_id"] == result["operator_session_id"]
             and binding["motion_session_id"] == motion, "Recording binding/session mismatch")
    _require(type(result["app_generation"]) is int and 0 <= result["app_generation"] <= MAX_SEQUENCE,
             "Invalid application generation")
    for name, limit in (("image_source_id", 128), ("topic", 256)):
        pattern = _NAME if name == "image_source_id" else re.compile(r"/[A-Za-z_][A-Za-z0-9_/]*")
        _require(type(result[name]) is str and 0 < len(result[name]) <= limit
                 and pattern.fullmatch(result[name]) is not None, "Invalid image " + name)
    _require(result["topic"].startswith("/") and "//" not in result["topic"], "Image topic must be absolute")
    _require(type(result["publisher_endpoint_sha256"]) is str
             and _SHA.fullmatch(result["publisher_endpoint_sha256"]) is not None,
             "Invalid image publisher endpoint digest")
    _require(result["clock_domain"] == "ros_simulation", "Synchronized capture requires one ROS simulation clock")
    return result


def _jpeg_dimensions(raw):
    """Read bounded JPEG headers without importing a codec or decompressing data."""
    _require(raw.startswith(b"\xff\xd8") and raw.endswith(b"\xff\xd9"), "Invalid JPEG markers")
    offset = 2
    while offset + 4 <= len(raw):
        _require(raw[offset] == 255, "Invalid JPEG header")
        while offset < len(raw) and raw[offset] == 255:
            offset += 1
        _require(offset < len(raw), "Truncated JPEG marker")
        marker = raw[offset]
        offset += 1
        _require(marker not in (0, 0xD8, 0xD9, 0xDA) and not 0xD0 <= marker <= 0xD7,
                 "JPEG dimensions missing before scan")
        _require(offset + 2 <= len(raw), "Truncated JPEG segment")
        length = int.from_bytes(raw[offset:offset + 2], "big")
        _require(length >= 2 and offset + length <= len(raw), "Invalid JPEG segment length")
        if marker in (0xC0, 0xC2):
            _require(length >= 8 and raw[offset + 2] == 8, "Unsupported JPEG precision")
            height = int.from_bytes(raw[offset + 3:offset + 5], "big")
            width = int.from_bytes(raw[offset + 5:offset + 7], "big")
            components = raw[offset + 7]
            _require(components in (1, 3) and length == 8 + 3 * components, "Unsupported JPEG components")
            return width, height
        _require(marker not in (0xC1, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF),
                 "Unsupported JPEG encoding")
        offset += length
    raise ValueError("JPEG dimensions missing")


def _directory(path, *, create=False):
    """Open every directory component without following symlinks, including parents."""
    path = Path(path).absolute()
    _require(".." not in path.parts, "Recording paths cannot traverse parent directories")
    flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    descriptor = os.open(path.anchor, flags)
    try:
        for part in path.parts[1:]:
            if create:
                try:
                    os.mkdir(part, 0o700, dir_fd=descriptor)
                except FileExistsError:
                    pass
            child = os.open(part, flags, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _read(directory, name, maximum):
    descriptor = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory)
    try:
        info = os.fstat(descriptor)
        _require(stat.S_ISREG(info.st_mode) and 0 <= info.st_size <= maximum, "Recording file exceeds bounds or is not regular")
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            raw = stream.read(maximum + 1)
        _require(len(raw) <= maximum, "Recording file grew beyond limit")
        return raw
    finally:
        os.close(descriptor)


def _publish(directory, name, raw, *, reuse=False):
    """Publish complete bytes exclusively; never replace an existing bundle member."""
    if reuse:
        try:
            existing = _read(directory, name, len(raw))
        except FileNotFoundError:
            pass
        else:
            _require(existing == raw, "Existing recording member differs")
            return
    temporary = ".recording-" + uuid.uuid4().hex
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=directory)
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(descriptor)
        os.link(temporary, name, src_dir_fd=directory, dst_dir_fd=directory, follow_symlinks=False)
        os.fsync(directory)
    finally:
        os.close(descriptor)
        os.unlink(temporary, dir_fd=directory)


class SyncRecording:
    """A checked immutable bundle. Public structured values are defensive copies."""
    def __init__(self, document, assets=None, images=None):
        _require(type(document) is dict and set(document) == _DOCUMENT_FIELDS
                 and type(document["schema_version"]) is int and document["schema_version"] == 1
                 and document["kind"] == "synchronized-recording-v1", "Invalid synchronized recording")
        _require(type(document["label"]) is str and 0 < len(document["label"]) <= 128, "Invalid recording label")
        _require(type(document["ended_reason"]) is str and 0 < len(document["ended_reason"]) <= 256
                 and all(ord(character) >= 32 for character in document["ended_reason"]), "Invalid recording end reason")
        origin = _decimal_ns(document["origin_ns"])
        manifest, identity = validate_manifest(document["manifest"]), _identity(document["identity"])
        _require(len(_canonical(manifest)) <= MAX_MANIFEST_BYTES, "Recording manifest exceeds limit")
        poses, image_frames = document["poses"], document["images"]
        _require(type(poses) is list and 2 <= len(poses) <= MAX_POSES, "Invalid recording pose count")
        _require(type(image_frames) is list and 1 <= len(image_frames) <= MAX_IMAGES, "Invalid recording image count")
        checked_poses, checked_images = [], []
        for track, target, fields in ((poses, checked_poses, {"stamp_ns", "sequence", "positions_rad", "pose"}),
                                      (image_frames, checked_images, {"stamp_ns", "sequence", "jpeg_sha256", "byte_length", "width", "height"})):
            previous_stamp, previous_sequence = origin - 1, 0
            for item in track:
                _require(type(item) is dict and set(item) == fields, "Invalid recording track sample fields")
                stamp, sequence = _decimal_ns(item["stamp_ns"]), _sequence(item["sequence"])
                _require(previous_stamp < stamp <= origin + MAX_DURATION_NS and sequence > previous_sequence,
                         "Recording track timestamps and sequences must strictly advance within one epoch")
                copied = deepcopy(item)
                if track is poses:
                    positions = item["positions_rad"]
                    _require(type(positions) is dict and set(positions) == set(manifest["joint_names"])
                             and all(type(value) in (int, float) and abs(value) <= 1e6 and math.isfinite(value)
                                     for value in positions.values()), "Recording must cover named joints exactly")
                    copied["pose"] = validate_pose(item["pose"], manifest)
                else:
                    _require(type(item["jpeg_sha256"]) is str and _SHA.fullmatch(item["jpeg_sha256"]) is not None,
                             "Invalid recording image digest")
                    _require(type(item["byte_length"]) is int and 0 < item["byte_length"] <= MAX_JPEG_BYTES,
                             "Recording JPEG exceeds limit")
                    _require(type(item["width"]) is int and type(item["height"]) is int
                             and 0 < item["width"] <= 640 and 0 < item["height"] <= 480, "Recording image dimensions exceed limit")
                target.append(copied)
                previous_stamp, previous_sequence = stamp, sequence
        _require(min(int(poses[0]["stamp_ns"]), int(image_frames[0]["stamp_ns"])) == origin,
                 "Recording origin must be the first acquisition stamp")
        duration = max(int(poses[-1]["stamp_ns"]), int(image_frames[-1]["stamp_ns"])) - origin
        _require(0 < duration <= MAX_DURATION_NS, "Invalid recording duration")
        asset_bytes, image_bytes = dict(assets or {}), dict(images or {})
        _require(set(asset_bytes) == {item["sha256"] for item in manifest["assets"]}, "Incomplete recording assets")
        _require(set(image_bytes) == {item["jpeg_sha256"] for item in checked_images}, "Incomplete recording images")
        for item in manifest["assets"]:
            raw = asset_bytes[item["sha256"]]
            _require(type(raw) is bytes and len(raw) == item["byte_length"]
                     and hashlib.sha256(raw).hexdigest() == item["sha256"], "Recording asset digest mismatch")
        _require(sum(map(len, asset_bytes.values())) <= MAX_TOTAL_ASSET_BYTES, "Recording asset budget exceeded")
        dimensions = {}
        for item in checked_images:
            digest = item["jpeg_sha256"]
            raw = image_bytes[digest]
            _require(type(raw) is bytes and len(raw) == item["byte_length"]
                     and hashlib.sha256(raw).hexdigest() == digest, "Recording image digest mismatch")
            if digest not in dimensions:
                dimensions[digest] = _jpeg_dimensions(raw)
            _require(dimensions[digest] == (item["width"], item["height"]), "JPEG dimensions do not match recording")
        _require(sum(map(len, image_bytes.values())) <= MAX_TOTAL_IMAGE_BYTES, "Recording image budget exceeded")
        self._document = dict(schema_version=1, kind="synchronized-recording-v1", label=document["label"],
            manifest=manifest, identity=identity, origin_ns=document["origin_ns"], poses=checked_poses,
            images=checked_images, ended_reason=document["ended_reason"])
        self._raw = _canonical(self._document)
        _require(len(self._raw) <= MAX_RECORDING_BYTES, "Recording document exceeds limit")
        self._sha256 = hashlib.sha256(self._raw).hexdigest()
        self._assets, self._images = MappingProxyType(asset_bytes), MappingProxyType(image_bytes)
        self._origin_ns, self._duration_ns = origin, duration
        self._pose_stamps = tuple(int(item["stamp_ns"]) for item in checked_poses)
        self._image_stamps = tuple(int(item["stamp_ns"]) for item in checked_images)

    @property
    def document(self): return deepcopy(self._document)
    @property
    def manifest(self): return deepcopy(self._document["manifest"])
    @property
    def identity(self): return deepcopy(self._document["identity"])
    @property
    def poses(self): return deepcopy(self._document["poses"])
    @property
    def image_frames(self): return deepcopy(self._document["images"])
    @property
    def label(self): return self._document["label"]
    @property
    def ended_reason(self): return self._document["ended_reason"]
    @property
    def sha256(self): return self._sha256
    @property
    def assets(self): return self._assets
    @property
    def images(self): return self._images
    @property
    def origin_ns(self): return self._origin_ns
    @property
    def duration_ns(self): return self._duration_ns
    @property
    def duration(self): return self._duration_ns / NS_PER_SEC

    @classmethod
    def load(cls, filename):
        path = Path(filename).absolute()
        directory = _directory(path.parent)
        try:
            value = _json(_read(directory, path.name, MAX_RECORDING_BYTES))
            _require(type(value) is dict and set(value) == _DOCUMENT_FIELDS, "Invalid synchronized recording fields")
            manifest = validate_manifest(value["manifest"])
            entries = value["images"]
            _require(type(entries) is list and 1 <= len(entries) <= MAX_IMAGES, "Invalid recording image count")
            assets, images = {}, {}
            for folder, catalog, result, suffix, digest_key, budget in (
                    ("assets", manifest["assets"], assets, ".bin", "sha256", MAX_TOTAL_ASSET_BYTES),
                    ("images", entries, images, ".jpg", "jpeg_sha256", MAX_TOTAL_IMAGE_BYTES)):
                if not catalog:
                    continue
                child = os.open(folder, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
                try:
                    total = 0
                    for item in catalog:
                        _require(type(item) is dict and type(item.get(digest_key)) is str
                                 and _SHA.fullmatch(item[digest_key]) is not None, "Invalid recording member digest")
                        digest, size = item[digest_key], item.get("byte_length")
                        maximum = MAX_JPEG_BYTES if folder == "images" else MAX_TOTAL_ASSET_BYTES
                        _require(type(size) is int and 0 < size <= maximum, "Invalid recording member length")
                        if digest not in result:
                            total += size
                            _require(total <= budget, "Recording member budget exceeded")
                            result[digest] = _read(child, digest + suffix, size)
                finally:
                    os.close(child)
            return cls(value, assets, images)
        finally:
            os.close(directory)

    def save(self, filename):
        """Publish a complete bundle, rejecting overwrite and all symlink parents."""
        path = Path(filename).absolute()
        directory = _directory(path.parent, create=True)
        try:
            try:
                os.stat(path.name, dir_fd=directory, follow_symlinks=False)
            except FileNotFoundError:
                pass
            else:
                raise FileExistsError("Recording already exists")
            for folder, values, suffix in (("assets", self._assets, ".bin"), ("images", self._images, ".jpg")):
                if not values:
                    continue
                try:
                    os.mkdir(folder, 0o700, dir_fd=directory)
                except FileExistsError:
                    pass
                child = os.open(folder, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
                try:
                    for digest, raw in values.items():
                        _publish(child, digest + suffix, raw, reuse=True)
                finally:
                    os.close(child)
            _publish(directory, path.name, self._raw)
        finally:
            os.close(directory)
        return path


class SynchronizedSceneHost:
    """One server cursor and one atomic descriptor for two immutable tracks."""
    def __init__(self, recording, *, clock=time.monotonic):
        _require(isinstance(recording, SyncRecording), "Expected a checked synchronized recording")
        self.recording, self.clock, self.lock = recording, clock, threading.RLock()
        self.operator_session_id = uuid.uuid4().hex
        self.generation, self.closed, self.playing, self.speed = 0, False, False, 1.
        self._position_ns = self._anchor_position_ns = 0
        self._anchor_clock = self._last_clock = self._now()
        self.manifest = recording.manifest
        self.raw = _canonical(self.manifest) + b"\n"
        self._steps = tuple(sorted(set(recording._pose_stamps + recording._image_stamps)))
        self._bind()

    def _now(self):
        value = self.clock()
        _require(type(value) in (int, float) and math.isfinite(value), "Invalid playback monotonic clock")
        return value

    def _bind(self):
        self.generation += 1
        self.binding = make_binding(self.operator_session_id, self.generation, "scene:synchronized-recording", "live")
        self.identifier = binding_id(self.binding)

    def _tick(self):
        now = self._now()
        _require(now >= self._last_clock, "Playback monotonic clock reversed")
        self._last_clock = now
        if self.playing:
            elapsed = round((now - self._anchor_clock) * NS_PER_SEC * self.speed)
            self._position_ns = min(self.recording.duration_ns, self._anchor_position_ns + elapsed)
            if self._position_ns == self.recording.duration_ns:
                self.playing = False
        return now

    def _ready(self, identifier):
        _require(not self.closed and identifier == self.identifier, "Recording binding changed")

    def _selection(self):
        cursor = self.recording.origin_ns + self._position_ns
        selected, indices = {}, {}
        for name, stamps, track in (("pose", self.recording._pose_stamps, self.recording._document["poses"]),
                                     ("image", self.recording._image_stamps, self.recording._document["images"])):
            index = bisect_right(stamps, cursor) - 1
            indices[name] = index
            stamp = None if index < 0 else stamps[index]
            age = None if stamp is None else cursor - stamp
            ready = not self.closed and age is not None and age <= MAX_SAMPLE_AGE_NS
            selected[name + "_status"] = "ready" if ready else "missing"
            selected[name + "_time_sec"] = None if stamp is None else (stamp - self.recording.origin_ns) / NS_PER_SEC
            selected[name + "_delta_sec"] = None if age is None else age / NS_PER_SEC
            # The pose envelope validator makes its own defensive pose copy.
            # Avoid copying a complete geometry catalog twice on every tick.
            selected[name] = (track[index] if name == "pose" else deepcopy(track[index])) if ready else None
        item = selected["pose"]
        if item is not None:
            result = envelope(self.binding, item["pose"], item["sequence"],
                              source_state_age_sec=selected["pose_delta_sec"], simulation_sec=None)
            result.update(schema_version=2, source=dict(session_id=self.operator_session_id,
                clock_domain="recording:" + self.recording.sha256, stamp_sec=selected["pose_time_sec"],
                stamp_ns=item["stamp_ns"],
                sequence=item["sequence"], joint_positions_rad=deepcopy(item["positions_rad"])))
            selected["pose"] = result
        selected.update(cursor_ns=str(cursor), time_sec=self._position_ns / NS_PER_SEC)
        return selected, indices

    def _snapshot(self):
        selection, indices = self._selection()
        return dict(schema_version=1, status="closed" if self.closed else "ready",
            binding_id=self.identifier, binding=deepcopy(self.binding),
            source=dict(id="scene:synchronized-recording", label=self.recording.label, kind="synchronized-recording", readonly=True),
            manifest_descriptor=dict(sha256=hashlib.sha256(self.raw).hexdigest(), byte_length=len(self.raw),
                model_sha256=self.manifest["model_sha256"], scene_sha256=self.manifest["scene_sha256"]),
            sequence=(0 if indices["pose"] < 0 else self.recording._document["poses"][indices["pose"]]["sequence"]),
            pose_ready=selection["pose"] is not None, worker_running=False,
            position_sec=self._position_ns / NS_PER_SEC, duration_sec=self.recording.duration,
            playing=self.playing, speed=self.speed, recording_sha256=self.recording.sha256,
            origin_ns=str(self.recording.origin_ns), acquisition_clock_domain="ros_simulation",
            acquisition_identity=self.recording.identity,
            selection=selection, error=None)

    def snapshot(self):
        with self.lock:
            self._tick()
            return self._snapshot()

    def control(self, action, position=None, speed=None):
        with self.lock:
            self._ready(self.identifier)
            _require(action in ("play", "pause", "seek", "speed", "previous", "next"), "Only read-only playback controls are supported")
            _require(position is None or action == "seek", "Only seek accepts position")
            _require(speed is None or action == "speed", "Only speed accepts a rate")
            if action == "seek":
                _require(type(position) in (int, float) and 0 <= position <= self.recording.duration
                         and math.isfinite(position), "Playback time is outside recording")
            if action == "speed":
                _require(type(speed) in (int, float) and speed in SPEEDS, "Unsupported playback speed")
            now = self._tick()
            if action == "seek":
                self._position_ns = min(self.recording.duration_ns, round(position * NS_PER_SEC))
                self._bind()
            elif action in ("previous", "next"):
                cursor = self.recording.origin_ns + self._position_ns
                index = bisect_left(self._steps, cursor) - 1 if action == "previous" else bisect_right(self._steps, cursor)
                self._position_ns = self._steps[max(0, min(index, len(self._steps) - 1))] - self.recording.origin_ns
                self.playing = False
                self._bind()
            elif action == "speed":
                self.speed = float(speed)
            elif action == "pause":
                self.playing = False
            else:
                if self._position_ns >= self.recording.duration_ns:
                    self._position_ns = 0
                    self._bind()
                self.playing = True
            self._anchor_position_ns, self._anchor_clock = self._position_ns, now
            return self._snapshot()

    def manifest_bytes(self, identifier):
        with self.lock:
            self._ready(identifier)
            return self.raw

    def asset_bytes(self, digest):
        with self.lock:
            self._ready(self.identifier)
            _require(type(digest) is str and digest in self.recording.assets, "Unknown recording asset")
            return self.recording.assets[digest]

    def image_bytes(self, digest):
        with self.lock:
            self._ready(self.identifier)
            _require(type(digest) is str and digest in self.recording.images, "Unknown recording image")
            return self.recording.images[digest]

    def close(self):
        with self.lock:
            self.closed, self.playing = True, False
