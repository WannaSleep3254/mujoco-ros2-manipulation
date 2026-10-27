"""Bounded, immutable display recordings and a receiver-local playback source.

Playback never imports ROS or a simulator, does FK, or dispatches commands.
Seek/restart starts a new binding; pause/EOF ages the last received pose.
"""
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import threading
import time
import uuid

from .display_contract import WORLD, VERSION, scene_digest, validate_manifest, validate_pose
from .scene_host import _canonical, _json, _regular, _require, MAX_MANIFEST_BYTES, MAX_TOTAL_ASSET_BYTES
from .view_binding import make_binding, binding_id, envelope

MAX_RECORDING_BYTES = 64 * 1024 * 1024
MAX_FRAMES = 20000


class Recording:
    def __init__(self, document, assets=None):
        _require(type(document) is dict and set(document) == {"schema_version", "kind", "label", "manifest", "frames"}
                 and type(document["schema_version"]) is int and document["schema_version"] == 1
                 and document["kind"] == "display-recording-v1"
                 and isinstance(document["label"], str) and 0 < len(document["label"]) <= 128,
                 "Invalid display recording")
        self.label = document["label"]
        self.manifest = validate_manifest(document["manifest"])
        _require(len(_canonical(self.manifest)) <= MAX_MANIFEST_BYTES, "Recording manifest exceeds limit")
        self.assets = dict(assets or {})
        _require(set(self.assets) == {item["sha256"] for item in self.manifest["assets"]}, "Incomplete recording assets")
        for item in self.manifest["assets"]:
            raw = self.assets[item["sha256"]]
            _require(type(raw) is bytes and len(raw) == item["byte_length"]
                     and hashlib.sha256(raw).hexdigest() == item["sha256"], "Recording asset digest mismatch")
        _require(sum(map(len, self.assets.values())) <= MAX_TOTAL_ASSET_BYTES, "Recording asset budget exceeded")
        frames = document["frames"]
        _require(type(frames) is list and 2 <= len(frames) <= MAX_FRAMES, "Invalid recording frame count")
        self.frames = []
        previous = -1.
        for item in frames:
            _require(type(item) is dict and set(item) == {"time_sec", "positions_rad", "pose"}, "Invalid recording sample")
            stamp = item["time_sec"]
            _require(type(stamp) in (int, float) and previous < stamp <= 3600 and math.isfinite(stamp),
                     "Recording time must strictly advance")
            positions = item["positions_rad"]
            _require(type(positions) is dict and set(positions) == set(self.manifest["joint_names"])
                     and all(type(value) in (int, float) and math.isfinite(value) and abs(value) <= 1e6
                             for value in positions.values()), "Recording must cover named joints exactly")
            pose = validate_pose(item["pose"], self.manifest)
            self.frames.append(dict(time_sec=stamp, positions_rad=deepcopy(positions), pose=pose))
            previous = stamp
        _require(self.frames[0]["time_sec"] == 0 and previous > 0, "Recording must start at zero")
        self.duration = previous
        self.sha256 = hashlib.sha256(_canonical(document)).hexdigest()

    @classmethod
    def load(cls, filename):
        path = Path(filename).absolute()
        raw = _regular(path, MAX_RECORDING_BYTES)
        value = _json(raw)
        manifest = validate_manifest(value["manifest"])
        folder = path.parent / "assets"
        _require(not folder.is_symlink(), "Recording assets directory cannot be linked")
        assets = {item["sha256"]: _regular(folder / (item["sha256"] + ".bin"), item["byte_length"])
                  for item in manifest["assets"]}
        return cls(value, assets)


class RecordedSceneHost:
    """Same manifest/assets/pose interface as the compiled producer adapter."""
    def __init__(self, recording, *, clock=time.monotonic):
        self.clock, self.lock = clock, threading.RLock()
        self.operator_session_id = uuid.uuid4().hex
        self.generation = 0
        self.closed = False
        self.select(recording)

    def _bind(self):
        self.generation += 1
        self.binding = make_binding(self.operator_session_id, self.generation, "scene:recording", "live")
        self.identifier = binding_id(self.binding)
        self.index = self.sequence = 0
        self.received = self.clock()

    def select(self, recording):
        _require(isinstance(recording, Recording), "Expected a checked recording")
        with self.lock:
            _require(not self.closed, "Recording source is closed")
            self.recording = recording
            self.manifest = deepcopy(recording.manifest)
            self.raw = _canonical(self.manifest) + b"\n"
            self.playing, self.position = False, 0.
            self._bind()
            self.origin = self.clock()
            self.sequence = 1

    def control(self, action, position=None):
        with self.lock:
            _require(not self.closed, "Recording source is closed")
            self._tick()
            _require(action in ("play", "pause", "seek"), "Only read-only playback controls are supported")
            if action == "seek":
                _require(type(position) in (int, float) and math.isfinite(position)
                         and 0 <= position <= self.recording.duration, "Playback time is outside recording")
                self.position = position
                self._bind()  # Rewind is an explicit new epoch, never an old sequence.
            else:
                _require(position is None, "Only seek accepts a position")
                if action == "play" and self.position >= self.recording.duration:
                    self.position = 0.
                    self._bind()
                self.playing = action == "play"
            self.origin = self.clock() - self.position
            self._tick(force=action == "seek")
            return self.snapshot()

    def _tick(self, force=False):
        now = self.clock()
        if self.playing:
            self.position = min(self.recording.duration, max(0., now - self.origin))
        index = self.index
        # At most MAX_FRAMES; a seek starts at zero and bounded latest-frame
        # selection coalesces skipped samples instead of building a queue.
        while index + 1 < len(self.recording.frames) and self.recording.frames[index + 1]["time_sec"] <= self.position:
            index += 1
        if force or index != self.index:
            self.index, self.received = index, now
            self.sequence += 1
        if self.position >= self.recording.duration:
            self.playing = False

    def snapshot(self):
        with self.lock:
            self._tick()
            return dict(schema_version=1, status="closed" if self.closed else "ready",
                binding_id=self.identifier, binding=deepcopy(self.binding),
                source=dict(id="scene:recording", label=self.recording.label, kind="recording", readonly=True),
                manifest_descriptor=dict(sha256=hashlib.sha256(self.raw).hexdigest(), byte_length=len(self.raw),
                    model_sha256=self.manifest["model_sha256"], scene_sha256=self.manifest["scene_sha256"]),
                sequence=self.sequence, pose_ready=not self.closed, worker_running=False,
                position_sec=self.position, duration_sec=self.recording.duration, playing=self.playing,
                recording_sha256=self.recording.sha256, error=None)

    def _ready(self, identifier):
        _require(not self.closed and identifier == self.identifier, "Recording binding changed")

    def manifest_bytes(self, identifier):
        with self.lock:
            self._ready(identifier)
            return self.raw

    def asset_bytes(self, digest):
        with self.lock:
            self._ready(self.identifier)
            _require(digest in self.recording.assets, "Unknown recording asset")
            return self.recording.assets[digest]

    def pose_envelope(self, identifier):
        with self.lock:
            self._ready(identifier)
            self._tick()
            frame = self.recording.frames[self.index]
            result = envelope(self.binding, frame["pose"], self.sequence,
                              source_state_age_sec=max(0., self.clock() - self.received),
                              simulation_sec=None)
            result.update(schema_version=2, source=dict(session_id=self.operator_session_id,
                clock_domain="recording:" + self.recording.sha256, stamp_sec=frame["time_sec"],
                sequence=self.index + 1, joint_positions_rad=deepcopy(frame["positions_rad"])))
            return result

    def close(self):
        with self.lock:
            self.closed, self.playing = True, False


def demo_recording():
    """Analytic two-link fixture, explicitly not a manufacturer/dynamics model."""
    definition = dict(kind="analytic-two-link", lengths_m=[1., .8], period_sec=8., sample_hz=20)
    identity = hashlib.sha256(_canonical(definition)).hexdigest()
    geoms = [dict(id=index, name=name, type="box", group=0, default_visible=True, size=size,
                  rgba=color, asset_sha256=None)
             for index, (name, size, color) in enumerate([
                 ("floor", [2., 2., .025], [.3, .35, .38, 1.]),
                 ("link1", [.5, .06, .06], [.9, .7, .2, 1.]),
                 ("link2", [.4, .05, .05], [.1, .65, .8, 1.])])]
    manifest = dict(schema_version=2, export_version=VERSION, model_sha256=identity,
        provenance=dict(adapter="recorded-analytic", source_sha256=identity, engine=None), world_frame=deepcopy(WORLD),
        units=dict(length="m", angle="rad", up_axis="Z", rotation="row-major"),
        joint_names=["joint1", "joint2"], home_rad=dict(joint1=0., joint2=0.),
        default_visible_groups=[0, 1, 2], geoms=geoms, assets=[])
    manifest["scene_sha256"] = scene_digest(manifest)
    frames = []
    def transform(index, angle, position):
        c, s = math.cos(angle), math.sin(angle)
        return dict(id=index, position=position, rotation=[c, -s, 0., s, c, 0., 0., 0., 1.])
    for index in range(161):
        stamp = index / 20
        a, b = .7 * math.sin(stamp * math.pi / 4), .9 * math.sin(stamp * math.pi / 2)
        pose = dict(schema_version=1, model_sha256=identity, scene_sha256=manifest["scene_sha256"], geoms=[
            transform(0, 0., [0., 0., -.025]),
            transform(1, a, [.5 * math.cos(a), .5 * math.sin(a), .5]),
            transform(2, a + b, [math.cos(a) + .4 * math.cos(a + b), math.sin(a) + .4 * math.sin(a + b), .5])])
        frames.append(dict(time_sec=stamp, positions_rad=dict(joint1=a, joint2=b), pose=pose))
    return Recording(dict(schema_version=1, kind="display-recording-v1", label="ROS 없는 2축 기록 예제",
                          manifest=manifest, frames=frames))
