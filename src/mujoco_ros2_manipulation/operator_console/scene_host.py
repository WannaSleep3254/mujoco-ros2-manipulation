"""Own one read-only scene worker, with no ROS or MuJoCo imports in this process.

Only trusted application code constructs SceneSource and selects a binding. This
class never discovers a model, rebuilds generated files, admits robot commands or
decides whether an observed ROS graph belongs to the caller. It freezes reviewed
worker output and retains the ORIGINAL receiver timestamp for every input seq.
"""
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import threading
import time
from typing import Callable
import uuid

if __package__:
    from .view_binding import binding_id as _binding_id, envelope
else:  # Detached, CPU-free draft tests; package integration uses the branch above.
    from view_binding import binding_id as _binding_id, envelope


MAX_SAFE_INTEGER = 2**53 - 1
MAX_MANIFEST_BYTES = 8 * 1024 * 1024
MAX_POSE_BYTES = 8 * 1024 * 1024
MAX_INPUT_BYTES = 256 * 1024
MAX_ARGUMENT_BYTES = 64 * 1024
MAX_XML_BYTES = 16 * 1024 * 1024
MAX_ASSET_BYTES = 48 * 1024 * 1024
MAX_TOTAL_ASSET_BYTES = 128 * 1024 * 1024
SHA256 = re.compile(r"[0-9a-f]{64}")


def _require(value, message):
    if not value:
        raise ValueError(message)


def _number(value, label, *, maximum=1e9, nonnegative=False):
    _require(type(value) in (int, float) and abs(value) <= maximum and math.isfinite(value)
             and (not nonnegative or value >= 0), "Invalid " + label)
    return value


def _state(value, names):
    _require(type(value) is dict and set(value) == set(names), "State must cover exactly the named joints")
    return {name: _number(value[name], "joint position", maximum=1e6) for name in names}


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def _pairs(items):
    result = {}
    for key, value in items:
        _require(key not in result, "Duplicate JSON field")
        result[key] = value
    return result


def _constant(value):
    raise ValueError("Nonfinite JSON constant")


def _json(raw):
    return json.loads(raw, object_pairs_hook=_pairs, parse_constant=_constant)


def _regular(path, maximum):
    """Bound a regular file without following its final link or opening a FIFO."""
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0))
    try:
        info = os.fstat(descriptor)
        _require(stat.S_ISREG(info.st_mode) and info.st_size <= maximum, "Invalid bounded worker file")
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            raw = stream.read(maximum + 1)
        _require(len(raw) <= maximum, "Worker file exceeds byte limit")
        return raw
    finally:
        os.close(descriptor)


def _atomic_json(path, value):
    raw = _canonical(value) + b"\n"
    _require(len(raw) <= MAX_INPUT_BYTES, "Scene input exceeds byte limit")
    temporary = path.with_name("." + path.name + "." + uuid.uuid4().hex)
    try:
        with temporary.open("xb") as stream:
            stream.write(raw)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


@dataclass(frozen=True)
class SceneSource:
    """Trusted registry entry; validate() checks existing files and never builds.

    A returned JSON metadata/token is compared before/after loading and on later
    source checks. Returning None is allowed when validation itself rejects every
    input/output change. Neither this value nor xml_path is exposed in snapshots.
    """

    id: str
    label: str
    xml_path: Path
    joint_names: tuple
    home: dict
    validate: Callable[[], object]

    def __post_init__(self):
        _require(isinstance(self.id, str) and len(self.id) <= 128 and
                 re.fullmatch(r"(?:(?:workcell|profile|scene):)?[a-z][a-z0-9_-]*", self.id), "Invalid trusted source ID")
        _require(isinstance(self.label, str) and 0 < len(self.label) <= 256, "Invalid trusted source label")
        path = Path(self.xml_path)
        _require(path.is_absolute(), "Trusted model path must be absolute")
        names = tuple(self.joint_names)
        _require(len(names) <= 512 and all(isinstance(name, str) and 0 < len(name) <= 256 for name in names)
                 and len(set(names)) == len(names), "Invalid source joint names")
        home = _state(self.home, names)
        _require(callable(self.validate), "Trusted source requires a validator")
        _require(len(_canonical(list(names))) <= MAX_ARGUMENT_BYTES and len(_canonical(home)) <= MAX_ARGUMENT_BYTES,
                 "Source joint contract exceeds worker argument limit")
        object.__setattr__(self, "xml_path", path)
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "home", deepcopy(home))


def _manifest(raw, source, model_sha256):
    data = _json(raw)
    if type(data) is dict and data.get("schema_version") == 2:
        from .display_contract import validate_manifest
        data = validate_manifest(data)
        _require(data["model_sha256"] == model_sha256 and data["joint_names"] == list(source.joint_names)
                 and data["home_rad"] == source.home, "Display source model/joint mismatch")
        return data, {item["sha256"]: item for item in data["assets"]}
    fields = {"schema_version", "export_version", "scene_sha256", "model_sha256", "compiled_model_sha256",
              "mujoco_version", "units", "joint_names", "home_rad", "default_visible_groups", "geoms", "assets"}
    _require(type(data) is dict and set(data) == fields and type(data["schema_version"]) is int
             and data["schema_version"] == 1 and data["export_version"] == "compiled-scene-v1", "Invalid worker manifest")
    for name in ("scene_sha256", "model_sha256", "compiled_model_sha256"):
        _require(isinstance(data[name], str) and SHA256.fullmatch(data[name]), "Invalid compiled scene digest")
    _require(data["model_sha256"] == model_sha256 and data["joint_names"] == list(source.joint_names)
             and data["home_rad"] == source.home, "Worker compiled another model/joint contract")
    without_digest = {key: value for key, value in data.items() if key != "scene_sha256"}
    _require(hashlib.sha256(_canonical(without_digest)).hexdigest() == data["scene_sha256"], "Compiled scene digest mismatch")
    _require(data["units"] == dict(length="m", angle="rad", up_axis="Z", rotation="row-major")
             and data["default_visible_groups"] == [0, 1, 2]
             and isinstance(data["mujoco_version"], str) and re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+(?:[a-zA-Z0-9.+-]*)", data["mujoco_version"]),
             "Invalid scene units or engine version")
    geoms, assets = data["geoms"], data["assets"]
    _require(type(geoms) is list and len(geoms) <= 10000 and type(assets) is list and len(assets) <= 1024,
             "Worker scene exceeds catalog limits")
    catalog = {}
    total = 0
    for asset in assets:
        _require(type(asset) is dict and set(asset) == {"sha256", "byte_length", "media_type", "layout"}, "Invalid asset descriptor")
        digest, length = asset["sha256"], asset["byte_length"]
        _require(isinstance(digest, str) and SHA256.fullmatch(digest) and digest not in catalog
                 and type(length) is int and 0 < length <= MAX_ASSET_BYTES
                 and asset["media_type"] == "application/octet-stream", "Invalid compiled asset")
        layout = asset["layout"]
        _require(type(layout) is dict and set(layout) == {"positions", "indices"}, "Invalid compiled layout")
        p, i = layout["positions"], layout["indices"]
        for item in (p, i):
            _require(type(item) is dict and set(item) == {"byte_offset", "byte_length", "count", "item_size", "component_type"}, "Invalid compiled array")
        _require(type(p["count"]) is int and 3 <= p["count"] <= 2000000 and p["item_size"] == 3
                 and p["component_type"] == "float32" and p["byte_offset"] == 0 and p["byte_length"] == p["count"] * 12
                 and type(i["count"]) is int and 3 <= i["count"] <= 6000000 and i["count"] % 3 == 0 and i["item_size"] == 1
                 and i["component_type"] == "uint32" and i["byte_offset"] == p["byte_length"]
                 and i["byte_length"] == i["count"] * 4 and length == p["byte_length"] + i["byte_length"], "Invalid compiled array ranges")
        total += length
        catalog[digest] = asset
    _require(total <= MAX_TOTAL_ASSET_BYTES, "Scene assets exceed byte budget")
    for index, geom in enumerate(geoms):
        _require(type(geom) is dict and set(geom) == {"id", "name", "type", "group", "default_visible", "size", "rgba", "asset_sha256"}
                 and type(geom["id"]) is int and geom["id"] == index and isinstance(geom["name"], str)
                 and geom["type"] in {"box", "sphere", "cylinder", "capsule", "plane", "mesh"}
                 and type(geom["group"]) is int and 0 <= geom["group"] <= 5
                 and type(geom["default_visible"]) is bool and geom["default_visible"] == (geom["group"] in (0, 1, 2)), "Invalid compiled geometry")
        for field, count, maximum in (("size", 3, 1e9), ("rgba", 4, 1)):
            _require(type(geom[field]) is list and len(geom[field]) == count, "Invalid geom vector")
            for value in geom[field]:
                _number(value, "geom value", maximum=maximum, nonnegative=True)
        _require((geom["type"] == "mesh" and geom["asset_sha256"] in catalog)
                 or (geom["type"] != "mesh" and geom["asset_sha256"] is None), "Invalid geom asset reference")
    return data, catalog


class SceneHost:
    """Asynchronous single-child supervisor; every binding switch drops caches first."""

    def __init__(self, directory, worker_python, *, worker_module="mujoco_ros2_manipulation.operator_console.scene_worker",
                 clock=time.monotonic, process_factory=subprocess.Popen, start_timeout_sec=30,
                 validation_interval_sec=1, max_pending=64, start_thread=True, cwd=None):
        self.directory = Path(directory).absolute()
        _require(not self.directory.exists(), "Scene host directory must be new")
        self.directory.mkdir(parents=True, mode=0o700)
        self.worker_python = Path(worker_python).absolute()
        _require(isinstance(worker_module, str) and re.fullmatch(r"[a-zA-Z_]\w*(?:\.[a-zA-Z_]\w*)*", worker_module), "Invalid trusted worker module")
        _number(start_timeout_sec, "startup timeout", maximum=120, nonnegative=True)
        _number(validation_interval_sec, "source check interval", maximum=60, nonnegative=True)
        _require(start_timeout_sec > 0 and validation_interval_sec > 0 and type(max_pending) is int and 2 <= max_pending <= 256,
                 "Invalid scene supervisor limits")
        self.worker_module, self.clock, self.process_factory = worker_module, clock, process_factory
        self.start_timeout_sec, self.validation_interval_sec, self.max_pending = start_timeout_sec, validation_interval_sec, max_pending
        self.cwd = None if cwd is None else Path(cwd)
        self.lock, self.step_lock = threading.RLock(), threading.Lock()
        self.wake = threading.Event()
        self.closed = False
        self.revision = 0
        self.attempted_revision = -1
        self.binding = self.binding_id = self.source = None
        self.status, self.error = "idle", None
        self._manifest = self._manifest_raw = self._manifest_descriptor = None
        self._assets = {}
        self._pose = None
        self._samples = OrderedDict()
        self._sequence = self._published_sequence = 0
        self._pending_input = None
        self._last_submission = None
        self._epoch = None
        self._thread = threading.Thread(target=self._run, name="scene-host", daemon=True) if start_thread else None
        if self._thread is not None:
            self._thread.start()

    def _clear(self):
        self._manifest = self._manifest_raw = self._manifest_descriptor = self._pose = None
        self._assets = {}
        self._samples.clear()
        self._sequence = self._published_sequence = 0
        self._pending_input = self._last_submission = None

    def activate(self, binding, source):
        identifier = _binding_id(binding)
        _require(isinstance(source, SceneSource) and binding["source_id"] == source.id, "Binding/source mismatch")
        with self.lock:
            _require(not self.closed, "Scene host is closed")
            if identifier == self.binding_id and self.source is not None:
                _require(self.source.id == source.id and self.source.xml_path == source.xml_path
                         and self.source.joint_names == source.joint_names and self.source.home == source.home,
                         "A new source requires a new view generation")
                return identifier
            self.revision += 1
            self.binding, self.binding_id, self.source = deepcopy(binding), identifier, source
            self.status, self.error = "preparing", None
            self._clear()  # Includes all earlier bindings' pose/asset data immediately.
            if binding["mode"] == "model_preview":
                self.submit(source.home)
            self.wake.set()
        return identifier

    def invalidate(self, binding=None):
        """Blank a transition immediately, without selecting/compiling any source.

        The caller supplies the new binding generation when one is known. The
        host independently advances its internal revision to reject old output.
        No binding mutation or synthetic live/home pose is invented here.
        """
        identifier = None if binding is None else _binding_id(binding)
        with self.lock:
            _require(not self.closed, "Scene host is closed")
            self.revision += 1
            self.binding = None if binding is None else deepcopy(binding)
            self.binding_id, self.source = identifier, None
            self.status, self.error = "waiting", None
            self._clear()
            self.wake.set()
        return identifier

    def submit(self, positions, source_received_monotonic=None, simulation_sec=None, preview_time_sec=None):
        with self.lock:
            _require(not self.closed and self.binding is not None and self.status in ("preparing", "ready"), "No active scene source")
            state = _state(positions, self.source.joint_names)
            now = self.clock()
            mode = self.binding["mode"]
            if mode == "live":
                _number(source_received_monotonic, "original state receiver time", nonnegative=True)
                _require(source_received_monotonic <= now, "State receiver time is in the future")
            else:
                _require(source_received_monotonic is None, "Preview state has no live receiver time")
            if simulation_sec is not None:
                _number(simulation_sec, "simulation stamp", nonnegative=True)
            if mode == "plan_preview":
                _number(preview_time_sec, "preview time", nonnegative=True)
            else:
                _require(preview_time_sec is None, "Only plan preview accepts playback time")
            sample = dict(source_received_monotonic=source_received_monotonic,
                          simulation_sec=simulation_sec, preview_time_sec=preview_time_sec)
            current = (state, sample)
            if self._last_submission == current:
                return self._sequence
            if mode == "live" and self._last_submission is not None:
                previous_time = self._last_submission[1]["source_received_monotonic"]
                _require(source_received_monotonic > previous_time, "Live sample did not advance its original receive time")
            _require(self._sequence < MAX_SAFE_INTEGER, "Scene sequence exhausted")
            sequence = self._sequence + 1
            payload = dict(sequence=sequence, positions=state)
            _require(len(_canonical(payload)) + 1 <= MAX_INPUT_BYTES, "Scene input exceeds byte limit")
            self._sequence = sequence
            self._last_submission = deepcopy(current)
            self._samples[sequence] = sample
            while len(self._samples) > self.max_pending:
                self._samples.popitem(last=False)
            self._pending_input = payload  # Coalesce; do not build a state backlog.
            self.wake.set()
            return sequence

    def snapshot(self):
        with self.lock:
            process = None if self._epoch is None else self._epoch.get("process")
            return deepcopy(dict(schema_version=1, status=self.status, binding_id=self.binding_id, binding=self.binding,
                source=None if self.source is None else dict(id=self.source.id, label=self.source.label),
                manifest_descriptor=self._manifest_descriptor, sequence=self._published_sequence,
                pose_ready=self._pose is not None, worker_running=process is not None and process.poll() is None,
                error=self.error))

    def _ready(self, identifier):
        _require(not self.closed and identifier == self.binding_id and self.status == "ready", "Current scene is not ready")

    def manifest_bytes(self, binding_id):
        with self.lock:
            self._ready(binding_id)
            return self._manifest_raw

    def asset_bytes(self, sha256, binding_id=None):
        _require(isinstance(sha256, str) and SHA256.fullmatch(sha256), "Invalid scene asset digest")
        with self.lock:
            self._ready(self.binding_id if binding_id is None else binding_id)
            _require(sha256 in self._assets, "Asset is not in the current scene catalog")
            return self._assets[sha256]

    def pose_envelope(self, binding_id):
        with self.lock:
            self._ready(binding_id)
            if self._pose is None:
                return None
            sequence, pose, sample = self._pose
            age = None if self.binding["mode"] != "live" else max(0., self.clock() - sample["source_received_monotonic"])
            return envelope(self.binding, pose, sequence, source_state_age_sec=age,
                            simulation_sec=sample["simulation_sec"], preview_time_sec=sample["preview_time_sec"])

    def _fail(self, revision, code):
        # Never forward local exception strings, CLI paths or worker error.json.
        with self.lock:
            if revision == self.revision and not self.closed:
                self.attempted_revision = revision  # A failed revision is never silently retried.
                self.status = "error"
                self.error = dict(code=code, message="Read-only scene data is unavailable.")
                self._clear()

    def _validated_source(self, source):
        token = deepcopy(source.validate())
        if token is not None:
            _canonical(token)  # Validator metadata must be a finite detached JSON value.
        digest = hashlib.sha256(_regular(source.xml_path, MAX_XML_BYTES)).hexdigest()
        return token, digest

    def _launch(self, revision, source):
        try:
            validation, digest = self._validated_source(source)
        except Exception:
            self._fail(revision, "source_validation_failed")
            return
        folder = self.directory / ("worker-" + uuid.uuid4().hex)
        log = None
        try:
            with self.lock:
                if self.closed or revision != self.revision:
                    return
                folder.mkdir(mode=0o700)
                log = (folder / "worker.log").open("wb")
                argv = [str(self.worker_python), "-m", self.worker_module, "--model", str(source.xml_path),
                    "--joint-names-json", _canonical(list(source.joint_names)).decode(),
                    "--home-json", _canonical(source.home).decode(), "--directory", str(folder),
                    "--parent-pid", str(os.getpid())]
                process = self.process_factory(argv, cwd=self.cwd, stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                now = self.clock()
                self._epoch = dict(revision=revision, source=source, folder=folder, process=process, log=log,
                    validation=validation, model_sha256=digest, started=now, checked=now, manifest_ready=False,
                    last_written_sequence=0, last_output_digest=None)
        except Exception:
            if log is not None:
                log.close()
            shutil.rmtree(folder, ignore_errors=True)
            self._fail(revision, "worker_start_failed")

    def _stop_epoch(self):
        epoch = self._epoch
        if epoch is None:
            return True
        process = epoch["process"]
        try:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
        except Exception:
            if process.poll() is None:
                return False  # Never create a second child while the first is alive.
        epoch["log"].close()
        shutil.rmtree(epoch["folder"], ignore_errors=True)
        with self.lock:
            if self._epoch is epoch:
                self._epoch = None
        return True

    def _prepare(self, epoch):
        path = epoch["folder"] / "manifest.json"
        try:
            raw = _regular(path, MAX_MANIFEST_BYTES)
        except FileNotFoundError:
            if self.clock() - epoch["started"] > self.start_timeout_sec:
                raise RuntimeError("Worker manifest readiness timeout")
            return
        source = epoch["source"]
        data, catalog = _manifest(raw, source, epoch["model_sha256"])
        assets = {}
        asset_directory = epoch["folder"] / "assets"
        _require(not asset_directory.is_symlink() and asset_directory.is_dir(), "Invalid owned asset directory")
        for digest, descriptor in catalog.items():
            blob = _regular(asset_directory / (digest + ".bin"), MAX_ASSET_BYTES)
            _require(len(blob) == descriptor["byte_length"] and hashlib.sha256(blob).hexdigest() == digest,
                     "Compiled asset digest mismatch")
            assets[digest] = blob
        validation, digest = self._validated_source(source)
        _require(validation == epoch["validation"] and digest == epoch["model_sha256"], "Source changed during compilation")
        with self.lock:
            if self.closed or epoch["revision"] != self.revision:
                return
            self._manifest, self._manifest_raw, self._assets = data, raw, assets
            self._manifest_descriptor = dict(sha256=hashlib.sha256(raw).hexdigest(), byte_length=len(raw),
                model_sha256=data["model_sha256"], scene_sha256=data["scene_sha256"], compiled_model_sha256=data["compiled_model_sha256"],
                geoms=len(data["geoms"]), assets=len(catalog))
            epoch["manifest_ready"] = True
            epoch["checked"] = self.clock()
            self.status = "ready"

    def _refresh(self, epoch):
        revision = epoch["revision"]
        if epoch["process"].poll() is not None:
            self._fail(revision, "worker_exited")
            return
        if (epoch["folder"] / "error.json").exists():
            # The diagnostic file is intentionally local-only, including its paths.
            self._fail(revision, "worker_reported_error")
            return
        try:
            if not epoch["manifest_ready"]:
                self._prepare(epoch)
            if not epoch["manifest_ready"]:
                return
            if self.clock() - epoch["checked"] >= self.validation_interval_sec:
                validation, digest = self._validated_source(epoch["source"])
                _require(validation == epoch["validation"] and digest == epoch["model_sha256"], "Source changed after preparation")
                epoch["checked"] = self.clock()
        except Exception:
            self._fail(revision, "source_or_manifest_changed")
            return
        with self.lock:
            if self.closed or revision != self.revision:
                return
            pending = deepcopy(self._pending_input)
        if pending is not None and pending["sequence"] > epoch["last_written_sequence"]:
            _atomic_json(epoch["folder"] / "input.json", pending)
            epoch["last_written_sequence"] = pending["sequence"]
        try:
            raw = _regular(epoch["folder"] / "pose.json", MAX_POSE_BYTES)
        except FileNotFoundError:
            return
        digest = hashlib.sha256(raw).hexdigest()
        if digest == epoch["last_output_digest"]:
            return
        record = _json(raw)
        _require(type(record) is dict and set(record) == {"sequence", "pose", "processed_monotonic"}, "Invalid worker pose record")
        sequence = record["sequence"]
        _require(type(sequence) is int and 1 <= sequence <= MAX_SAFE_INTEGER, "Invalid worker sequence")
        _number(record["processed_monotonic"], "worker completion time", nonnegative=True)
        _require(epoch["started"] <= record["processed_monotonic"] <= self.clock(), "Worker completion time is not same-host monotonic")
        with self.lock:
            if self.closed or revision != self.revision:
                return
            epoch["last_output_digest"] = digest
            _require(sequence <= self._sequence, "Worker sequence was never submitted")
            sample = self._samples.get(sequence)
            if sample is None or sequence <= self._published_sequence:
                return  # Eviction, replay or a superseded state never invents a new age.
            _require(sequence <= epoch["last_written_sequence"], "Worker sequence was not submitted")
            checked = envelope(self.binding, record["pose"], sequence,
                source_state_age_sec=(self.clock() - sample["source_received_monotonic"]) if self.binding["mode"] == "live" else None,
                simulation_sec=sample["simulation_sec"], preview_time_sec=sample["preview_time_sec"])
            pose = checked["pose"]
            _require(pose["scene_sha256"] == self._manifest["scene_sha256"] and pose["model_sha256"] == self._manifest["model_sha256"]
                     and len(pose["geoms"]) == len(self._manifest["geoms"]), "Worker pose compiled identity mismatch")
            self._pose = (sequence, pose, deepcopy(sample))
            self._published_sequence = sequence

    def _step(self):
        """One serialized supervisor step; fake-process tests call this without a thread."""
        with self.step_lock:
            with self.lock:
                revision, source, closed = self.revision, self.source, self.closed
            if self._epoch is not None and (closed or self._epoch["revision"] != revision or self.status == "error"):
                if not self._stop_epoch():
                    self._fail(revision, "worker_cleanup_failed")
                    return
            if closed:
                return
            if source is not None and self.attempted_revision != revision:
                self.attempted_revision = revision
                self._launch(revision, source)
            epoch = self._epoch
            if epoch is not None and epoch["revision"] == revision:
                try:
                    self._refresh(epoch)
                except Exception:
                    self._fail(revision, "invalid_worker_output")

    def _run(self):
        try:
            while True:
                self._step()
                with self.lock:
                    if self.closed:
                        return
                self.wake.wait(.05)
                self.wake.clear()
        finally:
            self._stop_epoch()

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            self.status, self.error = "closed", None
            self.revision += 1
            self._clear()
            self.wake.set()
        if self._thread is not None:
            self._thread.join(timeout=6)
            if self._thread.is_alive():
                raise RuntimeError("Scene worker supervisor did not stop")
        else:
            self._step()
        _require(self._epoch is None, "Scene worker is still alive")
