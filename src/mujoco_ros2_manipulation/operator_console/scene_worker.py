"""One trusted model, read-only named states, and atomic CPU scene snapshots.

The parent owns the model/asset path validation, source freezing, output directory,
binding, input sequence and ROS subscriptions. This child has no ROS subscription,
GL renderer, physics integration, command publisher or model-selection endpoint.
Internal monotonic timestamps are only for its same-host parent, never browsers.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import stat
import time
import uuid

if __package__:
    from .scene_view import CompiledScene
else:
    from scene_view import CompiledScene


MAX_INPUT_BYTES = 256 * 1024
MAX_ARGUMENT_BYTES = 64 * 1024
MAX_JOINTS = 512
MAX_SAFE_INTEGER = 2**53 - 1
MAX_ABS_STATE = 1e6


class InputRejected(ValueError):
    def __init__(self, kind, message):
        super().__init__(message)
        self.kind = kind


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise InputRejected("invalid_json", "Duplicate input JSON field")
        result[key] = value
    return result


def _invalid_constant(value):
    raise InputRejected("invalid_json", "Nonfinite input JSON constant")


def _load(raw, maximum):
    if type(raw) is not bytes or len(raw) > maximum:
        raise InputRejected("oversized_input", "Input exceeds the byte limit")
    try:
        return json.loads(raw, object_pairs_hook=_pairs, parse_constant=_invalid_constant)
    except InputRejected:
        raise
    except (UnicodeDecodeError, ValueError, RecursionError) as error:
        raise InputRejected("invalid_json", "Input is not complete finite JSON") from error


def validate_joint_names(value):
    if (type(value) is not list or len(value) > MAX_JOINTS
            or any(not isinstance(name, str) or not name or len(name) > 256 for name in value)
            or len(value) != len(set(value))):
        raise ValueError("Joint names must be a bounded unique list")
    return tuple(value)


def _positions(value, joint_names):
    if type(value) is not dict or set(value) != set(joint_names):
        raise InputRejected("invalid_positions", "Positions must contain exactly the named joints")
    if any(type(q) not in (int, float) or abs(q) > MAX_ABS_STATE or not math.isfinite(q)
           for q in value.values()):
        raise InputRejected("invalid_positions", "Positions must be finite bounded numbers")
    return {name:value[name] for name in joint_names}


def decode_input(raw, joint_names, *, last_sequence=0):
    """Validate a complete state; stale or malformed input never becomes a pose."""
    if type(last_sequence) is not int or not 0 <= last_sequence <= MAX_SAFE_INTEGER:
        raise ValueError("Invalid last input sequence")
    value = _load(raw, MAX_INPUT_BYTES)
    if type(value) is not dict or set(value) != {"sequence", "positions"}:
        raise InputRejected("invalid_input", "Input requires sequence and positions only")
    sequence = value["sequence"]
    if type(sequence) is not int or not 1 <= sequence <= MAX_SAFE_INTEGER:
        raise InputRejected("invalid_sequence", "Input sequence must be a positive safe integer")
    positions = _positions(value["positions"], joint_names)
    if sequence <= last_sequence:
        raise InputRejected("old_sequence", "Input sequence did not advance")
    return dict(sequence=sequence, positions=positions)


def _read_regular(path, maximum):
    # NOFOLLOW also prevents a FIFO/device from blocking the read-only worker.
    flags = os.O_RDONLY | os.O_NONBLOCK | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise InputRejected("invalid_input_file", "Input must be a regular local file")
        if metadata.st_size > maximum:
            raise InputRejected("oversized_input", "Input exceeds the byte limit")
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            raw = stream.read(maximum+1)
        if len(raw) > maximum:
            raise InputRejected("oversized_input", "Input exceeds the byte limit")
        return raw
    finally:
        os.close(descriptor)


def _atomic_bytes(path, data):
    temporary = path.with_name("."+path.name+"."+uuid.uuid4().hex+".tmp")
    try:
        with temporary.open("xb") as stream:
            stream.write(data)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path, value):
    encoded = (json.dumps(value, sort_keys=True, separators=(",", ":"),
                          ensure_ascii=True, allow_nan=False)+"\n").encode("ascii")
    _atomic_bytes(path, encoded)


def publish_manifest(scene, directory):
    """Publish complete owned mesh bytes first and its manifest last."""
    directory = Path(directory)
    assets = directory / "assets"
    if assets.is_symlink():
        raise ValueError("Scene asset directory must not be a symlink")
    assets.mkdir(exist_ok=True)
    if not assets.is_dir():
        raise ValueError("Scene asset directory is unavailable")
    manifest = scene.manifest()
    if type(manifest) is not dict or manifest.get("schema_version") != 1 or type(manifest.get("assets")) is not list:
        raise ValueError("Invalid compiled scene manifest")
    seen = set()
    for asset in manifest["assets"]:
        digest = asset.get("sha256") if type(asset) is dict else None
        if (not isinstance(digest, str) or len(digest) != 64
                or any(c not in "0123456789abcdef" for c in digest) or digest in seen):
            raise ValueError("Invalid compiled asset identity")
        seen.add(digest)
        blob = scene.asset(digest)
        if (type(blob) is not bytes or type(asset.get("byte_length")) is not int
                or len(blob) != asset["byte_length"] or hashlib.sha256(blob).hexdigest() != digest):
            raise ValueError("Compiled mesh does not match its descriptor")
        _atomic_bytes(assets/(digest+".bin"), blob)
    _atomic_json(directory/"manifest.json", manifest)


class SceneWorker:
    """Small file-protocol loop, with the compiled scene injected for unit tests."""

    def __init__(self, scene, directory, joint_names, *, clock=time.monotonic):
        self.scene = scene
        self.directory = Path(directory)
        self.joint_names = validate_joint_names(list(joint_names))
        self.clock = clock
        self.last_sequence = 0
        self._last_raw_digest = None
        self._last_error = None

    def _error(self, kind, error):
        # Full local exception messages may contain private paths. This file is
        # diagnostic-only and must not be passed through the browser API.
        value = dict(schema_version=1, kind=kind,
                     error=dict(type=type(error).__name__, message=str(error)))
        if value != self._last_error:
            _atomic_json(self.directory/"error.json", value)
            self._last_error = value

    def step(self):
        """Return true only after a newly accepted complete pose was published."""
        try:
            raw = _read_regular(self.directory/"input.json", MAX_INPUT_BYTES)
        except FileNotFoundError:
            return False  # Waiting for the parent's first/current atomic input.
        except (InputRejected, OSError) as error:
            self._error(getattr(error, "kind", "input_read_failed"), error)
            return False
        digest = hashlib.sha256(raw).hexdigest()
        if digest == self._last_raw_digest:
            return False
        self._last_raw_digest = digest
        try:
            record = decode_input(raw, self.joint_names, last_sequence=self.last_sequence)
        except InputRejected as error:
            self._error(error.kind, error)
            return False
        try:
            pose = self.scene.pose(record["positions"])
            processed = self.clock()
            if type(processed) not in (int, float) or not math.isfinite(processed) or processed < 0:
                raise ValueError("Invalid worker monotonic clock")
            _atomic_json(self.directory/"pose.json", dict(sequence=record["sequence"],
                         pose=pose, processed_monotonic=processed))
        except Exception as error:
            self._error("pose_failed", error)
            return False
        self.last_sequence = record["sequence"]
        (self.directory/"error.json").unlink(missing_ok=True)
        self._last_error = None
        return True


def parent_present(expected_pid, *, getppid=os.getppid):
    """Reparenting signals exit; PID reuse cannot restore a child's parent."""
    return type(expected_pid) is int and expected_pid > 1 and getppid() == expected_pid


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--joint-names-json", required=True)
    parser.add_argument("--home-json", required=True)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--parent-pid", type=int, required=True)
    args = parser.parse_args(argv)
    # The trusted parent creates a fresh owned directory. Do not accept relative
    # paths or follow a final output-directory symlink by accident.
    if (not args.model.is_absolute() or not args.directory.is_absolute()
            or args.directory.is_symlink() or not args.directory.is_dir()
            or not parent_present(args.parent_pid)):
        parser.error("Model/output must be trusted absolute paths with a live direct parent")
    stopped = [False]
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.__setitem__(0, True))
    try:
        names = validate_joint_names(_load(args.joint_names_json.encode("utf-8"), MAX_ARGUMENT_BYTES))
        home = _positions(_load(args.home_json.encode("utf-8"), MAX_ARGUMENT_BYTES), names)
        # A restart cannot present a pose produced by an earlier child.
        for name in ("pose.json", "error.json", "manifest.json"):
            (args.directory/name).unlink(missing_ok=True)
        scene = CompiledScene(args.model, list(names), home)
        if stopped[0] or not parent_present(args.parent_pid):
            return 0
        publish_manifest(scene, args.directory)
        worker = SceneWorker(scene, args.directory, names)
        while not stopped[0] and parent_present(args.parent_pid):
            worker.step()
            time.sleep(.05)
        return 0
    except Exception as error:
        _atomic_json(args.directory/"error.json", dict(schema_version=1, kind="startup_or_io_failed",
                     error=dict(type=type(error).__name__, message=str(error))))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
