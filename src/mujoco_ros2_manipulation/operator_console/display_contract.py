"""Engine-independent, versioned display data. No ROS, NumPy or engine imports.

Geometry sizes use half extents for boxes, radius/half length for cylinders and
capsules, radius for spheres; compiled mesh vertices are already in local metres.
Transforms are complete world positions and right-handed row-major rotations.
These are visual records, never a command or collision-admission interface.
"""
from copy import deepcopy
import hashlib
import json
import math
import re
from types import SimpleNamespace

from .scene_host import _manifest, _canonical, _require, _json
from .view_binding import envelope, _checked_pose
from .object_hierarchy import from_compiled_hierarchy


VERSION = "display-scene-v1"
WORLD = dict(name="world", handedness="right", primitive_size="half-extents-radius-half-length")


def scene_digest(value):
    return hashlib.sha256(_canonical({key: item for key, item in value.items()
                                     if key != "scene_sha256"})).hexdigest()


def validate_manifest(value):
    """Validate bounded common data with producer details confined to provenance."""
    value = deepcopy(value)
    _require(type(value) is dict and set(value) == {
        "schema_version", "export_version", "model_sha256", "scene_sha256", "provenance",
        "world_frame", "units", "joint_names", "home_rad", "default_visible_groups", "geoms", "assets"},
        "Invalid display manifest fields")
    _require(type(value["schema_version"]) is int and value["schema_version"] == 2
             and value["export_version"] == VERSION and value["world_frame"] == WORLD,
             "Unsupported display conventions")
    provenance = value["provenance"]
    _require(type(provenance) is dict and set(provenance) == {"adapter", "source_sha256", "engine"}
             and isinstance(provenance["adapter"], str)
             and re.fullmatch(r"[a-z][a-z0-9_-]{0,63}", provenance["adapter"])
             and isinstance(provenance["source_sha256"], str)
             and re.fullmatch(r"[0-9a-f]{64}", provenance["source_sha256"]), "Invalid display provenance")
    engine = provenance["engine"]
    _require(engine is None or (type(engine) is dict and set(engine) == {"name", "version", "compiled_sha256"}
             and isinstance(engine["name"], str) and 0 < len(engine["name"]) <= 64
             and isinstance(engine["version"], str) and 0 < len(engine["version"]) <= 64
             and isinstance(engine["compiled_sha256"], str)
             and re.fullmatch(r"[0-9a-f]{64}", engine["compiled_sha256"])), "Invalid engine provenance")
    _require(scene_digest(value) == value["scene_sha256"], "Display scene digest mismatch")
    # Reuse the engine-free geometry/buffer bounds validator. The compatibility
    # fields below exist only inside this validator, never in the common format.
    legacy = {key: deepcopy(item) for key, item in value.items()
              if key not in {"provenance", "world_frame", "scene_sha256"}}
    legacy.update(schema_version=1, export_version="compiled-scene-v1", mujoco_version="0.0.0",
                  compiled_model_sha256=value["model_sha256"])
    legacy["scene_sha256"] = scene_digest(legacy)
    source = SimpleNamespace(joint_names=tuple(value["joint_names"]), home=value["home_rad"])
    _manifest(_canonical(legacy), source, value["model_sha256"])
    _require(value["geoms"], "Display scene must contain geometry")
    _require(len(value["assets"]) <= len(value["geoms"]), "Display asset catalog exceeds geometry count")
    for asset in value["assets"]:
        for array in asset["layout"].values():
            _require(all(type(array[field]) is int for field in ("byte_offset", "byte_length", "count", "item_size")),
                     "Display array layout must contain exact integers")
    names, home = value["joint_names"], value["home_rad"]
    _require(type(names) is list and len(names) <= 512
             and all(isinstance(name, str) and 0 < len(name) <= 256 for name in names) and len(names) == len(set(names))
             and type(home) is dict and set(home) == set(names), "Invalid display joint correspondence")
    for number in home.values():
        _require(type(number) in (int, float) and math.isfinite(number) and abs(number) <= 1e6, "Invalid display home position")
    for geom in value["geoms"]:
        kind, size = geom["type"], geom["size"]
        if kind not in ("mesh", "plane"):
            _require(size[0] > 0, "Degenerate display primitive radius/extent")
            if kind == "box":
                _require(all(number > 0 for number in size), "Degenerate display box")
            elif kind == "cylinder":
                _require(size[1] > 0, "Degenerate display cylinder")
    return value


def from_compiled_manifest(value):
    """Explicit compatibility adapter; retain engine provenance, not engine fields."""
    source = SimpleNamespace(joint_names=tuple(value["joint_names"]), home=value["home_rad"])
    _manifest(_canonical(value), source, value["model_sha256"])
    result = {key: deepcopy(item) for key, item in value.items()
              if key not in {"scene_sha256", "compiled_model_sha256", "mujoco_version"}}
    result.update(schema_version=2, export_version=VERSION, world_frame=deepcopy(WORLD),
        provenance=dict(adapter="mujoco-compiled", source_sha256=value["scene_sha256"], engine=dict(
            name="mujoco", version=value["mujoco_version"], compiled_sha256=value["compiled_model_sha256"])))
    result["scene_sha256"] = scene_digest(result)
    return validate_manifest(result)


def validate_pose(pose, manifest):
    pose = _checked_pose(pose)
    _require(pose["model_sha256"] == manifest["model_sha256"]
             and pose["scene_sha256"] == manifest["scene_sha256"]
             and len(pose["geoms"]) == len(manifest["geoms"]), "Display pose model/catalog mismatch")
    return pose


def from_compiled_pose(pose, manifest):
    """Change only the declared scene digest after matching the source digest."""
    pose = _checked_pose(pose)
    _require(manifest["provenance"]["adapter"] == "mujoco-compiled"
             and pose["scene_sha256"] == manifest["provenance"]["source_sha256"],
             "Pose belongs to another compiled scene")
    pose["scene_sha256"] = manifest["scene_sha256"]
    return validate_pose(pose, manifest)


class DisplayHostAdapter:
    """Adapt existing MuJoCo producer to the same common format as recordings.

    Worker ownership, admission and stale ages stay with the existing owner.
    A source switch clears its cache before a new manifest/pose is observable.
    """
    def __init__(self, host):
        self.host = host
        self._key = self._manifest = self._raw = None
        self._hierarchy_key = self._hierarchy_raw = None

    def __getattr__(self, name):
        return getattr(self.host, name)

    def _current(self, identifier):
        raw = self.host.manifest_bytes(identifier)
        key = (identifier, hashlib.sha256(raw).hexdigest())
        if key != self._key:
            value = from_compiled_manifest(_json(raw))
            self._manifest, self._raw, self._key = value, _canonical(value) + b"\n", key
            self._hierarchy_key = self._hierarchy_raw = None
        return self._manifest, self._raw

    def snapshot(self):
        with self.host.lock:
            state = self.host.snapshot()
            if state["status"] == "ready" and state.get("manifest_descriptor"):
                manifest, raw = self._current(state["binding_id"])
                state["manifest_descriptor"] = dict(sha256=hashlib.sha256(raw).hexdigest(), byte_length=len(raw),
                    model_sha256=manifest["model_sha256"], scene_sha256=manifest["scene_sha256"],
                    compiled_model_sha256=manifest["provenance"]["engine"]["compiled_sha256"],
                    geoms=len(manifest["geoms"]), assets=len(manifest["assets"]))
                if state.get("hierarchy_available"):
                    hierarchy_raw = self.hierarchy_bytes(state["binding_id"])
                    hierarchy = _json(hierarchy_raw)
                    state["hierarchy_descriptor"] = dict(sha256=hashlib.sha256(hierarchy_raw).hexdigest(),
                        byte_length=len(hierarchy_raw), scene_sha256=manifest["scene_sha256"],
                        bodies=hierarchy["counts"]["bodies"], geoms=hierarchy["counts"]["geoms"])
            return state

    def manifest_bytes(self, identifier):
        with self.host.lock:
            return self._current(identifier)[1]

    def pose_envelope(self, identifier):
        with self.host.lock:
            value = self.host.pose_envelope(identifier)
            if value is None:
                return None
            manifest, _ = self._current(identifier)
            value["pose"] = from_compiled_pose(value["pose"], manifest)
            return value

    def hierarchy_bytes(self, identifier):
        """Map exact geometry/body IDs to the adapted current display digest."""
        with self.host.lock:
            manifest, _ = self._current(identifier)  # Also rejects an old binding.
            read = getattr(self.host, "hierarchy_bytes", None)
            raw = None if read is None else read(identifier)
            if raw is None:
                return None
            key = (self._key, raw)
            if key != self._hierarchy_key:
                hierarchy = from_compiled_hierarchy(_json(raw), manifest)
                self._hierarchy_raw, self._hierarchy_key = _canonical(hierarchy) + b"\n", key
            return self._hierarchy_raw
