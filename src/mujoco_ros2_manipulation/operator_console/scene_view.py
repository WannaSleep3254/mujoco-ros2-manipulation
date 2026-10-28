"""CPU-only, read-only geometry for a browser scene viewer.

The caller validates the XML and every referenced asset against its owned roots
before construction.  This module does not discover files, run ROS, integrate
physics, create a GL context, or assign live-state freshness.  Binding, sequence,
source timestamps and receive ages belong to the owner of the view.

Mesh vertices are MuJoCo's *compiled* mesh vertices.  They are paired with the
compiled geometry's world pose; applying ``mesh_pos`` or ``mesh_quat`` again is
incorrect.  Visual appearance is RGBA only; textures are not exported. Skins,
flex and heightfields are unsupported rather than approximated as rigid meshes.
"""
from collections.abc import Mapping, Sequence
from copy import deepcopy
from dataclasses import dataclass, fields
import hashlib
import json
import math
from numbers import Real
from pathlib import Path


EXPORT_VERSION = "compiled-scene-v1"
DEFAULT_VISIBLE_GROUPS = (0, 1, 2)


@dataclass(frozen=True)
class SceneLimits:
    """Post-compilation export limits; the caller must also bound input assets."""

    max_xml_bytes: int = 16 * 1024 * 1024
    max_compiled_bytes: int = 256 * 1024 * 1024
    max_geoms: int = 10_000
    max_meshes: int = 1_024
    max_vertices: int = 2_000_000
    max_indices: int = 6_000_000
    max_total_asset_bytes: int = 128 * 1024 * 1024
    max_joints: int = 512

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if type(value) is not int or value < 1:
                raise ValueError(f"{field.name} must be a positive integer")


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      allow_nan=False, ensure_ascii=False).encode("utf-8")


def _finite(value, label, *, maximum=1e9):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{label} must be numeric")
    result = float(value)
    if not math.isfinite(result) or abs(result) > maximum:
        raise ValueError(f"{label} must be finite and bounded")
    return result


def _vector(value, count, label, *, maximum=1e9):
    if len(value) != count:
        raise ValueError(f"{label} has an invalid length")
    return [_finite(item, label, maximum=maximum) for item in value]


def compiled_mesh_assets(model, np, limits):
    """Pack checked compiled mesh buffers without choosing a state contract.

    Scalar-only ``CompiledScene`` and explicit observed-qpos recording exporters
    can share byte-for-byte geometry packing. This helper neither creates a
    model nor admits unobserved joints or assigns live-state freshness.
    """
    descriptors, mesh_assets, assets = {}, {}, {}
    total_bytes = 0
    for mesh_id in range(int(model.nmesh)):
        vertex_start, vertex_count = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
        face_start, face_count = int(model.mesh_faceadr[mesh_id]), int(model.mesh_facenum[mesh_id])
        if vertex_count < 1 or face_count < 1:
            raise ValueError("Mesh has no triangle geometry")
        vertices = np.asarray(model.mesh_vert[vertex_start:vertex_start+vertex_count], dtype="<f4")
        faces = np.asarray(model.mesh_face[face_start:face_start+face_count])
        if (vertices.shape != (vertex_count, 3) or faces.shape != (face_count, 3)
                or not np.isfinite(vertices).all() or np.any(np.abs(vertices) > 1e9)
                or np.any(faces < 0) or np.any(faces >= vertex_count)):
            raise ValueError("Mesh geometry is malformed")
        positions = vertices.tobytes(order="C")
        indices = np.asarray(faces, dtype="<u4").tobytes(order="C")
        blob = positions + indices
        asset_sha = hashlib.sha256(blob).hexdigest()
        descriptor = dict(sha256=asset_sha, byte_length=len(blob),
            media_type="application/octet-stream", layout=dict(
                positions=dict(byte_offset=0, byte_length=len(positions), count=vertex_count,
                               item_size=3, component_type="float32"),
                indices=dict(byte_offset=len(positions), byte_length=len(indices), count=face_count*3,
                             item_size=1, component_type="uint32")))
        if asset_sha not in assets:
            total_bytes += len(blob)
            if total_bytes > limits.max_total_asset_bytes:
                raise ValueError("Mesh assets exceed scene resource limits")
            assets[asset_sha] = blob
            descriptors[asset_sha] = descriptor
        elif descriptors[asset_sha] != descriptor:
            raise ValueError("Mesh asset identity has inconsistent layouts")
        mesh_assets[mesh_id] = asset_sha
    return [descriptors[key] for key in sorted(descriptors)], mesh_assets, assets


class CompiledScene:
    """An immutable compiled model and explicitly named scalar-joint contract.

    ``xml_path`` must already be a trusted, validated local model.  All model
    joints must be represented by ``joint_names`` and must be scalar hinge/slide
    joints.  Unobserved free/ball joints would otherwise look authoritative while
    remaining at their default pose, so this first viewer rejects them.

    ``manifest()`` returns a detached JSON value. ``asset(sha256)`` returns one
    immutable little-endian buffer described by its manifest. ``pose(state)``
    takes the *complete* named state, uses its own MjData and only calls
    ``mj_forward``.  Independent preview calls cannot overwrite a live observer's
    MjData.  The caller must still distinguish static, live and saved-plan modes.
    """

    def __init__(self, xml_path, joint_names, home_rad, *, limits=None):
        self._limits = limits if limits is not None else SceneLimits()
        if not isinstance(self._limits, SceneLimits):
            raise TypeError("limits must be SceneLimits")
        if isinstance(joint_names, (str, bytes)) or not isinstance(joint_names, Sequence):
            raise ValueError("joint_names must be a sequence")
        self._joint_names = tuple(joint_names)
        if (len(self._joint_names) > self._limits.max_joints
                or any(not isinstance(name, str) or not name or len(name) > 256
                       for name in self._joint_names)
                or len(set(self._joint_names)) != len(self._joint_names)):
            raise ValueError("joint_names must contain unique bounded names")
        self._home = self._state(home_rad)
        path = Path(xml_path)
        size = path.stat().st_size
        if size < 1 or size > self._limits.max_xml_bytes:
            raise ValueError("XML exceeds scene resource limits")
        model_bytes = path.read_bytes()
        if len(model_bytes) != size:
            raise ValueError("XML changed while reading")
        self.model_sha256 = hashlib.sha256(model_bytes).hexdigest()

        # Keep both optional runtime dependencies out of module import. Neither
        # MjModel/MjData nor MjvScene creates a renderer or a GL context.
        import mujoco
        import numpy as np
        self._mj, self._np = mujoco, np
        self._model = mujoco.MjModel.from_xml_path(str(path))
        if hashlib.sha256(path.read_bytes()).hexdigest() != self.model_sha256:
            raise ValueError("XML changed while compiling")
        model = self._model
        if (model.ngeom > self._limits.max_geoms
                or model.nmesh > self._limits.max_meshes
                or model.nmeshvert > self._limits.max_vertices
                or model.nmeshface * 3 > self._limits.max_indices
                or model.njnt > self._limits.max_joints):
            raise ValueError("Compiled scene exceeds resource limits")
        if model.nflex or model.nskin:
            raise ValueError("Deforming skin/flex geometry is not supported")

        self._addresses = {}
        if model.njnt != len(self._joint_names):
            raise ValueError("Every model joint must have an observed named state")
        for name in self._joint_names:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError("Scene is missing a named joint")
            if int(model.jnt_type[joint_id]) not in (
                    int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise ValueError("Only fully observed scalar joints are supported")
            self._addresses[name] = int(model.jnt_qposadr[joint_id])

        compiled_size = mujoco.mj_sizeModel(model)
        if compiled_size < 1 or compiled_size > self._limits.max_compiled_bytes:
            raise ValueError("Compiled model exceeds scene resource limits")
        compiled = np.empty(compiled_size, dtype=np.uint8)
        mujoco.mj_saveModel(model, buffer=compiled)
        compiled_sha256 = hashlib.sha256(memoryview(compiled)).hexdigest()
        del compiled

        self._assets = {}
        descriptors, mesh_assets = self._mesh_assets()
        self._geom_count = int(model.ngeom)
        self._geom_types = {
            int(mujoco.mjtGeom.mjGEOM_BOX): "box",
            int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
            int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
            int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
            int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
            int(mujoco.mjtGeom.mjGEOM_MESH): "mesh",
        }
        geoms = []
        for geom_id in range(self._geom_count):
            kind = self._geom_types.get(int(model.geom_type[geom_id]))
            if kind is None:
                raise ValueError("Unsupported geometry type in scene")
            group = int(model.geom_group[geom_id])
            size = _vector(model.geom_size[geom_id], 3, "geometry size")
            if any(item < 0 for item in size) or (kind != "plane" and size[0] <= 0):
                raise ValueError("Invalid geometry size")
            if kind in {"box", "cylinder"} and size[1] <= 0:
                raise ValueError("Invalid geometry size")
            if kind == "box" and size[2] <= 0:
                raise ValueError("Invalid geometry size")
            asset_sha = None
            if kind == "mesh":
                mesh_id = int(model.geom_dataid[geom_id])
                if mesh_id not in mesh_assets:
                    raise ValueError("Geometry references an invalid mesh")
                asset_sha = mesh_assets[mesh_id]
            rgba = model.geom_rgba[geom_id]
            material_id = int(model.geom_matid[geom_id])
            if material_id >= 0 and list(rgba) == [0.5, 0.5, 0.5, 1.0]:
                rgba = model.mat_rgba[material_id]
            rgba = _vector(rgba, 4, "geometry RGBA", maximum=1)
            if any(item < 0 for item in rgba):
                raise ValueError("Invalid geometry RGBA")
            geoms.append(dict(id=geom_id,
                name=mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or "",
                type=kind, group=group, default_visible=group in DEFAULT_VISIBLE_GROUPS,
                size=size, rgba=rgba, asset_sha256=asset_sha))

        # MjvScene resolves MuJoCo material/geom RGBA precedence without rendering.
        # It is not retained: poses use the underlying compiled geom transforms.
        data = self._data(self._home)
        options = mujoco.MjvOption()
        options.geomgroup[:] = 1
        scene = mujoco.MjvScene(model, maxgeom=max(1, self._geom_count))
        mujoco.mjv_updateScene(model, data, options, None, mujoco.MjvCamera(),
                               mujoco.mjtCatBit.mjCAT_ALL, scene)
        for visual in scene.geoms[:scene.ngeom]:
            if (visual.objtype == mujoco.mjtObj.mjOBJ_GEOM
                    and 0 <= visual.objid < self._geom_count):
                rgba = _vector(visual.rgba, 4, "geometry RGBA", maximum=1)
                if any(item < 0 for item in rgba):
                    raise ValueError("Invalid geometry RGBA")
                geoms[int(visual.objid)]["rgba"] = rgba
        self._manifest = dict(schema_version=1, export_version=EXPORT_VERSION,
            model_sha256=self.model_sha256, compiled_model_sha256=compiled_sha256,
            mujoco_version=str(mujoco.__version__),
            units=dict(length="m", angle="rad", up_axis="Z", rotation="row-major"),
            joint_names=list(self._joint_names), home_rad=dict(self._home),
            default_visible_groups=list(DEFAULT_VISIBLE_GROUPS),
            geoms=geoms, assets=descriptors)
        self.scene_sha256 = hashlib.sha256(_canonical(self._manifest)).hexdigest()
        self._manifest["scene_sha256"] = self.scene_sha256

    def _state(self, value):
        if not isinstance(value, Mapping) or set(value) != set(self._joint_names):
            raise ValueError("State must contain exactly the named joints")
        return {name: _finite(value[name], "joint state", maximum=1e6)
                for name in self._joint_names}

    def _data(self, state):
        data = self._mj.MjData(self._model)
        for name, address in self._addresses.items():
            data.qpos[address] = state[name]
        self._mj.mj_forward(self._model, data)
        return data

    def _mesh_assets(self):
        descriptors, mesh_assets, self._assets = compiled_mesh_assets(
            self._model, self._np, self._limits)
        return descriptors, mesh_assets

    def manifest(self):
        """Return a detached, finite JSON manifest without filesystem paths."""
        return deepcopy(self._manifest)

    def asset(self, sha256):
        """Look up an owned immutable mesh; arbitrary paths are never accepted."""
        if (not isinstance(sha256, str) or len(sha256) != 64
                or any(c not in "0123456789abcdef" for c in sha256)):
            raise ValueError("Invalid mesh asset identity")
        if sha256 not in self._assets:
            raise KeyError("Unknown mesh asset")
        return self._assets[sha256]

    def pose(self, named_state):
        """Compute one full pose, with no publication, time step, or live claim."""
        data = self._data(self._state(named_state))
        geoms = [dict(id=geom_id,
            position=_vector(data.geom_xpos[geom_id], 3, "geometry position"),
            rotation=_vector(data.geom_xmat[geom_id], 9, "geometry rotation", maximum=1.000001))
            for geom_id in range(self._geom_count)]
        return dict(schema_version=1, scene_sha256=self.scene_sha256,
                    model_sha256=self.model_sha256, geoms=geoms)
