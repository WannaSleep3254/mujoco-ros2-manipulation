"""Export observed FR5/gripper/free-block states as a read-only display recording.

This adapter reconstructs geometry with ``mj_forward`` in a private MjData; it
never steps physics, connects ROS, starts a server or submits a command. The
existing scalar-only CompiledScene remains strict. Fingers and the free block
are carried by complete geometry transforms, not disguised as radian joints.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ..operator_console.display_contract import WORLD, VERSION, scene_digest, validate_manifest, validate_pose
from ..operator_console.recording import Recording, MAX_FRAMES, MAX_RECORDING_BYTES
from ..operator_console.scene_host import _canonical, _json, _regular, _require, MAX_MANIFEST_BYTES
from ..operator_console.scene_view import SceneLimits, DEFAULT_VISIBLE_GROUPS, compiled_mesh_assets, _vector
from .fr5_model import ARM_JOINTS, TCP_SITE


FINGER_JOINTS = ("fr5_left_finger_slide", "fr5_right_finger_slide")
FREE_JOINT = "fr5_block_free"
MAX_SOURCE_RESULT_BYTES = 8 * 1024 * 1024
MAX_SOURCE_MANIFEST_BYTES = 1024 * 1024
RECONSTRUCTION_TOLERANCE = 1e-9


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _manifest_digest(value):
    return _sha(json.dumps({key: item for key, item in value.items() if key != "manifest_sha256"},
        sort_keys=True, separators=(",", ":"), allow_nan=False).encode())


def _bounded_numbers(value, size, label):
    _require(type(value) is list and len(value) == size, "Missing complete " + label)
    return _vector(value, size, label, maximum=1e6)


def _validate_asset_inputs(xml, manifest, limits):
    """Validate every compiler file read before passing the XML to MuJoCo."""
    root = ET.fromstring(xml)
    _require(root.tag == "mujoco", "Expected an explicit MJCF prototype")
    _require(all(child.tag in {"compiler", "option", "asset", "worldbody", "contact", "actuator", "keyframe"}
                 for child in root), "Unsupported prototype MJCF section")
    declared = manifest.get("mesh_inputs_sha256")
    _require(type(declared) is dict and len(declared) <= limits.max_meshes, "Missing bounded mesh allowlist")
    inputs, total = {}, 0
    mesh_elements = list(root.findall("asset/mesh"))
    for element in root.iter():
        _require(element.tag not in {"include", "texture", "hfield", "skin", "flex", "plugin", "extension"},
                 "Unsupported external/deforming prototype asset")
        for name, value in element.attrib.items():
            if name == "file":
                _require(element in mesh_elements and value in declared and Path(value).is_absolute(),
                         "MJCF file is outside the declared mesh allowlist")
                raw = _regular(Path(value), limits.max_total_asset_bytes)
                total += len(raw)
                _require(total <= limits.max_total_asset_bytes and _sha(raw) == declared[value],
                         "Prototype source mesh digest or byte budget differs")
                inputs[value] = _sha(raw)
            elif element.tag == "compiler" and name.endswith("dir"):
                _require(name == "meshdir" and value == ".", "Unexpected prototype compiler asset directory")
    _require(set(inputs) == set(declared), "Unused or missing mesh allowlist entry")
    return inputs


def _state_contract(model, mj):
    rows, scalar = [], []
    kinds = {int(mj.mjtJoint.mjJNT_HINGE): ("hinge", 1),
             int(mj.mjtJoint.mjJNT_SLIDE): ("slide", 1),
             int(mj.mjtJoint.mjJNT_FREE): ("free", 7)}
    expected_types = {**dict.fromkeys(ARM_JOINTS, "hinge"),
                      **dict.fromkeys(FINGER_JOINTS, "slide"), FREE_JOINT: "free"}
    for index in range(model.njnt):
        _require(int(model.jnt_type[index]) in kinds, "Unsupported prototype joint type")
        kind, width = kinds[int(model.jnt_type[index])]
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, index)
        _require(type(name) is str and name, "Every prototype joint must be named")
        _require(kind == expected_types.get(name),
                 "Prototype joint type must match FR5 hinges, finger slides and the free block")
        rows.append(dict(name=name, type=kind, qpos_address=int(model.jnt_qposadr[index]), qpos_width=width))
        if width == 1:
            scalar.append(name)
    _require(model.nq == 15 and tuple(scalar) == ARM_JOINTS + FINGER_JOINTS
             and [row["name"] for row in rows if row["type"] == "free"] == [FREE_JOINT],
             "Expected observed FR5, two scalar fingers and one complete free block")
    return dict(nq=int(model.nq), qpos_layout=rows,
        free_joint_format="world xyz metres followed by quaternion wxyz",
        scalar_joint_names=scalar,
        scalar_joint_units={name: "rad" if name in ARM_JOINTS else "m" for name in scalar},
        scalar_joint_positions="Actual MuJoCo qpos observations, never position-servo ctrl targets")


def _geometry_manifest(model, data, mj, np, limits, model_sha, source_sha, first, addresses):
    _require(not model.nskin and not model.nflex, "Deforming prototype geometry is unsupported")
    _require(model.ngeom <= limits.max_geoms and model.nsite <= limits.max_geoms and model.nmesh <= limits.max_meshes
             and model.nmeshvert <= limits.max_vertices and model.nmeshface * 3 <= limits.max_indices
             and model.njnt <= limits.max_joints,
             "Prototype compiled scene exceeds resource limits")
    size = int(mj.mj_sizeModel(model))
    _require(0 < size <= limits.max_compiled_bytes, "Prototype compiled model exceeds byte limit")
    compiled = np.empty(size, dtype=np.uint8)
    mj.mj_saveModel(model, buffer=compiled)
    compiled_sha = _sha(memoryview(compiled))
    del compiled
    descriptors, mesh_map, assets = compiled_mesh_assets(model, np, limits)
    types = {int(getattr(mj.mjtGeom, "mjGEOM_" + key.upper())): key
             for key in ("box", "sphere", "cylinder", "capsule", "plane", "mesh")}
    geoms = []
    for index in range(model.ngeom):
        kind = types.get(int(model.geom_type[index]))
        _require(kind is not None, "Unsupported prototype geometry type")
        material = int(model.geom_matid[index])
        rgba = model.geom_rgba[index]
        if material >= 0 and list(rgba) == [.5, .5, .5, 1.]:
            rgba = model.mat_rgba[material]
        group = int(model.geom_group[index])
        geoms.append(dict(id=index, name=mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, index) or "",
            type=kind, group=group, default_visible=group in DEFAULT_VISIBLE_GROUPS,
            size=_vector(model.geom_size[index], 3, "prototype geometry size"),
            rgba=_vector(rgba, 4, "prototype geometry RGBA", maximum=1),
            asset_sha256=mesh_map[int(model.geom_dataid[index])] if kind == "mesh" else None))
    # Same CPU-only material precedence as CompiledScene. No GL context exists.
    options, camera = mj.MjvOption(), mj.MjvCamera()
    options.geomgroup[:] = 1
    scene = mj.MjvScene(model, maxgeom=max(1, model.ngeom + model.nsite + 8))
    mj.mjv_updateScene(model, data, options, None, camera, mj.mjtCatBit.mjCAT_ALL, scene)
    for visual in scene.geoms[:scene.ngeom]:
        if visual.objtype == mj.mjtObj.mjOBJ_GEOM and 0 <= visual.objid < model.ngeom:
            geoms[int(visual.objid)]["rgba"] = _vector(visual.rgba, 4, "prototype geometry RGBA", maximum=1)
    manifest = dict(schema_version=2, export_version=VERSION, model_sha256=model_sha,
        provenance=dict(adapter="mujoco-fr5-grasp-recording", source_sha256=source_sha,
            engine=dict(name="mujoco", version=str(mj.__version__), compiled_sha256=compiled_sha)),
        world_frame=deepcopy(WORLD), units=dict(length="m", angle="rad", up_axis="Z", rotation="row-major"),
        joint_names=list(ARM_JOINTS), home_rad={name: first[addresses[name]] for name in ARM_JOINTS},
        default_visible_groups=list(DEFAULT_VISIBLE_GROUPS), geoms=geoms, assets=descriptors)
    manifest["scene_sha256"] = scene_digest(manifest)
    return validate_manifest(manifest), assets


def _reconstruct(frame, model, data, contract, mj, np):
    _require(type(frame) is dict, "Missing observed prototype frame")
    qpos = _bounded_numbers(frame.get("qpos"), contract["nq"], "observed qpos")
    scalar = frame.get("scalar_joint_positions")
    names = contract["scalar_joint_names"]
    _require(type(scalar) is dict and set(scalar) == set(names), "Missing observed finger/scalar joints")
    for row in contract["qpos_layout"]:
        address = row["qpos_address"]
        if row["qpos_width"] == 1:
            _require(type(scalar[row["name"]]) in (int, float)
                     and math.isfinite(scalar[row["name"]])
                     and scalar[row["name"]] == qpos[address], "Observed scalar joints differ from qpos")
        else:
            _require(abs(math.hypot(*qpos[address+3:address+7])-1.) <= 1e-6,
                     "Observed free-body quaternion must be unit wxyz")
    data.qpos[:] = qpos
    mj.mj_forward(model, data)
    _require(np.isfinite(data.geom_xpos).all() and np.isfinite(data.geom_xmat).all(),
             "Nonfinite reconstructed geometry")
    block = int(model.body("fr5_grasp_block").id)
    tcp = int(model.site(TCP_SITE).id)
    errors = {}
    for name, actual in (("block_position_m", data.xpos[block]), ("tcp_position_m", data.site_xpos[tcp])):
        observed = _bounded_numbers(frame.get(name), 3, name)
        errors[name] = float(np.max(np.abs(np.asarray(observed)-actual)))
    tcp_rotation = np.asarray(frame.get("tcp_rotation_matrix"), dtype=float)
    relative = np.asarray(frame.get("relative_rotation_matrix"), dtype=float)
    _require(tcp_rotation.shape == (3, 3) and relative.shape == (3, 3)
             and np.isfinite(tcp_rotation).all() and np.isfinite(relative).all(),
             "Missing observed block/TCP rotations")
    errors["tcp_rotation"] = float(np.max(np.abs(tcp_rotation-data.site_xmat[tcp].reshape(3, 3))))
    errors["block_rotation"] = float(np.max(np.abs(tcp_rotation @ relative-data.xmat[block].reshape(3, 3))))
    _require(max(errors.values()) <= RECONSTRUCTION_TOLERANCE,
             "Reconstructed model pose differs from the actual observations")
    arm = _bounded_numbers(frame.get("arm_rad"), len(ARM_JOINTS), "observed FR5 radians")
    _require(arm == [scalar[name] for name in ARM_JOINTS], "Observed FR5 angles differ from qpos")
    return qpos, errors


def _implementation_sha():
    module_root = Path(__file__).parents[1]
    return {name: _sha((module_root/name).read_bytes()) for name in (
        "grasping/display_export.py", "operator_console/scene_view.py",
        "operator_console/display_contract.py", "operator_console/recording.py")}


class ObservedModelDisplay:
    """A common display catalog for an existing owned, immutable FR5 model.

    Construction reads the supplied model/data once. ``pose(data)`` reads geom
    transforms only: the producer must have completed its own mj_forward after
    the actual observation. This class never compiles, forwards or steps that
    model. Binding, run identity, sequence, cancellation and age stay with the
    owner; a geometry catalog is not evidence of a fresh physical observation.
    """
    def __init__(self, model, data, *, model_sha256, source_sha256, limits=None):
        import mujoco as mj
        import numpy as np
        self._mj, self._np, self._model = mj, np, model
        self._limits = SceneLimits() if limits is None else limits
        _require(isinstance(self._limits, SceneLimits) and data.model is model,
                 "Observed display data belongs to another model")
        self._contract = _state_contract(model, mj)
        self._check_data(data)
        addresses = {row["name"]: row["qpos_address"] for row in self._contract["qpos_layout"]
                     if row["qpos_width"] == 1}
        self._manifest, self._assets = _geometry_manifest(model, data, mj, np, self._limits,
            model_sha256, source_sha256, data.qpos.tolist(), addresses)
        self._raw = _canonical(self._manifest)+b"\n"
        _require(len(self._raw) <= MAX_MANIFEST_BYTES, "Observed display manifest exceeds byte limit")

    def _check_data(self, data):
        np, model = self._np, self._model
        _require(data.model is model and data.qpos.shape == (self._contract["nq"],)
                 and data.geom_xpos.shape == (model.ngeom, 3)
                 and data.geom_xmat.shape == (model.ngeom, 9)
                 and np.isfinite(data.qpos).all() and np.isfinite(data.geom_xpos).all()
                 and np.isfinite(data.geom_xmat).all(), "Incomplete or foreign observed model data")
        free = next(row for row in self._contract["qpos_layout"] if row["name"] == FREE_JOINT)
        address = free["qpos_address"]
        _require(abs(float(np.linalg.norm(data.qpos[address+3:address+7]))-1.) <= 1e-6,
                 "Observed free-body quaternion must be unit wxyz")

    @property
    def manifest(self):
        return deepcopy(self._manifest)

    @property
    def manifest_raw(self):
        return self._raw

    @property
    def assets(self):
        return dict(self._assets)

    @property
    def state_contract(self):
        return deepcopy(self._contract)

    def pose(self, data):
        self._check_data(data)
        return validate_pose(dict(schema_version=1, model_sha256=self._manifest["model_sha256"],
            scene_sha256=self._manifest["scene_sha256"], geoms=[dict(id=index,
                position=_vector(data.geom_xpos[index], 3, "observed world geometry position"),
                rotation=_vector(data.geom_xmat[index], 9, "observed geometry rotation", maximum=1.000001))
                for index in range(self._model.ngeom)]), self._manifest)


class LiveGraspDisplay:
    """Validate fixed model bytes once; reconstruct only explicit observations.

    The private keyframe prepares geometry/material metadata only. No live pose
    is published until ``pose(frame)`` receives a complete actual observation.
    The parent owner must pin the manifest/job, serialize calls and wrap returned
    poses in fresh run/generation/sequence envelopes. No robot command exists.
    """
    def __init__(self, model_xml, physical_manifest, limits=None):
        self._limits = SceneLimits() if limits is None else limits
        _require(isinstance(self._limits, SceneLimits) and type(model_xml) is bytes
                 and 0 < len(model_xml) <= self._limits.max_xml_bytes,
                 "Expected bounded explicit prototype model bytes")
        _require(type(physical_manifest) is dict, "Expected a fixed physical manifest")
        manifest = deepcopy(physical_manifest)
        _require(manifest.get("schema_version") == 1 and manifest.get("simulation_only") is True
                 and manifest.get("prototype") == "fr5_parallel_gripper_v1"
                 and manifest.get("manifest_sha256") == _manifest_digest(manifest)
                 and manifest.get("mjcf_sha256") == _sha(model_xml),
                 "Live prototype manifest/model digest differs")
        meshes = _validate_asset_inputs(model_xml, manifest, self._limits)
        import mujoco as mj
        import numpy as np
        _require(manifest.get("physical_engine_used_for_static_fk") == "MuJoCo "+mj.__version__,
                 "Live display engine differs from the physical manifest")
        model = mj.MjModel.from_xml_string(model_xml.decode("utf-8"))
        self._mj, self._np, self._model = mj, np, model
        self._contract = _state_contract(model, mj)
        _require(manifest.get("qpos_state_contract") == self._contract,
                 "Live prototype lacks its complete observed qpos contract")
        self._data = mj.MjData(model)
        if model.nkey:
            mj.mj_resetDataKeyframe(model, self._data, 0)
        mj.mj_forward(model, self._data)
        source_sha = _sha(_canonical(dict(manifest_sha256=manifest["manifest_sha256"],
            model_sha256=manifest["mjcf_sha256"], mesh_inputs=meshes, implementation=_implementation_sha())))
        self._display = ObservedModelDisplay(model, self._data,
            model_sha256=manifest["mjcf_sha256"], source_sha256=source_sha, limits=self._limits)
        for path, digest in meshes.items():
            _require(_sha(_regular(Path(path), self._limits.max_total_asset_bytes)) == digest,
                     "Live source mesh changed during preparation")

    @property
    def manifest(self):
        return self._display.manifest

    @property
    def manifest_raw(self):
        return self._display.manifest_raw

    @property
    def assets(self):
        return self._display.assets

    @property
    def state_contract(self):
        return deepcopy(self._contract)

    def pose(self, frame):
        _reconstruct(frame, self._model, self._data, self._contract, self._mj, self._np)
        return self._display.pose(self._data)


def export_recording(source_directory, output_directory, *, expected_result_sha256=None,
                     expected_manifest_sha256=None, limits=None):
    """Validate a frozen local run and write a NEW adjacent recording bundle.

    Optional expected digests bind this explicit export to its owner/job receipt.
    Existing files are never overwritten. Failed/cancelled physical outcomes are
    retained as such in the receipt and label rather than filtered into success.
    """
    source, output = Path(source_directory).resolve(), Path(output_directory).resolve()
    _require(output != source and not output.is_relative_to(source), "Export output must be outside the frozen source run")
    _require(not output.exists(), "Export output must be a new directory")
    limits = SceneLimits() if limits is None else limits
    _require(isinstance(limits, SceneLimits), "Expected explicit scene resource limits")
    raw = {"manifest.json": _regular(source/"manifest.json", MAX_SOURCE_MANIFEST_BYTES),
           "result.json": _regular(source/"result.json", MAX_SOURCE_RESULT_BYTES),
           "fr5_grasp.xml": _regular(source/"fr5_grasp.xml", limits.max_xml_bytes)}
    manifest, result = _json(raw["manifest.json"]), _json(raw["result.json"])
    _require(type(manifest) is dict and type(result) is dict, "Expected prototype manifest and result objects")
    _require((expected_result_sha256 is None or _sha(raw["result.json"]) == expected_result_sha256)
             and (expected_manifest_sha256 is None or manifest.get("manifest_sha256") == expected_manifest_sha256),
             "Export does not match the expected physical run receipt")
    _require(manifest.get("schema_version") == 1 and manifest.get("simulation_only") is True
             and manifest.get("prototype") == result.get("prototype") == "fr5_parallel_gripper_v1"
             and result.get("schema_version") == 1, "Unsupported FR5 prototype source")
    _require(manifest.get("manifest_sha256") == _manifest_digest(manifest)
             and result.get("manifest_sha256") == manifest["manifest_sha256"]
             and manifest.get("mjcf_sha256") == _sha(raw["fr5_grasp.xml"]),
             "Prototype manifest/result/model digest mismatch")
    meshes = _validate_asset_inputs(raw["fr5_grasp.xml"], manifest, limits)
    import mujoco as mj
    import numpy as np
    _require(result.get("engine") == manifest.get("physical_engine_used_for_static_fk") == "MuJoCo " + mj.__version__,
             "Export engine must match the observed physical run")
    model = mj.MjModel.from_xml_string(raw["fr5_grasp.xml"].decode("utf-8"))
    contract = _state_contract(model, mj)
    _require(manifest.get("qpos_state_contract") == result.get("qpos_state_contract") == contract,
             "Missing or changed complete observed qpos contract")
    samples = result.get("samples")
    _require(type(samples) is list and 2 <= len(samples) <= min(2500, MAX_FRAMES), "Missing bounded observed frames")
    outcome = result.get("outcome")
    _require(outcome in {"succeeded", "failed", "cancelled"}, "Physical run has no terminal outcome")
    input_sha = {name: _sha(value) for name, value in raw.items()}
    implementation_sha = _implementation_sha()
    source_sha = _sha(_canonical(dict(files=input_sha, mesh_inputs=meshes, implementation=implementation_sha)))
    data = mj.MjData(model)
    _reconstruct(samples[0], model, data, contract, mj, np)
    core = ObservedModelDisplay(model, data, model_sha256=manifest["mjcf_sha256"],
        source_sha256=source_sha, limits=limits)
    display, assets = core.manifest, core.assets
    frames, previous, origin, byte_count = [], None, None, len(_canonical(display))
    maximum_errors = {}
    duplicates = 0
    for sample in samples:
        stamp = sample.get("simulation_sec") if type(sample) is dict else None
        _require(type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= stamp <= 3600,
                 "Invalid source simulation time")
        qpos, errors = _reconstruct(sample, model, data, contract, mj, np)
        for name, error in errors.items():
            maximum_errors[name] = max(maximum_errors.get(name, 0.), error)
        if previous is not None:
            _require(stamp >= previous[0], "Observed simulation time moved backwards")
            if stamp == previous[0]:
                _require(qpos == previous[1], "Different observed poses share one simulation time")
                duplicates += 1
                continue
        if origin is None:
            origin = stamp
        pose = core.pose(data)
        frame = dict(time_sec=stamp-origin, positions_rad={name: sample["scalar_joint_positions"][name]
            for name in ARM_JOINTS}, pose=pose)
        byte_count += len(_canonical(frame))
        _require(byte_count <= MAX_RECORDING_BYTES, "Reconstructed recording exceeds byte budget")
        frames.append(frame)
        previous = stamp, qpos
    document = dict(schema_version=1, kind="display-recording-v1",
        label="FR5 물리 집기 기록 · " + outcome + " · 읽기 전용", manifest=display, frames=frames)
    checked = Recording(document, assets)
    recording_raw = _canonical(document) + b"\n"
    _require(len(recording_raw) <= MAX_RECORDING_BYTES, "Recording file exceeds byte limit")
    # Refuse a file/asset replacement during compilation or reconstruction.
    for name, value in raw.items():
        _require(_regular(source/name, len(value)) == value, "Physical run changed during display export")
    for path, digest in meshes.items():
        _require(_sha(_regular(Path(path), limits.max_total_asset_bytes)) == digest,
                 "Source mesh changed during display export")
    receipt = dict(schema_version=1, kind="fr5-grasp-display-export-v1", readonly=True,
        source_files_sha256=input_sha, source_manifest_sha256=manifest["manifest_sha256"],
        implementation_sha256=implementation_sha,
        source_bundle_sha256=source_sha, source_outcome=outcome, source_error=deepcopy(result.get("error")),
        source_engine=result["engine"], display_model_sha256=display["model_sha256"],
        display_scene_sha256=display["scene_sha256"], compiled_model_sha256=display["provenance"]["engine"]["compiled_sha256"],
        recording_sha256=checked.sha256, recording_file_sha256=_sha(recording_raw),
        recording_bytes=len(recording_raw), frames=len(frames), geoms=len(display["geoms"]),
        assets=len(assets), asset_bytes=sum(map(len, assets.values())),
        time=dict(clock_domain="source simulation, rebased to first actual observation",
            source_origin_sec=origin, source_end_sec=previous[0], duration_sec=checked.duration,
            duplicate_identical_observations_collapsed=duplicates),
        observed_qpos_contract=contract, reconstructed_pose_max_absolute_errors=maximum_errors,
        joint_positions_rad_scope="FR5 six rotational joints only; observed metre finger joints and free-body pose are represented by complete geom transforms",
        physics_steps_during_export=0, robot_command_capability=False,
        w07_complete=False, scope="Observed physical prototype replay; no ROS/native command integration, new planning approval or live-state claim")
    output.mkdir(parents=True, exist_ok=False)
    folder = output/"assets"
    folder.mkdir()
    for digest, blob in assets.items():
        (folder/(digest+".bin")).write_bytes(blob)
    (output/"recording.json").write_bytes(recording_raw)
    (output/"export-receipt.json").write_bytes(_canonical(receipt)+b"\n")
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-result-sha256")
    parser.add_argument("--expected-manifest-sha256")
    args = parser.parse_args(argv)
    receipt = export_recording(args.source, args.output,
        expected_result_sha256=args.expected_result_sha256,
        expected_manifest_sha256=args.expected_manifest_sha256)
    print(json.dumps(receipt, ensure_ascii=False, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
