"""Portable geometric replay of a recorded failure witness; no ROS imports."""
from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.workcells.motion_diagnostics import measure_endpoints
from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.evidence_export import EXPORT_NOTE, audit_public_evidence


ROOT_TOKEN = "{REPOSITORY_ROOT}"


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _digest(value):
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise ValueError("Expected a lowercase SHA-256 digest")
    return value


def _number(value, *, positive=False):
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and (not positive or value > 0)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError("Expected a finite numeric value" + (" greater than zero" if positive else ""))
    return value


def _keys(value, required, optional=()):
    if not isinstance(value, dict) or not set(required) <= set(value) or set(value) - set(required) - set(optional):
        raise ValueError("Unknown or missing reproduction fields")


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("Duplicate JSON field: " + key)
        result[key] = value
    return result


def _json(raw):
    value = json.loads(raw, object_pairs_hook=_pairs)
    if not isinstance(value, dict):
        raise ValueError("Expected a JSON object")
    return value


def _relative_file(root, name):
    if not isinstance(name, str) or not name or Path(name).is_absolute() or ".." in Path(name).parts:
        raise ValueError("Provenance path must be repository-relative")
    path = (root / name).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError("Provenance file is missing or escapes the repository")
    return path


def _export_metadata(data, kind):
    metadata = data["evidence_export"]
    _keys(metadata, {"schema_version", "kind", "raw_source_sha256", "retained_fields", "note"})
    if (type(metadata["schema_version"]) is not int or metadata["schema_version"] != 1
            or metadata["kind"] != kind or metadata["note"] != EXPORT_NOTE
            or metadata["retained_fields"] != [name for name in data if name != "evidence_export"]):
        raise ValueError("Expected an explicitly privacy-filtered evidence extract")
    _digest(metadata["raw_source_sha256"])
    audit_public_evidence(data)


def _provenance(root, source, *, uncompressed_sha256=None, export_kind=None):
    _keys(source, {"path", "sha256"})
    path = _relative_file(root, source["path"])
    raw = path.read_bytes()
    if _sha(raw) != _digest(source["sha256"]):
        raise ValueError("Provenance SHA-256 mismatch: " + source["path"])
    payload = gzip.decompress(raw) if path.suffix == ".gz" else raw
    if uncompressed_sha256 is not None and _sha(payload) != _digest(uncompressed_sha256):
        raise ValueError("Uncompressed provenance SHA-256 mismatch: " + source["path"])
    result = _json(payload)
    if export_kind is not None:
        _export_metadata(result, export_kind)
    return result


def _state(value, joints, limits):
    if not isinstance(value, dict) or set(value) != set(joints):
        raise ValueError("Expected a complete named workcell joint state")
    for name, position in value.items():
        _number(position)
        if not limits[name]["lower"] <= position <= limits[name]["upper"]:
            raise ValueError("Reproduction joint state exceeds limit: " + name)


def _close_state(left, right):
    return set(left) == set(right) and all(abs(left[name] - right[name]) <= 1e-12 for name in left)


def _assets(root, xml_path, raw):
    xml = ET.fromstring(raw)
    compiler = xml.find("compiler")
    meshdir = Path(compiler.get("meshdir", ".") if compiler is not None else ".")
    if not meshdir.is_absolute():
        meshdir = xml_path.parent / meshdir
    result = {}
    for mesh in xml.findall("./asset/mesh"):
        path = (meshdir / mesh.attrib["file"]).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            raise ValueError("Mesh asset escapes the repository or is missing")
        result[path.relative_to(root).as_posix()] = _sha(path.read_bytes())
    return result


def _geometry(root, cell, validated, historical, geometry):
    _keys(geometry, {"cell_xml_canonical_sha256", "repository_root_token", "asset_sha256",
                     "config_sha256", "workcell_sha256", "baseline_raw_cell_xml_sha256",
                     "baseline_verified_against_session_output"})
    if geometry["repository_root_token"] != ROOT_TOKEN or geometry["baseline_verified_against_session_output"] is not True:
        raise ValueError("Expected a verified portable geometry baseline")
    if geometry["workcell_sha256"] != cell.fingerprint:
        raise ValueError("Geometry workcell fingerprint mismatch")
    baseline = _digest(geometry["baseline_raw_cell_xml_sha256"])
    if baseline != historical["outputs"]["cell.xml"]:
        raise ValueError("Geometry baseline does not match the historical session output")
    xml_path = cell.output / "cell.xml"
    raw = xml_path.read_bytes()
    current = _sha(raw)
    if current != validated["outputs"]["cell.xml"]:
        raise ValueError("Current geometry does not match the validated manifest")
    canonical = _sha(raw.decode("utf-8").replace(str(root), ROOT_TOKEN).encode("utf-8"))
    if canonical != _digest(geometry["cell_xml_canonical_sha256"]):
        raise ValueError("Canonical workcell geometry SHA-256 mismatch")
    assets = geometry["asset_sha256"]
    if not isinstance(assets, dict) or not assets:
        raise ValueError("Expected pinned mesh assets")
    for name, digest in assets.items():
        _relative_file(root, name)
        _digest(digest)
    if _assets(root, xml_path, raw) != assets:
        raise ValueError("Mesh asset inventory or SHA-256 mismatch")
    configs = geometry["config_sha256"]
    if not isinstance(configs, dict) or set(configs) != {f"config/workcells/{cell.id}.yaml", "config/controller_defaults.yaml"}:
        raise ValueError("Expected workcell and controller configuration digests")
    for name, digest in configs.items():
        _digest(digest)
        if _sha(_relative_file(root, name).read_bytes()) != digest or historical["inputs"][name] != digest or validated["inputs"][name] != digest:
            raise ValueError("Workcell configuration SHA-256 mismatch")
    return dict(raw_cell_xml_sha256=dict(expected=baseline, current=current, matches=baseline == current),
                canonical_cell_xml_sha256=canonical, mesh_asset_count=len(assets),
                portable_geometry_matches=True)


def load_reproduction(path, cell, validated_manifest):
    """Validate source hashes and portable model identity; return detached input data."""
    try:
        root = cell.root.resolve()
        raw = Path(path).read_bytes()
        fixture = _json(raw)
        _keys(fixture, {"schema_version", "id", "workcell", "initial_rad", "target_rad", "witness", "source", "expected"}, {"setup", "note"})
        if type(fixture["schema_version"]) is not int or fixture["schema_version"] != 1:
            raise ValueError("Expected reproduction schema 1")
        if not isinstance(fixture["id"], str) or not re.fullmatch(r"[a-z][a-z0-9_-]*", fixture["id"]) or fixture["workcell"] != cell.id:
            raise ValueError("Reproduction ID or workcell mismatch")
        manifest = validated_manifest
        if manifest["workcell_sha256"] != cell.fingerprint or list(manifest["joints"]) != list(cell.joints) or len(cell.joints) != 12:
            raise ValueError("Validated workcell fingerprint or joint identity mismatch")
        limits = manifest["joint_limits"]
        _state(fixture["initial_rad"], cell.joints, limits)
        _state(fixture["target_rad"], cell.joints, limits)
        witness = fixture["witness"]
        _keys(witness, {"time_sec", "state_rad", "distance_m", "bodies"})
        _state(witness["state_rad"], cell.joints, limits)
        _number(witness["time_sec"], positive=True)
        _number(witness["distance_m"])
        if not isinstance(witness["bodies"], list) or len(witness["bodies"]) != 2 or not all(isinstance(name, str) and name for name in witness["bodies"]):
            raise ValueError("Expected the historical witness body pair")
        expected = fixture["expected"]
        _keys(expected, {"location", "threshold_m"})
        _number(expected["threshold_m"], positive=True)
        if expected["location"] != "intermediate" or witness["distance_m"] >= expected["threshold_m"]:
            raise ValueError("Expected an intermediate subthreshold historical witness")
        source = fixture["source"]
        _keys(source, {"code_commit", "request_ids", "traces", "session", "geometry"})
        if not isinstance(source["code_commit"], str) or not re.fullmatch(r"[0-9a-f]{40}", source["code_commit"]):
            raise ValueError("Expected a historical Git commit")
        _keys(source["request_ids"], {"coordinated", "departure_timing"})
        _keys(source["traces"], {"coordinated", "departure_timing"})
        session = _provenance(root, source["session"], export_kind="session")
        _keys(session, {"schema_version", "workcell", "model_manifest", "policy", "evaluator",
                        "code_commit", "implementation_sha256", "evidence_export"}, {"workcell_contract"})
        if type(session["schema_version"]) is not int or session["schema_version"] != 1 or session["workcell"] != cell.id or session["code_commit"] != source["code_commit"]:
            raise ValueError("Historical session schema or identity mismatch")
        historical = session["model_manifest"]
        if historical["workcell_sha256"] != cell.fingerprint or historical["joints"] != list(cell.joints) or historical["joint_limits"] != limits:
            raise ValueError("Historical model fingerprint or joint contract mismatch")
        policy = session["policy"]
        if not math.isclose(policy["min_inter_arm_distance_m"] + policy["tracking_reserve_m"], expected["threshold_m"], rel_tol=0, abs_tol=1e-12):
            raise ValueError("Historical policy threshold mismatch")
        evaluator = session["evaluator"]
        _digest(evaluator["source_sha256"])
        _digest(evaluator["library_sha256"])
        if evaluator["engine"] != "MuJoCo 3.12.0" or _sha((root / "native/path_distance.cpp").read_bytes()) != evaluator["source_sha256"]:
            raise ValueError("Historical native engine or source SHA-256 mismatch")
        for role in ("coordinated", "departure_timing"):
            identifier = source["request_ids"][role]
            if not isinstance(identifier, str) or not re.fullmatch(r"[0-9a-f]{32}", identifier):
                raise ValueError("Expected a named historical request ID")
            trace_source = source["traces"][role]
            if Path(trace_source["path"]).name not in {identifier + ".json", identifier + ".json.gz"}:
                raise ValueError("Historical request ID does not match its trace filename")
            trace = _provenance(root, trace_source, export_kind="trace")
            command = trace["command"]
            if trace["request_id"] != identifier or command["request_id"] != identifier or command["strategy"] != role or command["target_rad"] != fixture["target_rad"]:
                raise ValueError("Historical request ID, strategy or command mismatch")
            if trace["status"] != "failed" or trace["dispatched_goals"] != 0 or not _close_state(trace["initial_state_rad"], fixture["initial_rad"]):
                raise ValueError("Historical request outcome or initial state mismatch")
            if role == "coordinated" and any(trace["planned_clearance"]["minimum"][key] != value for key, value in witness.items()):
                raise ValueError("Historical witness differs from its source trace")
        setup_trajectories = []
        if "setup" in fixture:
            setup = fixture["setup"]
            _keys(setup, {"target_rad", "source_request_id", "source_trace_sha256", "note"}, {"sequence"})
            _state(setup["target_rad"], cell.joints, limits)
            if not isinstance(setup["source_request_id"], str) or not re.fullmatch(r"[0-9a-f]{32}", setup["source_request_id"]):
                raise ValueError("Invalid reference-only setup request ID")
            _digest(setup["source_trace_sha256"])
            if "sequence" in setup:
                sequence = setup["sequence"]
                if not isinstance(sequence, list) or not sequence or len(sequence) > 20:
                    raise ValueError("Expected a bounded ordered historical setup sequence")
                seen = set()
                for step in sequence:
                    _keys(step, {"target_rad", "source_request_id", "source", "source_trace_sha256"})
                    _state(step["target_rad"], cell.joints, limits)
                    identifier = step["source_request_id"]
                    if not isinstance(identifier, str) or not re.fullmatch(r"[0-9a-f]{32}", identifier) or identifier in seen:
                        raise ValueError("Invalid or repeated setup sequence request ID")
                    seen.add(identifier)
                    if Path(step["source"]["path"]).name != identifier + ".json.gz":
                        raise ValueError("Setup request ID does not match its compressed trace filename")
                    trace = _provenance(root, step["source"], uncompressed_sha256=step["source_trace_sha256"], export_kind="setup")
                    command = trace["command"]
                    if (trace["request_id"] != identifier or command["request_id"] != identifier
                            or trace["status"] != "succeeded" or command["strategy"] != "coordinated"
                            or command["target_rad"] != step["target_rad"]):
                        raise ValueError("Historical setup sequence outcome or command mismatch")
                    trajectories = trace["trajectories"]
                    if not isinstance(trajectories, list) or len(trajectories) != 1:
                        raise ValueError("Historical setup must contain one complete both-arm trajectory")
                    curve = Spline(**trajectories[0])
                    if set(curve.names) != set(cell.joints):
                        raise ValueError("Historical setup trajectory must cover every workcell joint")
                    start = dict(zip(curve.names, curve.points[0]["p"]))
                    endpoint = dict(zip(curve.names, curve.points[-1]["p"]))
                    _state(start, cell.joints, limits)
                    _state(endpoint, cell.joints, limits)
                    if not _close_state(start, trace["initial_state_rad"]):
                        raise ValueError("Historical setup trajectory start differs from its source state")
                    if max(abs(endpoint[name] - step["target_rad"][name]) for name in cell.joints) > 0.003 + 1e-9:
                        raise ValueError("Historical setup trajectory endpoint exceeds the declared goal tolerance")
                    setup_trajectories.append(dict(
                        trajectory=deepcopy(trajectories[0]),
                        source=dict(request_id=identifier, trace_file=step["source"]["path"],
                                    trace_compressed_sha256=step["source"]["sha256"],
                                    trace_sha256=step["source_trace_sha256"])))
                if (sequence[-1]["source_request_id"] != setup["source_request_id"]
                        or sequence[-1]["source_trace_sha256"] != setup["source_trace_sha256"]
                        or sequence[-1]["target_rad"] != setup["target_rad"]):
                    raise ValueError("Historical setup sequence endpoint reference mismatch")
        comparison = _geometry(root, cell, manifest, historical, source["geometry"])
        result = deepcopy(fixture)
        result.update(_historical_session=deepcopy(session), _fixture_sha256=_sha(raw), _model_comparison=comparison,
                      _setup_trajectories=setup_trajectories)
        return result
    except (KeyError, TypeError, OSError, OverflowError, ET.ParseError) as error:
        raise ValueError("Invalid motion reproduction: " + str(error)) from error


def replay_witness(fixture, evaluator, threshold_m):
    """Replay endpoints and one historical sample, never the unavailable full path."""
    _number(threshold_m, positive=True)
    if not math.isclose(threshold_m, fixture["expected"]["threshold_m"], rel_tol=0, abs_tol=1e-12):
        raise ValueError("Reproduction threshold mismatch")
    expected = fixture["_historical_session"]["evaluator"]
    current = evaluator.metadata()
    if any(current.get(name) != expected[name] for name in ("engine", "source_sha256")):
        raise ValueError("Native evaluator engine or source SHA-256 mismatch")
    _digest(current["library_sha256"])
    endpoints = measure_endpoints(fixture["initial_rad"], fixture["target_rad"], evaluator, threshold_m)
    witness = fixture["witness"]
    replayed = deepcopy(evaluator.at(witness["state_rad"]))
    _number(replayed["distance_m"])
    difference = abs(replayed["distance_m"] - witness["distance_m"])
    if endpoints["endpoint_violations"] or replayed["distance_m"] >= threshold_m or difference > 1e-8 or replayed["bodies"] != witness["bodies"]:
        raise ValueError("Historical intermediate witness geometry did not reproduce")
    return dict(reproduced=True, threshold_m=threshold_m,
                scope="Historical witness sample only; original rejected full trajectory was not saved. Endpoint geometry is separate from MoveIt validity.",
                endpoint_clearance=endpoints,
                witness=dict(historical_time_sec=witness["time_sec"], historical_distance_m=witness["distance_m"], replayed=replayed, distance_error_m=difference),
                model_comparison=deepcopy(fixture["_model_comparison"]),
                library_sha256=dict(expected=expected["library_sha256"], current=current["library_sha256"], matches=current["library_sha256"] == expected["library_sha256"]),
                source=deepcopy(fixture["source"]))
