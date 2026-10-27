"""Exact compiled body/geometry ownership, separate from display scene schemas.

Robot ownership comes only from explicitly declared compiled body roots and the
compiled parent tree. Names never imply a robot, tool, TCP or collision role.
Stable IDs are scoped to the current scene digest; changing a scene invalidates
every old selection. No engine is imported by this module.
"""
from copy import deepcopy
import re


MAX_BODIES = 10_000
MAX_GEOMS = 10_000
MAX_ROBOTS = 32
MAX_HIERARCHY_BYTES = 16 * 1024 * 1024
KIND = "compiled-object-hierarchy-v1"
ORIGIN = "mujoco_compiled_body_parentid_geom_bodyid"
SEMANTICS = "Compiled ownership only; tool, TCP, sensor and collision roles are unknown"
_SHA = re.compile(r"[0-9a-f]{64}")
_INSTANCE = re.compile(r"[a-z][a-z0-9_]{0,127}")
_NAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,255}")
_LIMITS = dict(bodies=MAX_BODIES, geoms=MAX_GEOMS, robots=MAX_ROBOTS)


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def normalize_robot_roots(value):
    """Freeze trusted (instance ID, exact root body name) declarations."""
    _require(type(value) in (tuple, list) and len(value) <= MAX_ROBOTS,
             "Invalid bounded robot roots")
    roots = []
    for entry in value:
        if type(entry) is dict:
            _require(set(entry) == {"instance_id", "root_body_name"}, "Invalid robot root declaration")
            instance, name = entry["instance_id"], entry["root_body_name"]
        else:
            _require(type(entry) in (tuple, list) and len(entry) == 2, "Invalid robot root declaration")
            instance, name = entry
        _require(type(instance) is str and _INSTANCE.fullmatch(instance) is not None
                 and type(name) is str and _NAME.fullmatch(name) is not None,
                 "Invalid robot root identity")
        roots.append((instance, name))
    _require(len({entry[0] for entry in roots}) == len(roots)
             and len({entry[1] for entry in roots}) == len(roots), "Duplicate robot root declaration")
    return tuple(roots)


def _name(value):
    _require(type(value) is str, "Invalid compiled object name")
    # A malicious/accidental filename or free-form secret is not a public label.
    return value if value == "" or _NAME.fullmatch(value) else None


def _stable(kind, scene, index):
    return kind + ":" + scene + ":" + str(index)


def _owners(parents, roots):
    _require(1 <= len(parents) <= MAX_BODIES and parents[0] is None,
             "Invalid compiled world body")
    by_root = {}
    for root in roots:
        index = root["root_body_id"]
        _require(type(index) is int and 0 < index < len(parents) and index not in by_root,
                 "Invalid compiled robot root body")
        by_root[index] = root["instance_id"]
    owners = []
    for index, parent in enumerate(parents):
        if index:
            _require(type(parent) is int and 0 <= parent < index,
                     "Compiled body parents must precede their children")
        inherited = None if parent is None else owners[parent]
        _require(not (index in by_root and inherited is not None), "Overlapping robot root subtrees")
        owners.append(by_root.get(index, inherited))
    return owners


def _identities(manifest):
    _require(type(manifest) is dict and type(manifest.get("geoms")) is list,
             "Expected current scene manifest")
    _require(type(manifest.get("schema_version")) is int, "Invalid hierarchy scene schema")
    scene, model = manifest.get("scene_sha256"), manifest.get("model_sha256")
    _require(all(type(value) is str and _SHA.fullmatch(value) for value in (scene, model)),
             "Invalid hierarchy scene identity")
    if manifest.get("schema_version") == 1:
        source, compiled = scene, manifest.get("compiled_model_sha256")
    else:
        provenance = manifest.get("provenance", {})
        _require(manifest.get("schema_version") == 2 and type(provenance) is dict
                 and provenance.get("adapter") == "mujoco-compiled"
                 and type(provenance.get("engine")) is dict,
                 "Hierarchy requires an exact compiled-model display adapter")
        source, compiled = provenance.get("source_sha256"), provenance["engine"].get("compiled_sha256")
    _require(all(type(value) is str and _SHA.fullmatch(value) for value in (source, compiled)),
             "Invalid hierarchy compiled identity")
    _require(len(manifest["geoms"]) <= MAX_GEOMS and all(type(geom) is dict
        and type(geom.get("id")) is int and geom["id"] == index
        and type(geom.get("name")) is str for index, geom in enumerate(manifest["geoms"])),
        "Invalid exact compiled geometry catalog")
    return scene, model, source, compiled


def compiled_object_hierarchy(scene, robot_roots=()):
    """Extract from the SAME CompiledScene MjModel that produced its manifest."""
    roots = normalize_robot_roots(robot_roots)
    manifest = scene.manifest()
    scene_sha, model_sha, source_sha, compiled_sha = _identities(manifest)
    _require(manifest.get("schema_version") == 1 and scene.scene_sha256 == scene_sha,
             "Expected the original compiled scene")
    model, mj = scene._model, scene._mj
    _require(1 <= int(model.nbody) <= MAX_BODIES and int(model.ngeom) == len(manifest["geoms"]),
             "Compiled hierarchy exceeds limits or differs from scene")
    names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, index) or ""
             for index in range(int(model.nbody))]
    declared = []
    for instance, name in roots:
        matches = [index for index, current in enumerate(names) if current == name]
        _require(len(matches) == 1 and matches[0] > 0, "Declared robot root is absent from the compiled model")
        declared.append(dict(instance_id=instance, root_body_name=name, root_body_id=matches[0]))
    parents = [None] + [int(model.body_parentid[index]) for index in range(1, int(model.nbody))]
    owners = _owners(parents, declared)
    bodies = [dict(id=_stable("body", scene_sha, index), body_id=index,
        parent_body_id=parent, parent_id=None if parent is None else _stable("body", scene_sha, parent),
        name=_name(names[index]), name_redacted=_name(names[index]) is None,
        robot_instance_id=owners[index], role="world" if index == 0 else "robot_body" if owners[index] else "unassigned_body",
        tool_role="unknown") for index, parent in enumerate(parents)]
    geoms = []
    for index, descriptor in enumerate(manifest["geoms"]):
        body = int(model.geom_bodyid[index])
        _require(0 <= body < len(bodies), "Compiled geometry refers to an invalid body")
        actual_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, index) or ""
        _require(actual_name == descriptor["name"], "Compiled geometry name/order differs from scene")
        geoms.append(dict(id=_stable("geom", scene_sha, index), geom_id=index, body_id=body,
            body_ref=_stable("body", scene_sha, body), name=_name(actual_name),
            name_redacted=_name(actual_name) is None, robot_instance_id=owners[body],
            role="robot_geometry" if owners[body] else "unassigned_geometry", tool_role="unknown"))
    value = dict(schema_version=1, kind=KIND, scene_sha256=scene_sha, model_sha256=model_sha,
        source_scene_sha256=source_sha, compiled_model_sha256=compiled_sha,
        origin=ORIGIN, robot_roots=declared, bodies=bodies, geoms=geoms,
        counts=dict(bodies=len(bodies), geoms=len(geoms), robots=len(declared)),
        limits=dict(_LIMITS), semantics=SEMANTICS, read_only=True)
    return validate_object_hierarchy(value, manifest, robot_roots=roots)


def validate_object_hierarchy(value, manifest, *, robot_roots=None):
    """Validate complete bounded metadata and exact current geometry correspondence."""
    scene, model, source, compiled = _identities(manifest)
    _require(type(value) is dict and set(value) == {"schema_version", "kind", "scene_sha256", "model_sha256",
        "source_scene_sha256", "compiled_model_sha256", "origin", "robot_roots", "bodies", "geoms",
        "counts", "limits", "semantics", "read_only"}, "Invalid hierarchy fields")
    _require(type(value["schema_version"]) is int and value["schema_version"] == 1
        and value["kind"] == KIND and value["origin"] == ORIGIN and value["semantics"] == SEMANTICS
        and value["read_only"] is True and value["limits"] == _LIMITS
        and (value["scene_sha256"], value["model_sha256"], value["source_scene_sha256"], value["compiled_model_sha256"])
            == (scene, model, source, compiled), "Hierarchy belongs to another compiled scene")
    _require(type(value["limits"]) is dict and all(type(item) is int for item in value["limits"].values()),
             "Invalid exact hierarchy limits")
    bodies, geoms, roots = value["bodies"], value["geoms"], value["robot_roots"]
    _require(type(bodies) is list and 1 <= len(bodies) <= MAX_BODIES
        and type(geoms) is list and len(geoms) == len(manifest["geoms"])
        and type(roots) is list and len(roots) <= MAX_ROBOTS, "Invalid bounded hierarchy catalog")
    for root in roots:
        _require(type(root) is dict and set(root) == {"instance_id", "root_body_name", "root_body_id"},
                 "Invalid compiled root metadata")
    normalized = normalize_robot_roots([(root["instance_id"], root["root_body_name"]) for root in roots])
    if robot_roots is not None:
        _require(normalized == normalize_robot_roots(robot_roots), "Hierarchy robot declarations changed")
    for body in bodies:
        _require(type(body) is dict and set(body) == {"id", "body_id", "parent_body_id", "parent_id", "name",
            "name_redacted", "robot_instance_id", "role", "tool_role"}, "Invalid compiled body metadata")
    parents = [body["parent_body_id"] for body in bodies]
    owners = _owners(parents, roots)
    _require(type(value["counts"]) is dict and all(type(item) is int for item in value["counts"].values())
        and value["counts"] == dict(bodies=len(bodies), geoms=len(geoms), robots=len(roots)),
             "Hierarchy catalog count mismatch")
    for index, body in enumerate(bodies):
        parent = parents[index]
        _require(type(body["body_id"]) is int and body["body_id"] == index
            and body["id"] == _stable("body", scene, index)
            and body["parent_id"] == (None if parent is None else _stable("body", scene, parent))
            and body["robot_instance_id"] == owners[index]
            and body["role"] == ("world" if index == 0 else "robot_body" if owners[index] else "unassigned_body")
            and body["tool_role"] == "unknown", "Body parent/ownership correspondence mismatch")
        _require(type(body["name_redacted"]) is bool
            and ((body["name"] is None and body["name_redacted"])
                or (type(body["name"]) is str and _name(body["name"]) == body["name"] and not body["name_redacted"])),
            "Invalid public body name")
    for root in roots:
        _require(bodies[root["root_body_id"]]["name"] == root["root_body_name"], "Root body name/ID mismatch")
    for index, geom in enumerate(geoms):
        _require(type(geom) is dict and set(geom) == {"id", "geom_id", "body_id", "body_ref", "name",
            "name_redacted", "robot_instance_id", "role", "tool_role"}, "Invalid compiled geometry metadata")
        body = geom["body_id"]
        _require(type(body) is int and 0 <= body < len(bodies) and type(geom["geom_id"]) is int
            and geom["geom_id"] == index and geom["id"] == _stable("geom", scene, index)
            and geom["body_ref"] == _stable("body", scene, body)
            and geom["robot_instance_id"] == owners[body]
            and geom["role"] == ("robot_geometry" if owners[body] else "unassigned_geometry")
            and geom["tool_role"] == "unknown" and type(geom["name_redacted"]) is bool
            and geom["name"] == _name(manifest["geoms"][index]["name"])
            and geom["name_redacted"] == (geom["name"] is None), "Geometry body/scene correspondence mismatch")
    return deepcopy(value)


def from_compiled_hierarchy(value, manifest):
    """Rebind exact IDs to a display digest, retaining the original source digest."""
    scene, model, source, compiled = _identities(manifest)
    native_manifest = dict(schema_version=1, scene_sha256=source, model_sha256=model,
                           compiled_model_sha256=compiled, geoms=manifest["geoms"])
    result = validate_object_hierarchy(value, native_manifest)
    result["scene_sha256"] = scene
    for body in result["bodies"]:
        body["id"] = _stable("body", scene, body["body_id"])
        body["parent_id"] = None if body["parent_body_id"] is None else _stable("body", scene, body["parent_body_id"])
    for geom in result["geoms"]:
        geom["id"] = _stable("geom", scene, geom["geom_id"])
        geom["body_ref"] = _stable("body", scene, geom["body_id"])
    return validate_object_hierarchy(result, manifest)
