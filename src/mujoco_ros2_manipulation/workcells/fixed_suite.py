"""Fixed public joint-input suite; no solver, ROS, geometry engine or execution."""
from collections.abc import Mapping, Sequence
from copy import deepcopy
import gzip
import hashlib
import json
import math
from pathlib import Path
import re

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.timing import load_policy


SOURCE_SHA256 = {
    "docs/reports/2026-10-21-random-motion/evidence.json.gz":
        "de54c8236464dfd280adaa90379a9e6433687e885c2513e300c9b0dd29283f45",
    "docs/timing_coordination/bundle.json.gz":
        "6d48d7e9da09f83e1cfcfb17a95bc908f3a9fda3513a02bf824733ba24fd8095",
    "docs/reports/2026-10-21-request-timing/evidence.json":
        "3eb2adf6464b3af9ac2f774711207ced674a4e8998237371416b0b5018deea90",
    "config/reproductions/home_return_clearance.json":
        "d81251326239aa2edcb5e37fa4448f6ab13363f7e6229d1a213fd403fb6d6f84",
}
RANDOM_SOURCE, CROSSING_SOURCE, INVALID_SOURCE, HOME_SOURCE = SOURCE_SHA256
BASE_ID = "fr5_ur5e"
POST_ID = "fr5_ur5e_eval_post"
POLICY = dict(min_inter_arm_distance_m=.02, tracking_reserve_m=.01, sample_dt_sec=.01,
              delay_step_sec=.1, max_delay_sec=4.0)
POLICY_NAMES = {BASE_ID:"dual_arm_timing", POST_ID:"eval_post_timing"}
POLICY_CASES = {BASE_ID:"dual_arm_crossing", POST_ID:"eval_post"}
VARIANTS = ({"id": "single", "max_plan_candidates": 1},
            {"id": "bounded", "max_plan_candidates": 12})
CASE_IDS = ("local_small", *(f"random_{i:04d}" for i in range(1, 6)),
            "crossing", "home_return", "environment_post", "invalid_goal")
CASE_KEYS = {"case_id", "label", "category", "workcell", "policy_name", "initial_rad",
             "target_rad", "expectation", "preparation", "provenance"}
POST_GEOMETRY = dict(shape="box", size=[0.16, 0.16, 1.0], xyz=[-0.45, 0.72, 0.5],
                     rpy=[0, 0, 0], rgba=[1.0, 0.15, 0.02, 1.0])


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _names(cell):
    names = list(cell.joints)
    if (len(names) != 12 or any(not isinstance(n, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", n) for n in names)
            or len(names) != len(set(names)) or len(cell.home) != len(names)
            or not all(_finite(q) for q in cell.home)):
        raise ValueError("Fixed suite requires twelve unique named finite workcell joints")
    return names


def _cells(base_cell, obstacle_cell):
    if base_cell.id != BASE_ID or obstacle_cell.id != POST_ID:
        raise ValueError("Fixed suite requires base and evaluation-post workcell IDs")
    names = _names(base_cell)
    if _names(obstacle_cell) != names or list(obstacle_cell.home) != list(base_cell.home):
        raise ValueError("Evaluation-post workcell must retain base joint order and nominal home")
    base, post = deepcopy(base_cell.data), deepcopy(obstacle_cell.data)
    for data in (base, post):
        data.pop("id", None)
        data.pop("label", None)
    objects = post.get("objects", [])
    if (len(objects) != len(base.get("objects", [])) + 1
            or objects[:-1] != base.get("objects", [])):
        raise ValueError("Evaluation-post workcell must retain base objects and append one fixed box")
    extra = objects[-1]
    if (not isinstance(extra, dict) or set(extra) != set(POST_GEOMETRY) | {"name"}
            or not isinstance(extra["name"], str) or not re.fullmatch(r"[a-z][a-z0-9_]*", extra["name"])
            or extra["name"] in {obj["name"] for obj in objects[:-1]}
            or {key:extra[key] for key in POST_GEOMETRY} != POST_GEOMETRY):
        raise ValueError("Evaluation-post box differs from the defined contact-demo geometry")
    post["objects"] = post["objects"][:-1]
    if post != base:
        raise ValueError("Evaluation-post workcell changed base robots, placement or physics")
    if ([instance.id for instance in base_cell.instances] != [instance.id for instance in obstacle_cell.instances]
            or [instance.joints for instance in base_cell.instances] != [instance.joints for instance in obstacle_cell.instances]
            or [instance.profile.fingerprint for instance in base_cell.instances]
                != [instance.profile.fingerprint for instance in obstacle_cell.instances]
            or [instance.data for instance in base_cell.instances] != [instance.data for instance in obstacle_cell.instances]
            or len(base_cell.instances) != 2):
        raise ValueError("Evaluation-post workcell changed arm ownership")
    return {base_cell.id:base_cell, obstacle_cell.id:obstacle_cell}, deepcopy(extra)


def _read_sources(root):
    sources = {}
    for relative, expected in SOURCE_SHA256.items():
        try:
            raw = (Path(root) / relative).read_bytes()
        except OSError as error:
            raise ValueError("Missing fixed-suite source: " + relative) from error
        if hashlib.sha256(raw).hexdigest() != expected:
            raise ValueError("Fixed-suite source SHA256 mismatch: " + relative)
        try:
            sources[relative] = json.loads(gzip.decompress(raw) if relative.endswith(".gz") else raw)
        except (ValueError, OSError) as error:
            raise ValueError("Invalid fixed-suite source: " + relative) from error
    return sources


def _refs(path, *selectors):
    return dict(kind="public_source", refs=[dict(path=path, sha256=SOURCE_SHA256[path], selector=selector)
        for selector in selectors])


def _case(case_id, label, category, cell, initial, target, provenance, *, preparation=None, invalid=False):
    return dict(case_id=case_id, label=label, category=category, workcell=cell.id,
        policy_name=POLICY_NAMES[cell.id],
        initial_rad=deepcopy(initial), target_rad=deepcopy(target),
        expectation="invalid_goal" if invalid else "valid_goal",
        preparation=deepcopy(preparation) if preparation is not None else dict(kind="joint_goal", targets=[deepcopy(initial)]),
        provenance=deepcopy(provenance))


def make_suite(base_cell, obstacle_cell, *, root=ROOT):
    """Construct ten declared cases from four byte-pinned public files and constants.

    ``valid_goal`` states the case's intent, not a promise that a solver will
    succeed. New geometry and MoveIt endpoint checks belong to the live runner.
    The invalid-goal case is reported separately from motion success rates.
    No private runtime path, historical path reuse or result-based selection is
    used. Source files are read and checked on every construction.
    """
    cells, post_object = _cells(base_cell, obstacle_cell)
    sources = _read_sources(root)
    home = dict(zip(base_cell.joints, base_cell.home))
    small = dict(home)
    for instance in base_cell.instances:
        small[instance.joints[0]] += .12
    cases = [_case("local_small", "근거리 양팔 목표", "local", base_cell, home, small,
        dict(kind="defined", parameters=dict(initial="base_workcell_nominal_home",
            first_joint_offsets_rad={instance.joints[0]:.12 for instance in base_cell.instances})))]
    workload = sources[RANDOM_SOURCE]["workload"]
    for index, source_case in enumerate(workload["cases"]):
        if source_case["case_id"] != f"case-{index+1:04d}":
            raise ValueError("Public random cases are not in their original order")
        cases.append(_case(f"random_{index+1:04d}", f"기존 고정 랜덤 목표 {index+1}", "fixed_random",
            base_cell, workload["initial_rad"], source_case["target_rad"],
            _refs(RANDOM_SOURCE, "workload.initial_rad", f"workload.cases[{index}].target_rad")))
    crossing = sources[CROSSING_SOURCE]
    cases.append(_case("crossing", "양팔 교차", "crossing", base_cell, crossing["initial_rad"], crossing["goal_rad"],
        _refs(CROSSING_SOURCE, "initial_rad", "goal_rad")))
    fixture = sources[HOME_SOURCE]
    cases.append(_case("home_return", "고정 시작 자세에서 홈 복귀", "home_return", base_cell,
        fixture["initial_rad"], fixture["target_rad"], _refs(HOME_SOURCE, "initial_rad", "target_rad", "setup"),
        preparation=dict(kind="home_fixture", fixture=HOME_SOURCE)))
    post_target = dict(home)
    offsets = {instance.joints[0]:offset for instance,offset in zip(base_cell.instances, (2.0, -2.0))}
    for name, offset in offsets.items():
        post_target[name] += offset
    cases.append(_case("environment_post", "고정 기둥을 포함한 환경", "environment", obstacle_cell, home, post_target,
        dict(kind="defined", parameters=dict(initial="base_workcell_nominal_home", first_joint_offsets_rad=offsets,
            fixed_obstacle=post_object))))
    invalid = sources[INVALID_SOURCE]["requests"][6]
    cases.append(_case("invalid_goal", "공개 기록의 충돌 목표 거부", "invalid_goal", base_cell, home,
        invalid["command"]["target_rad"], _refs(INVALID_SOURCE, "requests[6].command.target_rad"), invalid=True))
    suite = dict(schema_version=1, id="fixed-motion-v1", compute_budget_wall_sec=5.0, initial_tolerance_rad=.0015,
        policy=deepcopy(POLICY), variants=deepcopy(list(VARIANTS)),
        workcell_fingerprints={name:cell.fingerprint for name,cell in cells.items()}, cases=cases)
    _catalog(suite)
    return suite


def _catalog(data):
    if (not isinstance(data, dict) or type(data.get("schema_version")) is not int or data["schema_version"] != 1
            or data.get("id") != "fixed-motion-v1"
            or type(data.get("compute_budget_wall_sec")) is not float or data["compute_budget_wall_sec"] != 5.0
            or type(data.get("initial_tolerance_rad")) is not float or data["initial_tolerance_rad"] != .0015
            or data.get("variants") != list(VARIANTS)
            or any(type(v.get("max_plan_candidates")) is not int for v in data["variants"])):
        raise ValueError("Invalid fixed-suite identity, variant, budget or tolerance")
    policy = data.get("policy")
    if (not isinstance(policy, dict) or set(policy) != set(POLICY)
            or any(type(policy[key]) is not float or policy[key] != POLICY[key] for key in POLICY)):
        raise ValueError("Fixed-suite policy must match the five declared finite numeric constants")
    cases = data.get("cases")
    if (not isinstance(cases, list) or len(cases) != len(CASE_IDS)
            or any(not isinstance(case, dict) or set(case) != CASE_KEYS for case in cases)
            or tuple(case["case_id"] for case in cases) != CASE_IDS):
        raise ValueError("Fixed-suite cases were changed, duplicated, dropped or reordered")


def _state(value, names, limits, role):
    if not isinstance(value, Mapping) or set(value) != set(names):
        raise ValueError(role + " must cover every named joint exactly once")
    for name in names:
        q = value[name]
        if not _finite(q):
            raise ValueError(role + " requires finite numeric joint positions")
        if not limits[name]["lower"] <= q <= limits[name]["upper"]:
            raise ValueError(role + " exceeds metadata joint limits: " + name)


def check_suite(data, base_cell, obstacle_cell, metadata_by_id, *, root=ROOT):
    """Return a detached validated suite, including all cases and source checks.

    Metadata identity, complete finite joint limits and actual per-cell policy
    contents are checked separately from byte-pinned sources and exact factory
    equality. Policies are read through each cell's root, while ``root`` only
    overrides the four public source files. This does not perform
    collision checks or require that any valid-goal planner succeeds.
    """
    cells, _ = _cells(base_cell, obstacle_cell)
    expected = make_suite(base_cell, obstacle_cell, root=root)
    _catalog(data)
    if not isinstance(metadata_by_id, Mapping) or set(metadata_by_id) != set(cells):
        raise ValueError("Expected metadata for exactly both suite workcells")
    for name, cell in cells.items():
        meta = metadata_by_id[name]
        names = _names(cell)
        if (not isinstance(meta, Mapping) or type(meta.get("schema_version")) is not int or meta["schema_version"] != 1
                or meta.get("workcell_sha256") != cell.fingerprint or meta.get("joints") != names
                or meta.get("home_rad") != list(cell.home)):
            raise ValueError("Suite workcell metadata identity mismatch: " + name)
        limits = meta.get("joint_limits")
        if not isinstance(limits, Mapping) or set(limits) != set(names):
            raise ValueError("Incomplete named metadata joint limits")
        for bound in limits.values():
            if (not isinstance(bound, Mapping) or not _finite(bound.get("lower")) or not _finite(bound.get("upper"))
                    or not bound["lower"] < bound["upper"]):
                raise ValueError("Expected finite ordered metadata joint limits")
        _state(dict(zip(names, meta["home_rad"])), names, limits, "metadata nominal home")
        try:
            _, actual_policy = load_policy(cell, policy_name=POLICY_NAMES[name])
        except (OSError, ValueError) as error:
            raise ValueError("Fixed-suite runtime policy cannot be validated: " + POLICY_NAMES[name]) from error
        expected_policy = dict(schema_version=1, workcell=name, case=POLICY_CASES[name], **POLICY)
        if (actual_policy != expected_policy or type(actual_policy.get("schema_version")) is not int
                or any(type(actual_policy.get(key)) is not float for key in POLICY)):
            raise ValueError("Fixed-suite runtime policy differs from its declared schema, workcell, case or constants: " + name)
    for case in data["cases"]:
        if case["workcell"] not in cells:
            raise ValueError("Case references an unknown workcell")
        cell = cells[case["workcell"]]
        names, limits = _names(cell), metadata_by_id[cell.id]["joint_limits"]
        for field in ("initial_rad", "target_rad"):
            _state(case[field], names, limits, case["case_id"] + "." + field)
        prep = case["preparation"]
        if not isinstance(prep, dict):
            raise ValueError("Invalid case preparation")
        if prep.get("kind") == "joint_goal" and set(prep) == {"kind", "targets"} and isinstance(prep["targets"], list):
            for state in prep["targets"]:
                _state(state, names, limits, case["case_id"] + ".preparation")
        elif prep != dict(kind="home_fixture", fixture=HOME_SOURCE):
            raise ValueError("Invalid fixed case preparation contract")
    if data != expected:
        raise ValueError("Fixed suite differs from its complete factory cases, sources, model identities or policy")
    return deepcopy(data)


def suite_hash(data):
    """Canonical content SHA256; object-key order does not affect the digest."""
    if not isinstance(data, dict):
        raise ValueError("Expected a suite object")
    try:
        raw = json.dumps(data, sort_keys=True, allow_nan=False, separators=(",", ":")).encode()
    except (TypeError, ValueError) as error:
        raise ValueError("Suite hash requires finite JSON data") from error
    return hashlib.sha256(raw).hexdigest()


def planned_rows(suite, repeats=1, case_ids=None):
    """Build the full row catalog of a checked suite, including unselected cases.

    ``repeat`` is one-based; ``index`` is zero-based across the entire catalog.
    Ordering is repeat, original case, alternating variant. On repeat one case
    zero starts single/bounded; each next case or repeat reverses that order.
    No row has run, even when selected. The runner fills measured outcomes later.
    """
    _catalog(suite)
    if type(repeats) is not int or not 1 <= repeats <= 10:
        raise ValueError("repeats must be an integer from 1 to 10")
    if case_ids is None:
        selected = set(CASE_IDS)
    else:
        if (not isinstance(case_ids, (Sequence, set, frozenset)) or isinstance(case_ids, (str, bytes))
                or any(not isinstance(name, str) for name in case_ids)
                or len(case_ids) != len(set(case_ids)) or not set(case_ids).issubset(CASE_IDS)):
            raise ValueError("case_ids must be unique known case IDs")
        selected = set(case_ids)
    rows = []
    for repeat_index in range(repeats):
        for case_index, case in enumerate(suite["cases"]):
            variants = suite["variants"] if (case_index + repeat_index) % 2 == 0 else reversed(suite["variants"])
            for variant in variants:
                rows.append(dict(case_id=case["case_id"], variant=variant["id"],
                    max_plan_candidates=variant["max_plan_candidates"], repeat=repeat_index+1, index=len(rows),
                    status="not_run", request_id=None, selected=case["case_id"] in selected))
    return rows
