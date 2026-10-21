"""Reproducible local joint goals; collision outcomes are retained, never prefiltered."""
import hashlib
import json
import math
import random

from .motion_request import STRATEGIES, command, validate_command


def workload_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":")).encode()).hexdigest()


def make_workload(cell, limits, *, seed=20261021, count=5, radius=0.25):
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError("Seed must be a uint32")
    if type(count) is not int or not 1 <= count <= 200:
        raise ValueError("Case count must be 1..200")
    if type(radius) not in (int, float) or not math.isfinite(radius) or not 0.05 <= radius <= 1:
        raise ValueError("Radius must be 0.05..1 radians")
    home = dict(zip(cell.joints, cell.home))
    validate_command(command(home), limits)
    rng = random.Random(seed)
    cases = []
    for index in range(count):
        target = {}
        for name, center in home.items():
            low, high = max(limits[name]["lower"], center - radius), min(limits[name]["upper"], center + radius)
            if low >= high:
                raise ValueError("Empty random-goal interval")
            target[name] = rng.uniform(low, high)
        validate_command(command(target), limits)
        cases.append(dict(case_id=f"case-{index + 1:04d}", target_rad=target,
                          strategy_order=list(STRATEGIES if index % 2 == 0 else reversed(STRATEGIES))))
    return dict(schema_version=1, workcell=cell.id, workcell_sha256=cell.fingerprint,
                seed=seed, count=count, radius_rad=float(radius), initial_rad=home,
                joint_limits=limits, cases=cases,
                sampling="random.Random(seed), uniform intersection of home +/- radius and each joint limit; no collision or planner-result filtering",
                order="Alternating strategy order by case; resets excluded from scored requests")


def check_workload(value, cell, limits):
    expected = make_workload(cell, limits, seed=value["seed"], count=value["count"], radius=value["radius_rad"])
    if value != expected:
        raise ValueError("Workload differs from the recorded seed, limits, model or strategy order")
    return value
