"""Kinematics analysis of a recorded fixed-obstacle contact demo."""
from pathlib import Path
import hashlib
import json
import mujoco as m
import numpy as np
import argparse

parser = argparse.ArgumentParser(
    description="Recompute wrist motion from recorded joint states; runtime contact evidence stays in physics.json"
)
parser.add_argument("--report", type=Path, required=True)
parser.add_argument("--fixture", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
r = json.loads(args.report.read_text())
fixture = args.fixture
if r.get("status") != "passed" or not r.get("natural_fixed_obstacle_contact"):
    raise ValueError("Expected a successful fixed-obstacle demo report")
if hashlib.sha256(fixture.read_bytes()).hexdigest() != r["contact_test_model_sha256"]:
    raise ValueError("Fixture does not match the verified model hash")
model = m.MjModel.from_xml_path(str(fixture))
d = m.MjData(model)
lookup = {
    m.mj_id2name(model, m.mjtObj.mjOBJ_JOINT, j): int(model.jnt_qposadr[j])
    for j in range(model.njnt)
}
obstacle = m.mj_name2id(model, m.mjtObj.mjOBJ_GEOM, "demo_obstacle_geom")
measures = {}
for arm, body in [("left", "left_wrist3_link"), ("right", "right_wrist_3_link")]:
    bid = m.mj_name2id(model, m.mjtObj.mjOBJ_BODY, body)
    points = []
    for state in (
        [r["initial_positions_rad"]]
        + [s["positions_rad"] for s in r["motion_samples"]]
        + [r["stopped_positions_rad"]]
    ):
        for name, value in state.items():
            d.qpos[lookup[name]] = value
        m.mj_forward(model, d)
        points.append(d.xpos[bid].copy())
    measures[arm] = dict(
        body=body,
        initial_xyz_m=points[0].tolist(),
        stopped_xyz_m=points[-1].tolist(),
        straight_line_displacement_m=float(np.linalg.norm(points[-1] - points[0])),
        sampled_path_length_m=float(
            np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
        ),
    )
for name, value in r["initial_positions_rad"].items():
    d.qpos[lookup[name]] = value
m.mj_forward(model, d)
gaps = []
for g in range(model.ngeom):
    name = m.mj_id2name(model, m.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g])) or ""
    if name.startswith(("left_", "right_")) and model.geom_contype[g]:
        gaps.append(float(m.mj_geomDistance(model, d, obstacle, g, 10, None)))
a = dict(
    run_id=r["run_id"],
    kind="Kinematics of recorded joint-state samples using Python MuJoCo "
    + m.__version__,
    runtime_contact_authority="Native MuJoCo "
    + r["physics"]["mujoco_version"]
    + " physics.json",
    initial_robot_obstacle_minimum_gap_m=min(gaps),
    initial_contacts=int(d.ncon),
    endpoints=measures,
)
args.output.write_text(json.dumps(a, indent=2) + "\n")
print(json.dumps(a, indent=2))
