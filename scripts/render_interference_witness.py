"""Render a counterfactual planning collision; this is not an executed ROS frame."""
import argparse
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    if report.get("status") != "passed" or not report.get(
        "parallel_dispatch_gate_passed"
    ):
        raise ValueError("A passed interference report is required")
    cell = load_workcell(report["workcell_id"])
    check_workcell(cell)
    if file_sha256(cell.output / "source.json") != report["model_manifest_sha256"]:
        raise ValueError("Report/model mismatch")
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    from PIL import Image

    model = mujoco.MjModel.from_xml_path(str(cell.output / "cell.xml"))
    data = mujoco.MjData(model)
    witness = report["parallel_reference"]["first_collision"]
    for name, value in witness["state_rad"].items():
        data.qpos[model.joint(name).qposadr[0]] = value
    mujoco.mj_forward(model, data)
    pairs = [
        dict(
            bodies=[
                model.body(model.geom_bodyid[c.geom[0]]).name,
                model.body(model.geom_bodyid[c.geom[1]]).name,
            ],
            distance_m=float(c.dist),
        )
        for c in data.contact
    ]
    if not any(
        (p["bodies"][0].startswith("left_") and p["bodies"][1].startswith("right_"))
        or (p["bodies"][0].startswith("right_") and p["bodies"][1].startswith("left_"))
        for p in pairs
    ):
        raise ValueError("No MuJoCo inter-arm contact at the recorded MoveIt witness")
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, 0.95]
    camera.distance = 2.5
    camera.azimuth = 90
    camera.elevation = -25
    model.vis.global_.offwidth = 1280
    model.vis.global_.offheight = 720
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / "reference-collision.png"
    with mujoco.Renderer(model, height=720, width=1280) as renderer:
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(output)
    (args.output_dir / "reference-collision.json").write_text(
        json.dumps(
            dict(
                kind="Counterfactual state of rejected independent plans; no physics steps or ROS commands executed",
                source_run_id=report["run_id"],
                source_report_sha256=file_sha256(args.report),
                mujoco_version=mujoco.__version__,
                witness=witness,
                mujoco_contacts=pairs,
                image_sha256=file_sha256(output),
            ),
            ensure_ascii=False,
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
