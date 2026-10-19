"""Content checks for compositions of already-verified single-robot builds."""

import json
from pathlib import Path
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.profiles import check_generated_model

OUTPUTS = (
    "cell.xml",
    "cell.urdf",
    "cell.srdf",
    "controllers.yaml",
    "moveit.yaml",
    "moveit.rviz",
)


def snapshot(cell):
    files = {}
    paths = [
        cell.root / "config/workcells" / f"{cell.id}.yaml",
        cell.root / "config/controller_defaults.yaml",
    ]
    paths += [
        cell.root / "src/mujoco_ros2_manipulation/workcells" / name
        for name in ("config.py", "prepare.py", "integrity.py")
    ]
    paths += [cell.root / "packages/ur5e_moveit_config/config/moveit.rviz"]
    for inst in cell.instances:
        check_generated_model(inst.profile)
        paths += [
            inst.profile.output / "source.json",
            inst.profile.scene_path,
            inst.profile.urdf_path,
        ]
    for path in paths:
        files[path.relative_to(cell.root).as_posix()] = file_sha256(path)
    return files


def check_workcell(cell):
    try:
        report = json.loads((cell.output / "source.json").read_text())
        if (
            report["schema_version"] != 1
            or report["workcell_sha256"] != cell.fingerprint
            or report["inputs"] != snapshot(cell)
        ):
            raise ValueError("Workcell inputs changed")
        if set(report["outputs"]) != set(OUTPUTS):
            raise ValueError("Incomplete workcell outputs")
        for name in OUTPUTS:
            if file_sha256(cell.output / name) != report["outputs"][name]:
                raise ValueError("Changed output: " + name)
        return report
    except (ValueError, KeyError, TypeError, OSError) as error:
        raise ValueError(
            f"{cell.id}: workcell integrity failed ({error}); run prepare_workcell.py --workcell {cell.id}"
        ) from error
