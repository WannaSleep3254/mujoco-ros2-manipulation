"""Validated robot instances, common physics and non-overlapping controllers."""

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import yaml
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.profiles import load_profile, controller_config, _number
from mujoco_ros2_manipulation.scenes import vector


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch("[a-z][a-z0-9_]*", value):
        raise ValueError(f"Invalid workcell identifier: {value}")
    return value


@dataclass(frozen=True)
class Instance:
    data: dict
    profile: object

    @property
    def id(self):
        return self.data["instance"]

    @property
    def prefix(self):
        return self.id + "_"

    @property
    def joints(self):
        return [self.prefix + name for name in self.profile.joints]

    @property
    def controller(self):
        return self.id + "_controller"

    @property
    def group(self):
        return self.id + "_arm"

    @property
    def plan_target(self):
        return self.data.get("verification", {}).get(
            "plan_target_rad", self.profile.data["verification"]["moveit_target_rad"]
        )


@dataclass(frozen=True)
class Workcell:
    root: Path
    data: dict
    instances: tuple

    @property
    def id(self):
        return self.data["id"]

    @property
    def output(self):
        return self.root / "runtime/generated/workcells" / self.id

    @property
    def label(self):
        return self.data["label"]

    @property
    def ros(self):
        return {"planning_group": "both_arms"}

    @property
    def joints(self):
        return [name for inst in self.instances for name in inst.joints]

    @property
    def home(self):
        return [v for inst in self.instances for v in inst.profile.home]

    @property
    def fingerprint(self):
        return hashlib.sha256(
            json.dumps(
                {
                    "scene": self.data,
                    "profiles": [i.profile.fingerprint for i in self.instances],
                },
                sort_keys=True,
            ).encode()
        ).hexdigest()


def load_workcell(name="fr5_ur5e", root=ROOT):
    identifier(name)
    path = root / "config/workcells" / f"{name}.yaml"
    if not path.is_file():
        raise ValueError(f"Unknown workcell: {name}")
    data = yaml.safe_load(path.read_text())
    try:
        if not isinstance(data, dict) or set(data) != {
            "schema_version",
            "id",
            "label",
            "physics",
            "objects",
            "robots",
        }:
            raise ValueError("Unknown or missing workcell fields")
        if not isinstance(data["label"], str) or not data["label"].strip():
            raise ValueError("Expected a label")
        if data["schema_version"] != 1 or data["id"] != name:
            raise ValueError("Expected workcell schema 1 and matching id")
        if set(data["physics"]) != {"timestep", "integrator", "sim_speed_factor"}:
            raise ValueError(
                "Expected common timestep, integrator and sim_speed_factor"
            )
        for key in ("timestep", "sim_speed_factor"):
            _number(data["physics"][key], key, positive=True)
        if data["physics"]["integrator"] not in (
            "implicitfast",
            "implicit",
            "Euler",
            "RK4",
        ):
            raise ValueError("Unsupported integrator")
        # Validate environment with the same shape contract as single-arm scenes.
        names = set()
        if not isinstance(data["objects"], list):
            raise ValueError("objects must be a list")
        for obj in data["objects"]:
            if set(obj) != {"name", "shape", "size", "xyz", "rpy", "rgba"}:
                raise ValueError("Invalid object fields")
            identifier(obj["name"])
            if obj["name"] in names:
                raise ValueError("Duplicate object name")
            names.add(obj["name"])
            if obj["shape"] not in ("box", "cylinder"):
                raise ValueError("Unsupported object shape")
            vector(
                obj["size"], 3 if obj["shape"] == "box" else 2, "size", positive=True
            )
            for key in ("xyz", "rpy"):
                vector(obj[key], 3, key)
            vector(obj["rgba"], 4, "rgba")
            if any(v < 0 or v > 1 for v in obj["rgba"]):
                raise ValueError("Invalid RGBA")
        if not isinstance(data["robots"], list) or len(data["robots"]) != 2:
            raise ValueError("This stage supports exactly two robot instances")
        instances = []
        used = set()
        for entry in data["robots"]:
            if set(entry) - {"instance", "model", "xyz", "rpy", "verification"} or not {
                "instance",
                "model",
                "xyz",
                "rpy",
            }.issubset(entry):
                raise ValueError("Invalid instance fields")
            identifier(entry["instance"])
            if any(
                entry["instance"].startswith(other + "_")
                or other.startswith(entry["instance"] + "_")
                for other in used
            ):
                raise ValueError("Ambiguous instance prefixes")
            if entry["instance"] in used or entry["instance"] in (
                "world",
                "workcell",
                "both",
            ):
                raise ValueError("Duplicate or reserved instance name")
            used.add(entry["instance"])
            for key in ("xyz", "rpy"):
                vector(entry[key], 3, key)
            profile = load_profile(entry["model"], root=root)
            if "verification" in entry:
                if set(entry["verification"]) != {"plan_target_rad"}:
                    raise ValueError("Unknown instance verification fields")
                vector(
                    entry["verification"]["plan_target_rad"],
                    len(profile.joints),
                    "plan_target_rad",
                )
            instances.append(Instance(entry, profile))
        cell = Workcell(root.resolve(), data, tuple(instances))
        if len(cell.joints) != len(set(cell.joints)):
            raise ValueError("Overlapping joint names")
        return cell
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError(f"Malformed workcell: {error}") from error


def controllers(cell):
    update_rate = controller_config(cell.instances[0].profile)["controller_manager"][
        "ros__parameters"
    ]["update_rate"]
    result = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": update_rate,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
            }
        }
    }
    result["joint_state_broadcaster"] = {"ros__parameters": {"use_sim_time": True}}
    for inst in cell.instances:
        config = controller_config(inst.profile)[
            inst.profile.ros["trajectory_controller"]
        ]["ros__parameters"]
        config["joints"] = inst.joints
        config["use_sim_time"] = True
        # The hardware implements the position servo; preserve its last command during future-start holds.
        config["open_loop_control"] = True
        constraints = config["constraints"]
        config["constraints"] = {
            inst.prefix + k if k in inst.profile.joints else k: v
            for k, v in constraints.items()
        }
        result["controller_manager"]["ros__parameters"][inst.controller] = {
            "type": "joint_trajectory_controller/JointTrajectoryController"
        }
        result[inst.controller] = {"ros__parameters": config}
    return result
