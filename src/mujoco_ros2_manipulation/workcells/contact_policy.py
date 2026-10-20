"""Latched stop policy and isolated fixture setup, without ROS imports."""
import xml.etree.ElementTree as ET
from pathlib import Path
from mujoco_ros2_manipulation.model_build.integrity import file_sha256


class ContactPolicy:
    def __init__(self, controllers):
        self.controllers = tuple(controllers)
        if len(self.controllers) != 2 or len(set(self.controllers)) != 2:
            raise ValueError("Expected two different arm controllers")
        self.native_seen = False
        self.latched = False

    def observe(self, value):
        self.native_seen = True
        self.latched = self.latched or value

    def targets(self, states):
        return (
            [name for name in self.controllers if states.get(name) == "active"]
            if self.latched
            else []
        )

    def phase(self, states):
        if self.latched:
            return (
                "stopped"
                if all(states.get(n) == "inactive" for n in self.controllers)
                else "stopping"
            )
        return (
            "ready"
            if self.native_seen
            and all(states.get(n) == "active" for n in self.controllers)
            else "waiting"
        )


def check_contact_runtime(root):
    binary = root / "ros_ws/install/mujoco_ros2_control/lib/libmujoco_ros2_control.so"
    digest = file_sha256(root / "native/step_monitor.hpp")
    if not binary.is_file() or digest.encode() not in binary.read_bytes():
        raise ValueError(
            "Rebuild native runtime before launching the contact stop interlock"
        )


def contact_test_model(cell, directory):
    directory = Path(directory).resolve()
    if not directory.is_relative_to(cell.root / "runtime/validation/runs"):
        raise ValueError("Contact test requires a verification run directory")
    model = ET.parse(cell.output / "cell.xml")
    if model.find(".//body[@name='left_forearm_link']") is None:
        raise ValueError("Contact fixture requires the FR5 left forearm")
    if model.find(".//body[@name='contact_stop_test_probe']") is not None:
        raise ValueError("Reserved contact fixture name already present")
    # The parked mocap obstacle must not expand the viewer camera to 100m.
    statistic = model.find("statistic")
    if statistic is None:
        statistic = ET.SubElement(model.getroot(), "statistic")
    statistic.set("center", "0 0 0.6")
    statistic.set("extent", "2.0")
    body = ET.SubElement(
        model.find("worldbody"),
        "body",
        name="contact_stop_test_probe",
        mocap="true",
        pos="100 100 100",
    )
    ET.SubElement(
        body,
        "geom",
        name="contact_stop_test_geom",
        type="sphere",
        size="0.06",
        contype="1",
        conaffinity="1",
        group="3",
        rgba="1 0 0 1",
    )
    path = directory / "contact_test.xml"
    if path.exists():
        raise ValueError("Contact test model already exists")
    model.write(path, encoding="unicode")
    return path
