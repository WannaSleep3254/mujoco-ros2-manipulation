"""Fixed visible obstacle and bounded, deliberately colliding simulation motion."""
import xml.etree.ElementTree as ET
from pathlib import Path

OBSTACLE_BODY = "demo_obstacle"
OBSTACLE_GEOM = "demo_obstacle_geom"
OBSTACLE_XYZ = (-0.45, 0.72, 0.5)
OBSTACLE_SIZE = (0.16, 0.16, 1.0)
OBSTACLE_RGBA = (1.0, 0.15, 0.02, 1.0)
FIRST_JOINT_OFFSETS = (2.0, -2.0)
DURATION_SEC = 4.0


def numbers(values):
    return " ".join(str(v) for v in values)


def contact_demo_model(cell, directory, *, obstacle=True):
    """Add a fixed post before startup; never add a mocap or injection fixture."""
    directory = Path(directory).resolve()
    if cell.id != "fr5_ur5e":
        raise ValueError("Visible contact demo requires fr5_ur5e")
    if not directory.is_relative_to(cell.root / "runtime/validation/runs"):
        raise ValueError("Contact demo requires a verification run directory")
    tree = ET.parse(cell.output / "cell.xml")
    model = tree.getroot()
    if model.find(".//body[@name='demo_obstacle']") is not None:
        raise ValueError("Reserved obstacle already exists")
    if obstacle:
        body = ET.SubElement(
            model.find("worldbody"),
            "body",
            name=OBSTACLE_BODY,
            pos=numbers(OBSTACLE_XYZ),
        )
        ET.SubElement(
            body,
            "geom",
            name=OBSTACLE_GEOM,
            type="box",
            size=numbers(v / 2 for v in OBSTACLE_SIZE),
            rgba=numbers(OBSTACLE_RGBA),
            group="0",
            contype="1",
            conaffinity="1",
        )
    statistic = model.find("statistic")
    if statistic is None:
        statistic = ET.SubElement(model, "statistic")
    statistic.set("center", "0 .25 .65")
    statistic.set("extent", "1.5")
    visual = model.find("visual")
    if visual is None:
        visual = ET.SubElement(model, "visual")
    camera = visual.find("global")
    if camera is None:
        camera = ET.SubElement(visual, "global")
    camera.set("azimuth", "45")
    camera.set("elevation", "-25")
    path = directory / "contact_test.xml"
    if path.exists():
        raise ValueError("Contact demo fixture already exists")
    tree.write(path, encoding="unicode")
    return path


def add_demo_urdf(robot):
    """Use the same fixed pose and box for RViz/MoveIt and the physical scene."""
    if robot.find("link[@name='demo_obstacle']") is not None:
        raise ValueError("Reserved obstacle link already exists")
    link = ET.SubElement(robot, "link", name=OBSTACLE_BODY)
    for kind in ("visual", "collision"):
        element = ET.SubElement(link, kind)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "box", size=numbers(OBSTACLE_SIZE))
        if kind == "visual":
            material = ET.SubElement(element, "material", name="demo_obstacle_orange")
            ET.SubElement(material, "color", rgba=numbers(OBSTACLE_RGBA))
    joint = ET.SubElement(robot, "joint", name="demo_obstacle_fixed", type="fixed")
    ET.SubElement(joint, "parent", link="world")
    ET.SubElement(joint, "child", link=OBSTACLE_BODY)
    ET.SubElement(joint, "origin", xyz=numbers(OBSTACLE_XYZ), rpy="0 0 0")
