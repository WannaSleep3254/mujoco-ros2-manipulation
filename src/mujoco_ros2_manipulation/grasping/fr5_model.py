"""Build a private FR5/contact prototype from the checked workcell assets.

The original six-axis arm, inertia, limits, actuator gains and declared SRDF
exclusions are retained. The second arm is omitted, and an experimental tool,
free block and small table are added only to this detached XML document.
Nothing here changes the generated workcell, a ROS model or native policies.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ARM_JOINTS = tuple("left_j" + str(i) for i in range(1, 7))
TCP_SITE = "fr5_grasp_tcp"
TIP_BODY = "left_wrist3_link"
FINGER_GEOMS = ("fr5_left_finger_geom", "fr5_right_finger_geom")
BLOCK_GEOM = "fr5_block_geom"
TABLE_GEOM = "fr5_table_geom"


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def _fmt(values):
    return " ".join(format(float(value), ".17g") for value in values)


def _xml_bytes(root):
    copy = deepcopy(root)
    ET.indent(copy, space="  ")
    return ET.tostring(copy, encoding="utf-8", xml_declaration=True)


def build_model(source_directory):
    """Return detached MJCF bytes and a provenance manifest, using static FK.

    All manufacturer inputs are read only. No physical step or trajectory is
    executed while locating the initial TCP and the nearby block/table.
    """
    import mujoco
    import numpy as np

    source = Path(source_directory).resolve()
    raw = (source / "cell.xml").read_bytes()
    meta_raw = (source / "source.json").read_bytes()
    metadata = json.loads(meta_raw)
    if metadata.get("outputs", {}).get("cell.xml") != sha256(raw):
        raise ValueError("Workcell MJCF does not match its source manifest")
    if tuple(metadata.get("joints", [])[:6]) != ARM_JOINTS:
        raise ValueError("Expected the checked FR5 left six-joint contract")
    if len(source.parents) < 4:
        raise ValueError("Expected the generated workcell source directory")
    workspace = source.parents[3]
    if source != workspace / "runtime/generated/workcells/fr5_ur5e":
        raise ValueError("Expected runtime/generated/workcells/fr5_ur5e as the source")
    arm_metadata_path = workspace / "models/fr5/source.json"
    arm_metadata_raw = arm_metadata_path.read_bytes()
    if sha256(arm_metadata_raw) != metadata.get("inputs", {}).get("models/fr5/source.json"):
        raise ValueError("FR5 source metadata differs from the checked workcell")
    arm_metadata = json.loads(arm_metadata_raw)
    declared_files = arm_metadata.get("integrity", {}).get("inputs", {}).get("files", {})
    original = ET.fromstring(raw)
    left = original.find("worldbody/body[@name='left_world']")
    if left is None or tuple(j.get("name") for j in left.iter("joint")) != ARM_JOINTS:
        raise ValueError("Source FR5 bodies/joints do not match the checked contract")

    root = ET.Element("mujoco", model="fr5_parallel_gripper_prototype_v1")
    ET.SubElement(root, "compiler", angle="radian", meshdir=".")
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast",
                  iterations="100", impratio="10")
    asset = ET.SubElement(root, "asset")
    used_meshes = {geom.get("mesh") for geom in left.iter("geom") if geom.get("mesh")}
    mesh_inputs = {}
    for mesh in original.findall("asset/mesh"):
        if mesh.get("name") not in used_meshes:
            continue
        mesh_path = Path(mesh.attrib["file"])
        if not mesh_path.is_absolute() or not mesh_path.is_file():
            raise ValueError("Expected existing explicit manufacturer mesh paths")
        try:
            relative = mesh_path.relative_to(workspace).as_posix()
        except ValueError as error:
            raise ValueError("Manufacturer mesh is outside the checked workspace") from error
        digest = sha256(mesh_path.read_bytes())
        if declared_files.get(relative) != digest:
            raise ValueError("Manufacturer mesh differs from the FR5 input manifest")
        mesh_inputs[str(mesh_path)] = digest
        asset.append(deepcopy(mesh))
    if {mesh.get("name") for mesh in asset} != used_meshes:
        raise ValueError("Missing FR5 mesh asset")
    world = ET.SubElement(root, "worldbody")
    left = deepcopy(left)
    world.append(left)
    for body in left.iter("body"):
        for i, geom in enumerate(body.findall("geom")):
            geom.set("name", body.attrib["name"] + "_geom_" + str(i))
    for name in ("workcell_floor", "workcell_left_pedestal"):
        body = original.find("worldbody/body[@name='" + name + "']")
        if body is None:
            raise ValueError("Missing checked static support: " + name)
        body = deepcopy(body)
        for i, geom in enumerate(body.findall("geom")):
            geom.set("name", name + "_geom_" + str(i))
        world.append(body)
    tip = left.find(".//body[@name='" + TIP_BODY + "']")
    if tip is None:
        raise ValueError("Missing source FR5 flange body")
    tool = ET.SubElement(tip, "body", name="fr5_synthetic_gripper", pos="0 0 .045", quat="0 1 0 0")
    surface = dict(condim="4", friction=".8 .01 .001", solref=".008 1", solimp=".95 .99 .001")
    ET.SubElement(tool, "geom", name="fr5_palm_geom", type="box", size=".055 .025 .01",
                  mass=".15", rgba=".25 .45 .65 1", **surface)
    ET.SubElement(tool, "site", name=TCP_SITE, pos="0 0 -.10", size=".003", rgba="1 0 0 1")
    for side, x, axis, geom_name in (("left", -.04, "1 0 0", FINGER_GEOMS[0]),
                                      ("right", .04, "-1 0 0", FINGER_GEOMS[1])):
        finger = ET.SubElement(tool, "body", name="fr5_" + side + "_finger", pos=_fmt((x, 0, -.07)))
        ET.SubElement(finger, "joint", name="fr5_" + side + "_finger_slide", type="slide", axis=axis,
                      range="0 .034", damping="1", armature=".001")
        ET.SubElement(finger, "geom", name=geom_name, type="box", size=".005 .02 .04",
                      mass=".03", rgba=".75 .75 .8 1", **surface)

    contact = ET.SubElement(root, "contact")
    exclusions = []
    for excluded in original.findall("contact/exclude"):
        if all(excluded.get(key, "").startswith("left_") for key in ("body1", "body2")):
            exclusions.append(dict(excluded.attrib))
            contact.append(deepcopy(excluded))
    # No new broad robot/tool exclusions. MuJoCo's normal fixed/adjacent-body
    # collision filtering remains in effect and is stated in the manifest.
    actuator = ET.SubElement(root, "actuator")
    for name in ARM_JOINTS:
        entry = original.find("actuator/position[@joint='" + name + "']")
        if entry is None:
            raise ValueError("Missing source position servo: " + name)
        actuator.append(deepcopy(entry))
    for side in ("left", "right"):
        ET.SubElement(actuator, "position", name="fr5_"+side+"_finger_servo",
            joint="fr5_"+side+"_finger_slide", kp="2000", kv="20", ctrllimited="true",
            ctrlrange="0 .034", forcelimited="true", forcerange="-20 20")

    initial = list(metadata["home_rad"][:6])
    initial[3:5] = [-math.pi/2, -math.pi/2]
    intermediate = mujoco.MjModel.from_xml_string(_xml_bytes(root).decode())
    data = mujoco.MjData(intermediate)
    for name, value in zip(ARM_JOINTS, initial):
        data.qpos[intermediate.joint(name).qposadr[0]] = value
    mujoco.mj_forward(intermediate, data)
    tcp_id = intermediate.site(TCP_SITE).id
    initial_tcp = data.site_xpos[tcp_id].copy()
    rotation = data.site_xmat[tcp_id].reshape(3, 3).copy()
    if float(rotation[2, 2]) < .999:
        raise ValueError("Prototype initial gripper must face vertically down at the table")
    block_position = initial_tcp + np.array([0., 0., -.08])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    table_top = float(block_position[2] - .025)
    table = ET.SubElement(world, "body", name="fr5_grasp_table",
        pos=_fmt((block_position[0]-.04, block_position[1], table_top-.02)))
    ET.SubElement(table, "geom", name=TABLE_GEOM, type="box", size=".16 .1 .02",
                  rgba=".3 .35 .4 1", **surface)
    block = ET.SubElement(world, "body", name="fr5_grasp_block", pos=_fmt(block_position), quat=_fmt(quat))
    ET.SubElement(block, "freejoint", name="fr5_block_free")
    ET.SubElement(block, "geom", name=BLOCK_GEOM, type="box", size=".02 .015 .025",
                  mass=".08", rgba=".85 .5 .1 1", **surface)
    keys = ET.SubElement(root, "keyframe")
    ET.SubElement(keys, "key", name="prototype_home",
        qpos=_fmt(initial + [0., 0.] + block_position.tolist() + quat.tolist()), ctrl=_fmt(initial + [0., 0.]))
    model_raw = _xml_bytes(root)
    compiled = mujoco.MjModel.from_xml_string(model_raw.decode())
    scalar_names = (*ARM_JOINTS, "fr5_left_finger_slide", "fr5_right_finger_slide")
    qpos_layout = []
    for joint_id in range(compiled.njnt):
        kind = int(compiled.jnt_type[joint_id])
        qpos_layout.append(dict(name=mujoco.mj_id2name(compiled, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
            type={int(mujoco.mjtJoint.mjJNT_FREE): "free", int(mujoco.mjtJoint.mjJNT_HINGE): "hinge",
                  int(mujoco.mjtJoint.mjJNT_SLIDE): "slide"}[kind],
            qpos_address=int(compiled.jnt_qposadr[joint_id]),
            qpos_width=7 if kind == int(mujoco.mjtJoint.mjJNT_FREE) else 1))
    manifest = dict(schema_version=1, prototype="fr5_parallel_gripper_v1", simulation_only=True,
        scope="Detached FR5 dynamics/contact prototype; not ROS/MoveIt/native contact-stop integration",
        source_directory=str(source), source_cell_sha256=sha256(raw), source_manifest_sha256=sha256(meta_raw),
        source_arm_metadata_sha256=sha256(arm_metadata_raw),
        manufacturer_repository=arm_metadata.get("source_repository"),
        manufacturer_commit=arm_metadata.get("source_commit"),
        source_workcell_sha256=metadata["workcell_sha256"], mesh_inputs_sha256=mesh_inputs,
        source_home_rad=metadata["home_rad"][:6], initial_arm_rad=initial,
        arm_joints=list(ARM_JOINTS), arm_dynamics="Source FR5 joint, inertial, actuator and limit attributes retained",
        arm_joint_limits={name: metadata["joint_limits"][name] for name in ARM_JOINTS},
        inherited_source_collision_exclusions=exclusions, added_collision_exclusions=[],
        implicit_collision_filter="MuJoCo default welded/adjacent-body filtering remains active",
        tool=dict(id="fr5_synthetic_parallel_gripper", parent_tip=TIP_BODY, tcp_site=TCP_SITE,
            parent_to_tool_translation_m=[0., 0., .045], parent_to_tool_quaternion_wxyz=[0., 1., 0., 0.],
            tool_to_tcp_translation_m=[0., 0., -.10], physical_geometry=True,
            nominal_finger_tip_above_table_m=.015,
            finger_geoms=list(FINGER_GEOMS), finger_force_limit_n=20.),
        initial_tcp_position_m=initial_tcp.tolist(), tcp_rotation_matrix=rotation.tolist(),
        pickup_tcp_position_m=block_position.tolist(), placement_offset_m=[-.08, 0., 0.],
        lift_offset_m=[0., 0., .08], table_top_m=table_top,
        block=dict(body="fr5_grasp_block", geom=BLOCK_GEOM, free_joint="fr5_block_free",
            half_size_m=[.02, .015, .025], mass_kg=.08, initial_xyz_m=block_position.tolist(),
            initial_quaternion_wxyz=quat.tolist(), attachment_constraint="none"),
        qpos_state_contract=dict(nq=int(compiled.nq), qpos_layout=qpos_layout,
            free_joint_format="world xyz metres followed by quaternion wxyz",
            scalar_joint_names=list(scalar_names),
            scalar_joint_units={name: "rad" if name in ARM_JOINTS else "m" for name in scalar_names},
            scalar_joint_positions="Actual MuJoCo qpos observations, never position-servo ctrl targets"),
        physical_engine_used_for_static_fk="MuJoCo "+mujoco.__version__,
        mjcf_sha256=sha256(model_raw))
    manifest["manifest_sha256"] = sha256(json.dumps(manifest, sort_keys=True,
        separators=(",", ":"), allow_nan=False).encode())
    return model_raw, manifest


def write_model(source_directory, output):
    raw, manifest = build_model(source_directory)
    directory = Path(output)
    directory.mkdir(parents=True, exist_ok=False)
    (directory / "fr5_grasp.xml").write_bytes(raw)
    (directory / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n")
    return manifest
