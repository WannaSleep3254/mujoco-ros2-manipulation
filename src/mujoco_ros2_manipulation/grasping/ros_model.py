"""Immutable opt-in ROS grasp models over the checked two-arm workcell.

This generator performs static FK only. It preserves both arm subtrees and
actuators, adds two metre slide joints and a free block, and does not run ROS,
step physics or weaken the base workcell validator. Native execution validation
belongs to the owner of the explicitly selected grasp target.
"""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import tempfile
import xml.etree.ElementTree as ET

import yaml

from ..workcells.integrity import check_workcell, OUTPUTS as BASE_OUTPUTS
from ..workcells.environment_overlay import _dependencies
from ..workcells.environment_assets import publish_directory
from .fr5_model import ARM_JOINTS, TCP_SITE, TIP_BODY, FINGER_GEOMS, BLOCK_GEOM, TABLE_GEOM

FINGER_JOINTS = ("fr5_left_finger_slide", "fr5_right_finger_slide")
FREE_JOINT = "fr5_block_free"
TOOL_BODY = "fr5_synthetic_gripper"
BLOCK_BODY, TABLE_BODY = "fr5_grasp_block", "fr5_grasp_table"
POLICY = "fr5_parallel_block_v1"
CONTROLLER = "gripper_controller"
ARTIFACT = "grasp-execution-target-v1"
OUTPUTS = (*BASE_OUTPUTS, "grasp_plugins.yaml")
MAX_FILE_BYTES = 16 * 1024 * 1024
MAX_OUTPUT_BYTES = 64 * 1024 * 1024
MAX_DEPENDENCY_BYTES = 512 * 1024 * 1024
_SHA = re.compile(r"[0-9a-f]{64}")


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def _sha(value):
    return hashlib.sha256(value).hexdigest()


def _read(path, maximum=MAX_FILE_BYTES):
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        info = os.fstat(descriptor)
        _require(stat.S_ISREG(info.st_mode) and 0 <= info.st_size <= maximum,
                 "Grasp input must be a bounded regular file")
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            raw = stream.read(maximum + 1)
        _require(len(raw) <= maximum, "Grasp input grew beyond its byte limit")
        return raw
    finally:
        os.close(descriptor)


def _checked_directory(path):
    path = Path(path).absolute()
    _require(".." not in path.parts, "Grasp paths cannot traverse parents")
    for parent in (path, *path.parents):
        _require(not parent.is_symlink(), "Grasp artifact directory parents cannot be linked")
    _require(path.is_dir(), "Grasp artifact directory does not exist")
    return path


def _json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            _require(key not in result, "Duplicate grasp metadata field")
            result[key] = value
        return result
    value = json.loads(raw, object_pairs_hook=pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite grasp metadata")))
    _canonical(value)
    return value


def _numbers(values):
    return " ".join(format(float(value), ".17g") for value in values)


def _xml(root):
    copy = deepcopy(root)
    ET.indent(copy, space="  ")
    return ET.tostring(copy, encoding="utf-8", xml_declaration=True)


def _producers():
    return {name: _sha(_read(Path(__file__).with_name(name), 1024 * 1024))
            for name in ("ros_model.py", "ros_target.py")}


def _base(base, validator):
    value = validator(base)
    _require(base.id == "fr5_ur5e" and len(base.joints) == 12 and tuple(base.joints[:6]) == ARM_JOINTS
             and len(base.instances) == 2 and base.instances[1].profile.id == "ur5e",
             "ROS grasp requires the checked FR5 plus UR5e workcell")
    _require(set(value["outputs"]) == set(BASE_OUTPUTS), "Incomplete base output inventory")
    return value


def _home(base):
    values = dict(zip(base.joints, base.home))
    values["left_j4"] = values["left_j5"] = -math.pi / 2
    return [values[name] for name in base.joints]


def _tool_mjcf(mj):
    _require(not any(mj.find(".//*[@name='" + name + "']") is not None for name in
        (TOOL_BODY, TCP_SITE, *FINGER_JOINTS, BLOCK_BODY, TABLE_BODY)), "Reserved grasp model name already exists")
    tip = mj.find(".//body[@name='" + TIP_BODY + "']")
    _require(tip is not None and mj.find("worldbody/body[@name='right_world']") is not None,
             "Both checked robot bodies are required")
    tool = ET.SubElement(tip, "body", name=TOOL_BODY, pos="0 0 .045", quat="0 1 0 0")
    surface = dict(condim="4", friction=".8 .01 .001", solref=".008 1", solimp=".95 .99 .001")
    ET.SubElement(tool, "geom", name="fr5_palm_geom", type="box", size=".055 .025 .01",
        mass=".15", rgba=".25 .45 .65 1", **surface)
    ET.SubElement(tool, "site", name=TCP_SITE, pos="0 0 -.10", size=".003", rgba="1 0 0 1")
    actuator = mj.find("actuator")
    _require(actuator is not None, "Missing source arm actuators")
    for side, x, axis, geom in (("left", -.04, "1 0 0", FINGER_GEOMS[0]),
                                ("right", .04, "-1 0 0", FINGER_GEOMS[1])):
        finger = ET.SubElement(tool, "body", name="fr5_" + side + "_finger", pos=_numbers((x, 0, -.07)))
        joint = "fr5_" + side + "_finger_slide"
        ET.SubElement(finger, "joint", name=joint, type="slide", axis=axis,
            range="0 .034", damping="1", armature=".001")
        ET.SubElement(finger, "geom", name=geom, type="box", size=".005 .02 .04",
            mass=".03", rgba=".75 .75 .8 1", **surface)
        ET.SubElement(actuator, "position", name="fr5_" + side + "_finger_servo", joint=joint,
            kp="2000", kv="20", ctrllimited="true", ctrlrange="0 .034",
            forcelimited="true", forcerange="-20 20")
    size = mj.find("size")
    if size is None:
        size = ET.SubElement(mj, "size")
    size.set("nuserdata", "2")
    custom = mj.find("custom")
    if custom is None:
        custom = ET.SubElement(mj, "custom")
    ET.SubElement(custom, "text", name="manipulation_grasp_policy", data=POLICY)


def _box_link(robot, name, half_size, mass, rgba):
    link = ET.SubElement(robot, "link", name=name)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value=str(mass))
    x, y, z = half_size
    ET.SubElement(inertial, "inertia", ixx=str(mass*(y*y+z*z)/3), iyy=str(mass*(x*x+z*z)/3),
        izz=str(mass*(x*x+y*y)/3), ixy="0", ixz="0", iyz="0")
    for role in ("visual", "collision"):
        child = ET.SubElement(link, role)
        ET.SubElement(ET.SubElement(child, "geometry"), "box", size=_numbers([v*2 for v in half_size]))
        if role == "visual":
            ET.SubElement(ET.SubElement(child, "material", name=name+"_material"), "color", rgba=_numbers(rgba))
    return link


def _joint(robot, name, parent, child, kind, xyz, rpy=(0., 0., 0.)):
    value = ET.SubElement(robot, "joint", name=name, type=kind)
    ET.SubElement(value, "parent", link=parent)
    ET.SubElement(value, "child", link=child)
    ET.SubElement(value, "origin", xyz=_numbers(xyz), rpy=_numbers(rpy))
    return value


def _tool_urdf(robot, table_position):
    _box_link(robot, TOOL_BODY, (.055,.025,.01), .15, (.25,.45,.65,1.))
    _joint(robot, TOOL_BODY+"_fixed", TIP_BODY, TOOL_BODY, "fixed", (0.,0.,.045), (math.pi,0.,0.))
    ET.SubElement(robot, "link", name=TCP_SITE)
    _joint(robot, TCP_SITE+"_fixed", TOOL_BODY, TCP_SITE, "fixed", (0.,0.,-.10))
    for side, x, axis in (("left",-.04,(1.,0.,0.)),("right",.04,(-1.,0.,0.))):
        name = "fr5_"+side+"_finger"
        _box_link(robot, name, (.005,.02,.04), .03, (.75,.75,.8,1.))
        joint = _joint(robot, name+"_slide", TOOL_BODY, name, "prismatic", (x,0.,-.07))
        ET.SubElement(joint, "axis", xyz=_numbers(axis))
        ET.SubElement(joint, "limit", lower="0", upper=".034", effort="20", velocity=".05")
        ET.SubElement(joint, "dynamics", damping="1", friction="0")
    _box_link(robot, TABLE_BODY, (.16,.1,.02), 1., (.3,.35,.4,1.))
    _joint(robot, TABLE_BODY+"_fixed", "world", TABLE_BODY, "fixed", table_position)


def _gripper_controller():
    return dict(joints=list(FINGER_JOINTS),command_interfaces=["position"],state_interfaces=["position","velocity"],
        state_publish_rate=50.,action_monitor_rate=20.,use_sim_time=True,open_loop_control=True,
        allow_partial_joints_goal=False,allow_nonzero_velocity_at_trajectory_end=False,
        constraints=dict(stopped_velocity_tolerance=.005,goal_time=5.,
            **{name:dict(trajectory=.04,goal=.0015) for name in FINGER_JOINTS}))


def _tool_contract():
    return dict(id="fr5_synthetic_parallel_gripper",parent_tip=TIP_BODY,body=TOOL_BODY,tcp_site=TCP_SITE,tcp_frame=TCP_SITE,
        parent_to_tool_translation_m=[0.,0.,.045],parent_to_tool_quaternion_wxyz=[0.,1.,0.,0.],tool_to_tcp_translation_m=[0.,0.,-.10],
        finger_geoms=list(FINGER_GEOMS),finger_force_limit_n=20.,open_target_m=0.,close_target_m=.026,
        adjacent_collision_exclusions=[dict(link1=TIP_BODY,link2=TOOL_BODY,reason="Adjacent"),
            dict(link1=TOOL_BODY,link2="fr5_left_finger",reason="Adjacent"),
            dict(link1=TOOL_BODY,link2="fr5_right_finger",reason="Adjacent")])


def _contact_contract():
    return dict(name=POLICY,phase_userdata_index=0,armed_userdata_index=1,default="base contact-stop remains enabled",
        block_geom=BLOCK_GEOM,table_geom=TABLE_GEOM,finger_geoms=list(FINGER_GEOMS))


def _xml_layout(mj, base):
    expected={**dict.fromkeys(base.joints,"hinge"),**dict.fromkeys(FINGER_JOINTS,"slide"),FREE_JOINT:"free"}
    rows=[];qpos=velocity=0
    for joint in mj.find("worldbody").iter():
        if joint.tag not in ("joint","freejoint"):continue
        name=joint.get("name");kind="free" if joint.tag=="freejoint" else joint.get("type","hinge")
        _require(expected.get(name)==kind,"Grasp joint layout differs")
        width=7 if kind=="free" else 1
        rows.append(dict(name=name,type=kind,qpos_address=qpos,qpos_width=width,velocity_address=velocity))
        qpos+=width;velocity+=6 if kind=="free" else 1
    _require(len(rows)==15 and len({row["name"] for row in rows})==15 and (qpos,velocity)==(21,20),"Grasp qpos coverage differs")
    return rows


def _compose(base, directory, initial_tcp=None, block_xyz=None, block_quat=None):
    """Static composition; returned model has no free object until FK locates it."""
    raw = {name:_read(base.output/name) for name in BASE_OUTPUTS}
    mj, robot, semantic = (ET.fromstring(raw[name]) for name in ("cell.xml","cell.urdf","cell.srdf"))
    _tool_mjcf(mj)
    home = _home(base)
    for keys in list(mj.findall("keyframe")):
        mj.remove(keys)
    if initial_tcp is None:
        import mujoco
        import numpy as np
        compiled = mujoco.MjModel.from_xml_string(_xml(mj).decode())
        data = mujoco.MjData(compiled)
        for name,value in zip(base.joints,home):
            data.qpos[int(compiled.joint(name).qposadr[0])] = value
        mujoco.mj_forward(compiled,data)
        site = int(compiled.site(TCP_SITE).id)
        initial_tcp = data.site_xpos[site].tolist()
        rotation = data.site_xmat[site].reshape(3,3).copy()
        _require(float(rotation[2,2])>.999,"Initial ROS grasp TCP must point vertically down")
        block_xyz = (data.site_xpos[site]+np.array([0.,0.,-.08])).tolist()
        quat = np.zeros(4);mujoco.mju_mat2Quat(quat,rotation.ravel());block_quat=quat.tolist()
    else:
        rotation = None
    table_top = block_xyz[2]-.025
    table_position = [block_xyz[0]-.04,block_xyz[1],table_top-.02]
    world = mj.find("worldbody")
    surface = dict(condim="4",friction=".8 .01 .001",solref=".008 1",solimp=".95 .99 .001")
    table = ET.SubElement(world,"body",name=TABLE_BODY,pos=_numbers(table_position))
    ET.SubElement(table,"geom",name=TABLE_GEOM,type="box",size=".16 .1 .02",rgba=".3 .35 .4 1",**surface)
    block = ET.SubElement(world,"body",name=BLOCK_BODY,pos=_numbers(block_xyz),quat=_numbers(block_quat))
    ET.SubElement(block,"freejoint",name=FREE_JOINT)
    ET.SubElement(block,"geom",name=BLOCK_GEOM,type="box",size=".02 .015 .025",mass=".08",rgba=".85 .5 .1 1",**surface)
    _tool_urdf(robot,table_position)
    hardware = robot.find("ros2_control/hardware")
    _require(hardware is not None,"Missing checked ROS hardware")
    hardware.find("param[@name='mujoco_model']").text = str(directory/"cell.xml")
    hardware.find("param[@name='initial_keyframe']").text = "home"
    control = robot.find("ros2_control")
    for name,value in zip(base.joints,home):
        control.find("joint[@name='"+name+"']/state_interface[@name='position']/param[@name='initial_value']").text = str(value)
    for name in FINGER_JOINTS:
        joint=ET.SubElement(control,"joint",name=name)
        ET.SubElement(joint,"command_interface",name="position")
        state=ET.SubElement(joint,"state_interface",name="position")
        ET.SubElement(state,"param",name="initial_value").text="0"
        ET.SubElement(joint,"state_interface",name="velocity")
        ET.SubElement(joint,"state_interface",name="effort")
    hand=ET.SubElement(semantic,"group",name="fr5_hand")
    for name in FINGER_JOINTS:ET.SubElement(hand,"joint",name=name)
    ET.SubElement(semantic,"end_effector",name="fr5_parallel_gripper",parent_link=TIP_BODY,group="fr5_hand",parent_group="left_arm")
    for pair in _tool_contract()["adjacent_collision_exclusions"]:
        ET.SubElement(semantic,"disable_collisions",**pair)
    for state in semantic.findall("group_state[@name='home']/joint"):
        if state.get("name") in base.joints:
            state.set("value",str(home[base.joints.index(state.get("name"))]))
    config=yaml.safe_load(raw["controllers.yaml"])
    config["controller_manager"]["ros__parameters"][CONTROLLER]={"type":"joint_trajectory_controller/JointTrajectoryController"}
    config[CONTROLLER]={"ros__parameters":_gripper_controller()}
    planning=yaml.safe_load(raw["moveit.yaml"])
    # The arm group still terminates at its checked flange. The actual TCP
    # offset is exposed by the tool registry, not by silently changing the chain.
    planning["robot_description"]=ET.tostring(robot,encoding="unicode")
    planning["robot_description_semantic"]=ET.tostring(semantic,encoding="unicode")
    manager=planning["moveit_simple_controller_manager"]
    manager["controller_names"].append(CONTROLLER)
    manager[CONTROLLER]=dict(type="FollowJointTrajectory",action_ns="follow_joint_trajectory",default=True,joints=list(FINGER_JOINTS))
    for name in FINGER_JOINTS:
        planning["robot_description_planning"]["joint_limits"][name]=dict(has_velocity_limits=True,max_velocity=.05,
            has_acceleration_limits=True,max_acceleration=.1)
    plugins={"/**":{"ros__parameters":{"mujoco_plugins":{"manipulation_grasp":{"type":"manipulation_grasp/ObservationPlugin"}}}}}
    values={"cell.xml":mj,"cell.urdf":robot,"cell.srdf":semantic,"controllers.yaml":config,"moveit.yaml":planning,
            "grasp_plugins.yaml":plugins,
            "moveit.rviz":raw["moveit.rviz"]}
    return values,dict(home_rad=home,initial_tcp_position_m=initial_tcp,block_xyz_m=block_xyz,
        block_quaternion_wxyz=block_quat,table_top_m=table_top,table_position_m=table_position,
        tcp_rotation_matrix=None if rotation is None else rotation.tolist())


def _compiled_contract(mj, base):
    import mujoco
    model=mujoco.MjModel.from_xml_string(_xml(mj).decode())
    _require((model.nu,model.njnt,model.nq,model.nv)==(14,15,21,20),"ROS grasp compiled dimensions differ")
    expected={**dict.fromkeys(base.joints,"hinge"),**dict.fromkeys(FINGER_JOINTS,"slide"),FREE_JOINT:"free"}
    types={int(mujoco.mjtJoint.mjJNT_HINGE):("hinge",1),int(mujoco.mjtJoint.mjJNT_SLIDE):("slide",1),
           int(mujoco.mjtJoint.mjJNT_FREE):("free",7)}
    layout=[]
    for index in range(model.njnt):
        name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,index)
        kind,width=types[int(model.jnt_type[index])]
        _require(expected.get(name)==kind,"Unexpected grasp joint type/name")
        layout.append(dict(name=name,type=kind,qpos_address=int(model.jnt_qposadr[index]),qpos_width=width,
            velocity_address=int(model.jnt_dofadr[index])))
    _require(model.neq==0 and model.nmocap==0,"ROS grasp block must remain physically free")
    return model,layout


def build_model(base, destination, *, base_validator=check_workcell):
    """Publish seven explicit ROS outputs plus grasp.json in a new directory."""
    before=_base(base,base_validator)
    destination=Path(destination).absolute()
    _require(not destination.exists() and not destination.is_symlink() and ".." not in destination.parts,
             "ROS grasp output must be a new unlinked directory")
    parent=_checked_directory(destination.parent)
    _require(not destination.is_relative_to(base.output.absolute()),"ROS grasp cannot overwrite the base model")
    values,geometry=_compose(base,destination)
    model,layout=_compiled_contract(values["cell.xml"],base)
    data=__import__('mujoco').MjData(model)
    # Initial keyframe only. Runtime owners must never use this to carry the block.
    for name,value in zip(base.joints,geometry["home_rad"]):
        data.qpos[int(model.joint(name).qposadr[0])]=value
    ctrl=[]
    for index in range(model.nu):
        joint_id=int(model.actuator_trnid[index,0]);ctrl.append(float(data.qpos[int(model.jnt_qposadr[joint_id])]))
    ET.SubElement(ET.SubElement(values["cell.xml"],"keyframe"),"key",name="home",
        qpos=_numbers(data.qpos.tolist()),ctrl=_numbers(ctrl))
    outputs={name:_xml(values[name]) if name.endswith((".xml",".urdf",".srdf")) else
             yaml.safe_dump(values[name],sort_keys=False).encode() if name.endswith(".yaml") else values[name] for name in OUTPUTS}
    stage=Path(tempfile.mkdtemp(prefix=".ros-grasp-",dir=parent))
    try:
        for name,raw in outputs.items():(stage/name).write_bytes(raw)
        dependencies=_dependencies(stage,base,final_directory=destination)
        _require(sum(item["bytes"] for item in dependencies.values())<=MAX_DEPENDENCY_BYTES,"Grasp dependency budget exceeded")
        scalar=list(base.joints)+list(FINGER_JOINTS)
        manifest=dict(schema_version=1,artifact_kind=ARTIFACT,id="fr5_parallel_block_v1",simulation_only=True,
            inputs=dict(base_workcell_id=base.id,base_workcell_sha256=base.fingerprint,
                base_manifest_sha256=_sha(_read(base.output/"source.json")),base_outputs_sha256=before["outputs"],
                producer_sha256=_producers()),
            arm_joint_names=list(base.joints),arm_joint_limits=deepcopy(before["joint_limits"]),home_rad=geometry["home_rad"],
            finger_joint_names=list(FINGER_JOINTS),finger_joint_limits={name:dict(lower=0.,upper=.034,effort=20.,velocity=.05) for name in FINGER_JOINTS},
            scalar_joint_units={name:"rad" if name in base.joints else "m" for name in scalar},
            qpos_state_contract=dict(nq=21,nv=20,njnt=15,nu=14,layout=layout,
                free_joint_format="world xyz m followed by quaternion wxyz"),
            tool=_tool_contract(),
            block=dict(body=BLOCK_BODY,geom=BLOCK_GEOM,free_joint=FREE_JOINT,half_size_m=[.02,.015,.025],mass_kg=.08,
                initial_xyz_m=geometry["block_xyz_m"],initial_quaternion_wxyz=geometry["block_quaternion_wxyz"],attachment_constraint="none"),
            table=dict(body=TABLE_BODY,geom=TABLE_GEOM,half_size_m=[.16,.1,.02],position_m=geometry["table_position_m"],top_m=geometry["table_top_m"]),
            contact_policy=_contact_contract(),
            geometry=geometry,dependencies=dependencies,outputs={name:_sha(raw) for name,raw in outputs.items()},
            validation=dict(static_fk_engine="MuJoCo "+__import__('mujoco').__version__,physics_steps=0,
                native_engine_required="3.12.0",native_execution_verified=False,ros_execution_verified=False))
        manifest["fingerprint"]=_sha(_canonical(manifest["inputs"]))
        (stage/"grasp.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2,allow_nan=False)+"\n")
        _require(_base(base,base_validator)==before,"Base workcell changed during grasp composition")
        check_model(stage,base,base_validator=base_validator,final_directory=destination)
        publish_directory(stage,destination)
        return check_model(destination,base,base_validator=base_validator)
    except BaseException:
        shutil.rmtree(stage,ignore_errors=True)
        raise


def check_model(directory, base, *, expected_sha256=None, base_validator=check_workcell, final_directory=None):
    """Rehash the immutable artifact and check structural safety without an engine."""
    directory=_checked_directory(directory)
    final_directory=directory if final_directory is None else Path(final_directory).absolute()
    raw=_read(directory/"grasp.json")
    if expected_sha256 is not None:
        _require(type(expected_sha256) is str and _SHA.fullmatch(expected_sha256) is not None
                 and _sha(raw)==expected_sha256,"Grasp manifest SHA256 differs")
    value=_json(raw);base_manifest=_base(base,base_validator)
    expected_fields={"schema_version","artifact_kind","id","simulation_only","inputs","arm_joint_names","arm_joint_limits","home_rad",
        "finger_joint_names","finger_joint_limits","scalar_joint_units","qpos_state_contract","tool","block","table","contact_policy",
        "geometry","dependencies","outputs","validation","fingerprint"}
    _require(type(value) is dict and set(value)==expected_fields and type(value["schema_version"]) is int
             and value["schema_version"]==1 and value["artifact_kind"]==ARTIFACT and value["id"]==POLICY
             and value["simulation_only"] is True,"Invalid grasp execution artifact")
    expected_inputs=dict(base_workcell_id=base.id,base_workcell_sha256=base.fingerprint,
        base_manifest_sha256=_sha(_read(base.output/"source.json")),base_outputs_sha256=base_manifest["outputs"],producer_sha256=_producers())
    _require(value["inputs"]==expected_inputs and value["fingerprint"]==_sha(_canonical(expected_inputs)),"Grasp base or producer identity changed")
    _require(value["arm_joint_names"]==list(base.joints) and value["home_rad"]==_home(base)
             and value["arm_joint_limits"]==base_manifest["joint_limits"] and value["finger_joint_names"]==list(FINGER_JOINTS),
             "Grasp arm/finger ownership changed")
    scalar=list(base.joints)+list(FINGER_JOINTS)
    _require(value["scalar_joint_units"]=={name:"rad" if name in base.joints else "m" for name in scalar},"Grasp scalar joint units changed")
    _require(value["finger_joint_limits"]=={name:dict(lower=0.,upper=.034,effort=20.,velocity=.05) for name in FINGER_JOINTS},
             "Grasp finger limits changed")
    _require(set(value["outputs"])==set(OUTPUTS) and set(p.name for p in directory.iterdir())==set(OUTPUTS)|{"grasp.json"},"Grasp output inventory changed")
    total=0
    for name in OUTPUTS:
        content=_read(directory/name);total+=len(content)
        _require(type(value["outputs"][name]) is str and _SHA.fullmatch(value["outputs"][name]) is not None
                 and _sha(content)==value["outputs"][name],"Grasp output hash differs: "+name)
    _require(total<=MAX_OUTPUT_BYTES,"Grasp output byte budget exceeded")
    dependencies=_dependencies(directory,base,final_directory=final_directory)
    _require(dependencies==value["dependencies"],"Grasp external asset identity changed")
    geometry=value["geometry"]
    _require(type(geometry) is dict and geometry["home_rad"]==_home(base),"Grasp initial state changed")
    for name,size in (("initial_tcp_position_m",3),("block_xyz_m",3),("block_quaternion_wxyz",4),("table_position_m",3)):
        row=geometry[name]
        _require(type(row) is list and len(row)==size and all(type(x) in (int,float) and math.isfinite(x) and abs(x)<=100 for x in row),"Invalid grasp geometry")
    _require(abs(math.hypot(*geometry["block_quaternion_wxyz"])-1.)<=1e-6,"Grasp block quaternion is not unit")
    values,expected_geometry=_compose(base,final_directory,geometry["initial_tcp_position_m"],geometry["block_xyz_m"],geometry["block_quaternion_wxyz"])
    _require(geometry["block_xyz_m"]==[geometry["initial_tcp_position_m"][0],geometry["initial_tcp_position_m"][1],geometry["initial_tcp_position_m"][2]-.08],
             "Grasp block no longer matches the declared TCP pickup placement")
    _require(geometry["table_position_m"]==expected_geometry["table_position_m"] and geometry["table_top_m"]==expected_geometry["table_top_m"],"Grasp table location changed")
    actual_mj=ET.fromstring(_read(directory/"cell.xml"));keys=actual_mj.find("keyframe")
    _require(keys is not None and len(keys)==1 and keys[0].get("name")=="home","Grasp initial keyframe changed")
    layout=_xml_layout(actual_mj,base)
    expected_contract=dict(nq=21,nv=20,njnt=15,nu=14,layout=layout,free_joint_format="world xyz m followed by quaternion wxyz")
    _require(value["qpos_state_contract"]==expected_contract,"Grasp qpos state contract changed")
    initial={**dict(zip(base.joints,_home(base))),**dict.fromkeys(FINGER_JOINTS,0.)}
    qpos=[]
    for row in layout:
        qpos.extend(geometry["block_xyz_m"]+geometry["block_quaternion_wxyz"] if row["type"]=="free" else [initial[row["name"]]])
    ctrl=[initial[entry.get("joint")] for entry in actual_mj.find("actuator")]
    for field,expected in (("qpos",qpos),("ctrl",ctrl)):
        numbers=[float(v) for v in keys[0].get(field,"").split()]
        _require(len(numbers)==len(expected) and all(math.isfinite(a) and abs(a-b)<=1e-12 for a,b in zip(numbers,expected)),"Grasp keyframe "+field+" changed")
    _require(value["tool"]==_tool_contract() and value["contact_policy"]==_contact_contract(),"Grasp TCP or contact policy contract changed")
    _require(value["block"]==dict(body=BLOCK_BODY,geom=BLOCK_GEOM,free_joint=FREE_JOINT,half_size_m=[.02,.015,.025],mass_kg=.08,
        initial_xyz_m=geometry["block_xyz_m"],initial_quaternion_wxyz=geometry["block_quaternion_wxyz"],attachment_constraint="none"),"Grasp free block contract changed")
    _require(value["table"]==dict(body=TABLE_BODY,geom=TABLE_GEOM,half_size_m=[.16,.1,.02],position_m=geometry["table_position_m"],top_m=geometry["table_top_m"]),"Grasp table contract changed")
    validation=value["validation"]
    _require(type(validation) is dict and set(validation)=={"static_fk_engine","physics_steps","native_engine_required","native_execution_verified","ros_execution_verified"}
        and isinstance(validation["static_fk_engine"],str) and validation["static_fk_engine"].startswith("MuJoCo ")
        and type(validation["physics_steps"]) is int and validation["physics_steps"]==0
        and validation["native_engine_required"]=="3.12.0" and validation["native_execution_verified"] is False
        and validation["ros_execution_verified"] is False,"Static model artifact cannot claim native or ROS execution verification")
    actual_mj.remove(keys)
    _require(_xml(actual_mj)==_xml(values["cell.xml"]),"Grasp physical geometry differs from its declared composition")
    for name in ("cell.urdf","cell.srdf"):
        _require(_xml(ET.fromstring(_read(directory/name)))==_xml(values[name]),"Grasp ROS geometry differs: "+name)
    for name in ("controllers.yaml","moveit.yaml","grasp_plugins.yaml"):
        _require(yaml.safe_load(_read(directory/name))==values[name],"Grasp controller/planning contract differs: "+name)
    _require(_read(directory/"moveit.rviz")==_read(base.output/"moveit.rviz"),"Grasp changed base RViz configuration")
    return deepcopy(value)
