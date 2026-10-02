"""Prepare the manufacturer's FR5 V6 URDF for MuJoCo and ros2_control."""

import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parent
REPOSITORY = ROOT / 'external' / 'frcobot_ros2'
DESCRIPTION = REPOSITORY / 'fairino_description'
OUTPUT = ROOT / 'models' / 'fr5'
JOINTS = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
HOME = [0, -1.2, 1.2, -1.5, -1.5, 0]
# Demonstration servo settings, not identified FAIRINO motor parameters.
KP = [2000, 4000, 3000, 800, 600, 400]
KV = [80, 120, 100, 20, 15, 10]


def write_xml(root, destination):
    ET.indent(root)
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    source = DESCRIPTION / 'urdf' / 'fairino5_v6.urdf'
    robot = ET.parse(source).getroot()
    # Correct the source exporter's typo in a copy used by both simulators.
    for origin in robot.findall('.//collision/origins'):
        origin.tag = 'origin'
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    assert set(joints) == set(JOINTS)
    limits = {
        name: {key: float(joints[name].find('limit').get(key))
               for key in ['lower', 'upper', 'effort', 'velocity']}
        for name in JOINTS
    }

    imported = ET.fromstring(ET.tostring(robot))
    for mesh in imported.findall('.//mesh'):
        filename = mesh.get('filename')
        assert filename.startswith('package://fairino_description/')
        mesh_path = DESCRIPTION / filename.removeprefix('package://fairino_description/')
        assert mesh_path.is_file(), mesh_path
        mesh.set('filename', str(mesh_path))
    ET.SubElement(ET.SubElement(imported, 'mujoco'), 'compiler', {
        'strippath': 'false', 'discardvisual': 'false', 'fusestatic': 'false'})
    model = mujoco.MjModel.from_xml_string(ET.tostring(imported, encoding='unicode'))
    scene_path = OUTPUT / 'fr5.xml'
    mujoco.mj_saveLastXML(str(scene_path), model)
    scene = ET.parse(scene_path).getroot()
    scene.set('model', 'fairino5_v6_robot')
    scene.find('compiler').set('meshdir', os.path.relpath(
        DESCRIPTION / 'meshes' / 'fairino5_v6', OUTPUT))
    for mesh in scene.findall('./asset/mesh'):
        mesh.set('file', Path(mesh.get('file')).name)
    option = scene.find('option')
    if option is None:
        option = ET.SubElement(scene, 'option')
    option.set('timestep', '0.002')
    option.set('integrator', 'implicitfast')
    for joint in scene.findall('.//joint'):
        if joint.get('name') in JOINTS:
            joint.set('damping', '1')
            joint.set('armature', '0.05')
    for geom in scene.findall('.//geom'):
        visual = geom.get('contype', '1') == '0' and geom.get('conaffinity', '1') == '0'
        geom.set('group', '2' if visual else '3')
    world = scene.find('worldbody')
    ET.SubElement(world, 'light', {
        'name': 'scene_light', 'pos': '1 -1 2', 'dir': '-1 1 -2',
        'directional': 'true', 'ambient': '0.3 0.3 0.3'})
    ET.SubElement(world, 'geom', {
        'name': 'floor', 'type': 'plane', 'pos': '0 0 -0.005',
        'size': '2 2 0.1', 'rgba': '0.25 0.28 0.3 1'})
    contact = ET.SubElement(scene, 'contact')
    srdf_path = REPOSITORY / 'fairino5_v6_moveit2_config' / 'config' / 'fairino5_v6_robot.srdf'
    for pair in ET.parse(srdf_path).getroot().findall('disable_collisions'):
        ET.SubElement(contact, 'exclude', {
            'body1': pair.get('link1'), 'body2': pair.get('link2')})
    actuators = ET.SubElement(scene, 'actuator')
    for name, kp, kv in zip(JOINTS, KP, KV):
        bounds = limits[name]
        ET.SubElement(actuators, 'position', {
            'name': name, 'joint': name, 'kp': str(kp), 'kv': str(kv),
            'ctrllimited': 'true', 'ctrlrange': f"{bounds['lower']} {bounds['upper']}",
            'forcelimited': 'true', 'forcerange': f"{-bounds['effort']} {bounds['effort']}"})
    values = ' '.join(map(str, HOME))
    ET.SubElement(ET.SubElement(scene, 'keyframe'), 'key', {
        'name': 'home', 'qpos': values, 'ctrl': values})
    write_xml(scene, scene_path)

    control = ET.SubElement(robot, 'ros2_control', {'name': 'FR5MuJoCo', 'type': 'system'})
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mujoco_ros2_control/MujocoSystemInterface'
    for name, value in {
        'mujoco_model': str(scene_path), 'initial_keyframe': 'home',
        'headless': 'false', 'sim_speed_factor': '1.0',
    }.items():
        ET.SubElement(hardware, 'param', {'name': name}).text = value
    for name, initial in zip(JOINTS, HOME):
        joint = ET.SubElement(control, 'joint', {'name': name})
        ET.SubElement(joint, 'command_interface', {'name': 'position'})
        position = ET.SubElement(joint, 'state_interface', {'name': 'position'})
        ET.SubElement(position, 'param', {'name': 'initial_value'}).text = str(initial)
        ET.SubElement(joint, 'state_interface', {'name': 'velocity'})
        ET.SubElement(joint, 'state_interface', {'name': 'effort'})
    write_xml(robot, OUTPUT / 'fr5.ros2_control.urdf')

    compiled = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(compiled)
    mujoco.mj_resetDataKeyframe(compiled, data, 0)
    for _ in range(2000):
        mujoco.mj_step(compiled, data)
    assert np.isfinite(data.qpos).all()
    report = {
        'source_repository': 'https://github.com/FAIR-INNOVATION/frcobot_ros2',
        'source_commit': subprocess.check_output(
            ['git', '-C', str(REPOSITORY), 'rev-parse', 'HEAD'], text=True).strip(),
        'robot': 'FR5 V6', 'joints': JOINTS, 'home_rad': HOME,
        'joint_limits': limits, 'demo_kp': KP, 'demo_kv': KV,
        'demo_joint_damping': 1, 'demo_joint_armature': 0.05,
        'mujoco_python_version': mujoco.__version__,
        'initial_pose_error_rad': (data.qpos - HOME).tolist(),
        'contacts_at_settled_home': data.ncon,
        'collision_geometry': 'MuJoCo convex hulls of manufacturer STL meshes',
        'gripper': 'not included',
    }
    (OUTPUT / 'source.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
