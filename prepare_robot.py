"""Prepare a configured URDF robot for MuJoCo and ros2_control."""

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from robot_config import list_profiles, load_profile, write_controller_config


def write_xml(root, destination):
    ET.indent(root)
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)


def prepare(profile):
    output = profile.output
    joint_names = profile.joints
    home = profile.home
    settings = profile.data
    output.mkdir(parents=True, exist_ok=True)
    repository = profile.source_path('repository_path')
    source = profile.source_path('urdf')
    revision = subprocess.check_output(
        ['git', '-C', str(repository), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != settings['source']['commit']:
        raise ValueError(f'{profile.id}: source revision does not match the profile')
    if settings['source']['format'] == 'xacro':
        # Load the local ROS overlay only in the Xacro child process.
        rendered = subprocess.check_output([
            'bash', '-c', 'source "$1"\nexec xacro "$2"', 'prepare_robot',
            str(profile.root / 'manipulation_env.sh'), str(source)], text=True)
        robot = ET.fromstring(rendered)
    else:
        robot = ET.parse(source).getroot()
    if robot.find('ros2_control') is not None:
        raise ValueError(f'{profile.id}: source must not load a physical hardware driver')
    # Correct the source exporter's typo in a copy used by both simulators.
    for origin in robot.findall('.//collision/origins'):
        origin.tag = 'origin'
    joints = {joint.get('name'): joint for joint in robot.findall('joint')
              if joint.get('type') != 'fixed'}
    if set(joints) != set(joint_names):
        raise ValueError(f'{profile.id}: profile joints do not match the URDF')
    limits = {
        name: {key: float(joints[name].find('limit').get(key))
               for key in ['lower', 'upper', 'effort', 'velocity']}
        for name in joint_names
    }
    for index, name in enumerate(joint_names):
        for key, target in [('home_rad', home),
                            ('direct_target_rad', settings['verification']['direct_target_rad']),
                            ('moveit_target_rad', settings['verification']['moveit_target_rad'])]:
            if not limits[name]['lower'] <= target[index] <= limits[name]['upper']:
                raise ValueError(f'{profile.id}: {key} for {name} is outside the URDF limits')

    imported = ET.fromstring(ET.tostring(robot))
    if settings['source'].get('mujoco_visuals') == 'collision':
        # MuJoCo cannot import the official UR DAE visuals. Use the manufacturer's
        # collision STL for display; retain the original visuals in the ROS URDF.
        for link in imported.findall('link'):
            for visual in link.findall('visual'):
                link.remove(visual)
            for collision in link.findall('collision'):
                visual = deepcopy(collision)
                visual.tag = 'visual'
                link.append(visual)
    for mesh in imported.findall('.//mesh'):
        filename = mesh.get('filename')
        if not filename.startswith('package://'):
            raise ValueError(f'Expected a package mesh URI: {filename}')
        package, relative = filename.removeprefix('package://').split('/', 1)
        package_root = profile.path(settings['source']['mesh_packages'][package])
        mesh_path = (package_root / relative).resolve()
        if not mesh_path.is_relative_to(package_root) or not mesh_path.is_file():
            raise ValueError(f'Missing or invalid mesh: {filename}')
        mesh.set('filename', str(mesh_path))
    visual_assets = []
    if settings['source'].get('mujoco_visuals') == 'dae':
        from collada_visuals import export_dae_visuals
        cache = {}
        visual_directory = profile.root / 'runtime' / 'generated' / profile.id / 'visual_meshes'
        for link in imported.findall('link'):
            for visual in list(link.findall('visual')):
                mesh = visual.find('geometry/mesh')
                if mesh is None or Path(mesh.get('filename')).suffix.lower() != '.dae':
                    continue
                source_mesh = Path(mesh.get('filename'))
                if source_mesh not in cache:
                    relative = str(source_mesh.relative_to(profile.root))
                    prefix = source_mesh.stem + '_' + hashlib.sha256(relative.encode()).hexdigest()[:8]
                    cache[source_mesh] = export_dae_visuals(source_mesh, visual_directory, prefix)
                    visual_assets.append({'source': relative,
                        'triangles': sum(part.triangles for part in cache[source_mesh]),
                        'colors_rgba': [part.rgba for part in cache[source_mesh]]})
                link.remove(visual)
                for part in cache[source_mesh]:
                    replacement = deepcopy(visual)
                    replacement.find('geometry/mesh').set('filename', str(part.path))
                    for material in replacement.findall('material'):
                        replacement.remove(material)
                    material = ET.SubElement(replacement, 'material', {'name': part.path.stem})
                    ET.SubElement(material, 'color', {'rgba': ' '.join(map(str, part.rgba))})
                    link.append(replacement)
    ET.SubElement(ET.SubElement(imported, 'mujoco'), 'compiler', {
        'strippath': 'false', 'discardvisual': 'false', 'fusestatic': 'false'})
    model = mujoco.MjModel.from_xml_string(ET.tostring(imported, encoding='unicode'))
    scene_path = profile.scene_path
    mujoco.mj_saveLastXML(str(scene_path), model)
    scene = ET.parse(scene_path).getroot()
    scene.set('model', profile.ros['robot_name'])
    scene.find('compiler').set('meshdir', '.')
    for mesh in scene.findall('./asset/mesh'):
        # Keep subdirectories: UR collision and visual meshes have equal stems.
        mesh.set('file', os.path.relpath(mesh.get('file'), output))
    option = scene.find('option')
    if option is None:
        option = ET.SubElement(scene, 'option')
    option.set('timestep', str(settings['physics']['timestep']))
    option.set('integrator', settings['physics']['integrator'])
    for joint in scene.findall('.//joint'):
        if joint.get('name') in joint_names:
            joint.set('damping', str(settings['servo']['joint_damping']))
            joint.set('armature', str(settings['servo']['joint_armature']))
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
    srdf_path = profile.source_path('srdf')
    for pair in ET.parse(srdf_path).getroot().findall('disable_collisions'):
        ET.SubElement(contact, 'exclude', {
            'body1': pair.get('link1'), 'body2': pair.get('link2')})
    actuators = ET.SubElement(scene, 'actuator')
    for name, kp, kv in zip(joint_names, settings['servo']['kp'], settings['servo']['kv']):
        bounds = limits[name]
        ET.SubElement(actuators, 'position', {
            'name': name, 'joint': name, 'kp': str(kp), 'kv': str(kv),
            'ctrllimited': 'true', 'ctrlrange': f"{bounds['lower']} {bounds['upper']}",
            'forcelimited': 'true', 'forcerange': f"{-bounds['effort']} {bounds['effort']}"})
    # MJCF qpos order can differ from the configured ROS joint/actuator order.
    qpos = model.qpos0.copy()
    for name, initial in zip(joint_names, home):
        qpos[int(model.joint(name).qposadr[0])] = initial
    ET.SubElement(ET.SubElement(scene, 'keyframe'), 'key', {
        'name': 'home', 'qpos': ' '.join(map(str, qpos)),
        'ctrl': ' '.join(map(str, home))})
    write_xml(scene, scene_path)

    control = ET.SubElement(robot, 'ros2_control', {'name': profile.ros['system_name'], 'type': 'system'})
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'mujoco_ros2_control/MujocoSystemInterface'
    for name, value in {
        'mujoco_model': str(scene_path), 'initial_keyframe': 'home',
        'headless': 'false', 'sim_speed_factor': str(settings['physics']['sim_speed_factor']),
    }.items():
        ET.SubElement(hardware, 'param', {'name': name}).text = value
    for name, initial in zip(joint_names, home):
        joint = ET.SubElement(control, 'joint', {'name': name})
        ET.SubElement(joint, 'command_interface', {'name': 'position'})
        position = ET.SubElement(joint, 'state_interface', {'name': 'position'})
        ET.SubElement(position, 'param', {'name': 'initial_value'}).text = str(initial)
        ET.SubElement(joint, 'state_interface', {'name': 'velocity'})
        ET.SubElement(joint, 'state_interface', {'name': 'effort'})
    write_xml(robot, profile.urdf_path)
    write_controller_config(profile)

    compiled = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(compiled)
    mujoco.mj_resetDataKeyframe(compiled, data, 0)
    for _ in range(2000):
        mujoco.mj_step(compiled, data)
    assert np.isfinite(data.qpos).all()
    actual = np.array([data.joint(name).qpos[0] for name in joint_names])
    report = {
        'source_repository': settings['source']['repository_url'],
        'source_commit': revision,
        'robot': profile.label, 'robot_id': profile.id,
        'robot_profile_sha256': profile.fingerprint,
        'joints': joint_names, 'home_rad': home,
        'joint_limits': limits, 'demo_kp': settings['servo']['kp'], 'demo_kv': settings['servo']['kv'],
        'demo_joint_damping': settings['servo']['joint_damping'],
        'demo_joint_armature': settings['servo']['joint_armature'],
        'mujoco_python_version': mujoco.__version__,
        'initial_pose_error_rad': (actual - home).tolist(),
        'contacts_at_settled_home': data.ncon,
        'collision_geometry': 'MuJoCo convex hulls of manufacturer STL meshes',
        'mujoco_visual_geometry': settings['source'].get('mujoco_visuals', 'original'),
        'converted_visual_assets': visual_assets,
        'tip_frame': profile.ros['tip_frame'],
        'gripper': 'not included',
    }
    (output / 'source.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


def main(default_robot='fr5'):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default=default_robot)
    parser.add_argument('--list', action='store_true', help='list configured and pending robots')
    args = parser.parse_args()
    if args.list:
        for profile in list_profiles():
            state = 'enabled' if profile.data['enabled'] else 'pending'
            print(f'{profile.id}: {profile.label} ({state}, {profile.data["status"]})')
        return 0
    try:
        prepare(load_profile(args.robot))
    except ValueError as error:
        parser.error(str(error))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
