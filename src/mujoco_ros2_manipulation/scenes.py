"""Single-arm workcell geometry shared by MuJoCo and the ROS collision model."""
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET

import yaml


def vector(value, length, name, *, positive=False):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{name}: expected {length} numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           or (positive and v <= 0) for v in value):
        raise ValueError(f'{name}: invalid finite vector')


def load_scene(scene_id, root):
    if not isinstance(scene_id, str) or not re.fullmatch('[a-z][a-z0-9_]*', scene_id):
        raise ValueError('Invalid scene id')
    path = root / 'config/scenes' / f'{scene_id}.yaml'
    if not path.is_file():
        raise ValueError(f'Unknown scene {scene_id}; see config/scenes')
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get('schema_version') != 1 or data.get('id') != scene_id:
        raise ValueError('Expected scene schema_version: 1 and matching id')
    try:
        if set(data) != {'schema_version', 'id', 'label', 'robot_pose', 'objects'}:
            raise ValueError('Unknown or missing scene fields')
        if set(data['robot_pose']) != {'xyz', 'rpy'}:
            raise ValueError('robot_pose requires xyz/rpy')
        for key in ('xyz', 'rpy'):
            vector(data['robot_pose'][key], 3, 'robot_pose.' + key)
        if not isinstance(data['objects'], list):
            raise ValueError('objects must be a list')
        names = set()
        for obj in data['objects']:
            if set(obj) != {'name', 'shape', 'size', 'xyz', 'rpy', 'rgba'}:
                raise ValueError('Unknown or missing object fields')
            name = obj['name']
            if not isinstance(name, str) or not re.fullmatch('[a-z][a-z0-9_]*', name) or name in names:
                raise ValueError('Object names must be valid and unique')
            names.add(name)
            if obj['shape'] not in ('box', 'cylinder'):
                raise ValueError('Supported shapes: box, cylinder')
            vector(obj['size'], 3 if obj['shape'] == 'box' else 2, name + '.size', positive=True)
            for key in ('xyz', 'rpy'):
                vector(obj[key], 3, name + '.' + key)
            vector(obj['rgba'], 4, name + '.rgba')
            if any(v < 0 or v > 1 for v in obj['rgba']):
                raise ValueError('RGBA values must be in [0, 1]')
    except (KeyError, TypeError) as error:
        raise ValueError(f'Malformed scene: {error}') from error
    return data


def scene_fingerprint(scene):
    return hashlib.sha256(json.dumps(scene, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def numbers(values):
    return ' '.join(map(str, values))


def apply_scene(robot, scene, world_frame):
    """Mount the original root without altering manufacturer internal transforms.

    Environment primitives are fixed URDF links, so robot_state_publisher,
    MoveIt collision geometry and the MuJoCo URDF importer share one model.
    Existing manufacturer world roots are renamed before mounting.
    """
    links = {link.get('name') for link in robot.findall('link')}
    children = {joint.find('child').get('link') for joint in robot.findall('joint')}
    roots = links - children
    if len(roots) != 1:
        raise ValueError('Expected one URDF root')
    root, = roots
    reserved = {'workcell_mount', 'workcell_source_root'} | {
        'workcell_' + obj['name'] for obj in scene['objects']}
    joint_names = {joint.get('name') for joint in robot.findall('joint')}
    if (links | joint_names) & reserved:
        raise ValueError('URDF names collide with workcell names')
    if world_frame in links:
        if root != world_frame:
            raise ValueError('World frame must be the original root')
        robot.find(f"link[@name='{root}']").set('name', 'workcell_source_root')
        for parent in robot.findall('joint/parent'):
            if parent.get('link') == root:
                parent.set('link', 'workcell_source_root')
        root = 'workcell_source_root'
    ET.SubElement(robot, 'link', {'name': world_frame})
    mount = ET.SubElement(robot, 'joint', {'name': 'workcell_mount', 'type': 'fixed'})
    ET.SubElement(mount, 'parent', {'link': world_frame})
    ET.SubElement(mount, 'child', {'link': root})
    pose = scene['robot_pose']
    ET.SubElement(mount, 'origin', {'xyz': numbers(pose['xyz']), 'rpy': numbers(pose['rpy'])})
    for obj in scene['objects']:
        name = 'workcell_' + obj['name']
        link = ET.SubElement(robot, 'link', {'name': name})
        for kind in ('visual', 'collision'):
            element = ET.SubElement(link, kind)
            geometry = ET.SubElement(element, 'geometry')
            attrs = ({'size': numbers(obj['size'])} if obj['shape'] == 'box' else
                     {'radius': str(obj['size'][0]), 'length': str(obj['size'][1])})
            ET.SubElement(geometry, obj['shape'], attrs)
            if kind == 'visual':
                material = ET.SubElement(element, 'material', {'name': name + '_material'})
                ET.SubElement(material, 'color', {'rgba': numbers(obj['rgba'])})
        joint = ET.SubElement(robot, 'joint', {'name': name + '_fixed', 'type': 'fixed'})
        ET.SubElement(joint, 'parent', {'link': world_frame})
        ET.SubElement(joint, 'child', {'link': name})
        ET.SubElement(joint, 'origin', {'xyz': numbers(obj['xyz']), 'rpy': numbers(obj['rpy'])})
