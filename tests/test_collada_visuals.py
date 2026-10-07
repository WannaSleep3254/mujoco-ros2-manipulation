"""Visual scene fidelity and physical-model regression checks."""

from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import collada
import mujoco
import numpy as np

from collada_visuals import export_dae_visuals
from prepare_robot import prepare
from robot_config import ROOT, load_profile
from test_robot_profiles import TemporaryOutputProfile, axis_rotation, urdf_fk


class ColladaTests(unittest.TestCase):
    def test_scene_units_transforms_colors_and_normals_are_preserved(self):
        document = collada.Collada()
        document.assetInfo.unitmeter = 0.001
        document.assetInfo.unitname = 'millimeter'
        document.assetInfo.upaxis = 'Z_UP'
        points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
        vertices = collada.source.FloatSource('vertices', points, ('X', 'Y', 'Z'))
        normals = collada.source.FloatSource('normals', np.ones((4, 3)), ('X', 'Y', 'Z'))
        geometry = collada.geometry.Geometry(document, 'geometry', 'test geometry', [vertices, normals])
        inputs = collada.source.InputList()
        inputs.addInput(0, 'VERTEX', '#vertices')
        inputs.addInput(1, 'NORMAL', '#normals')
        faces = np.array([[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]])
        colors = [(0.8, 0.1, 0.1, 1.0), (0.1, 0.2, 0.8, 1.0)]
        material_nodes = []
        for index, color in enumerate(colors):
            symbol = f'color{index}'
            effect = collada.material.Effect(f'effect{index}', [], 'phong', diffuse=color)
            material = collada.material.Material(f'material{index}', symbol, effect)
            document.effects.append(effect)
            document.materials.append(material)
            indices = np.stack((faces[index*2:index*2+2], faces[index*2:index*2+2]), axis=-1).ravel()
            geometry.primitives.append(geometry.createTriangleSet(indices, inputs, symbol))
            material_nodes.append(collada.scene.MaterialNode(symbol, material, inputs=[]))
        document.geometries.append(geometry)
        node = collada.scene.Node('scaled', children=[collada.scene.GeometryNode(geometry, material_nodes)],
            transforms=[collada.scene.TranslateTransform(10, 20, 30),
                        collada.scene.ScaleTransform(-2, 3, 4)])
        scene = collada.scene.Scene('scene', [node])
        document.scenes.append(scene)
        document.scene = scene
        with tempfile.TemporaryDirectory() as temporary:
            source = Path(temporary) / 'scene.dae'
            with source.open('wb') as output:
                document.write(output)
            parts = export_dae_visuals(source, temporary, 'test')
            self.assertEqual([part.rgba for part in parts], colors)
            self.assertEqual(sum(part.triangles for part in parts), 4)
            expected_vertices = (points * [-2, 3, 4] + [10, 20, 30]) * 0.001
            expected_normal = np.array([-0.5, 1/3, 0.25])
            expected_normal /= np.linalg.norm(expected_normal)
            for part in parts:
                rows = [line.split() for line in part.path.read_text().splitlines()]
                actual_vertices = np.array([[float(v) for v in row[1:]] for row in rows if row[0] == 'v'])
                actual_normals = np.array([[float(v) for v in row[1:]] for row in rows if row[0] == 'vn'])
                np.testing.assert_allclose(actual_vertices, expected_vertices, atol=1e-9, rtol=0)
                np.testing.assert_allclose(actual_normals, np.tile(expected_normal, (4, 1)), atol=1e-8, rtol=0)
                first_face = next(row for row in rows if row[0] == 'f')
                face = [int(value.split('//')[0])-1 for value in first_face[1:]]
                expected_face = faces[colors.index(part.rgba)*2][::-1]
                np.testing.assert_array_equal(face, expected_face)


@unittest.skipUnless((ROOT / 'models/ur5e/ur5e.xml').exists(), 'generated UR5e model is required')
class UR5eVisualTests(unittest.TestCase):
    def test_visual_surface_bounds_match_original_dae_in_robot_poses(self):
        profile = load_profile('ur5e')
        self.assertEqual(profile.data['source']['mujoco_visuals'], 'dae')
        robot = ET.parse(profile.urdf_path).getroot()
        model = mujoco.MjModel.from_xml_path(str(profile.scene_path))
        state = mujoco.MjData(model)
        source_points = {}
        for link in robot.findall('link'):
            visual = link.find('visual')
            if visual is None:
                continue
            uri = visual.find('geometry/mesh').get('filename')
            path = ROOT / 'external/ur_description' / uri.removeprefix('package://ur_description/')
            document = collada.Collada(str(path))
            points = []
            for geometry in document.scene.objects('geometry'):
                for primitive in geometry.primitives():
                    if isinstance(primitive, collada.lineset.BoundLineSet):
                        continue
                    if isinstance(primitive, collada.polylist.BoundPolylist):
                        primitive = primitive.triangleset()
                    points.append(primitive.vertex[primitive.vertex_index].reshape(-1, 3))
            origin = visual.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
            rotation = (axis_rotation([0, 0, 1], rpy[2]) @ axis_rotation([0, 1, 0], rpy[1]) @
                        axis_rotation([1, 0, 0], rpy[0]))
            source_points[link.get('name')] = np.vstack(points) @ rotation.T + xyz
        for pose in [profile.home, profile.data['verification']['direct_target_rad'],
                     profile.data['verification']['moveit_target_rad']]:
            coordinates = dict(zip(profile.joints, pose))
            for name, value in coordinates.items():
                state.joint(name).qpos[0] = value
            mujoco.mj_forward(model, state)
            transforms = urdf_fk(robot, coordinates)
            for link, points in source_points.items():
                with self.subTest(link=link, pose=pose):
                    expected = points @ transforms[link][:3, :3].T + transforms[link][:3, 3]
                    actual = []
                    body_id = model.body(link).id
                    for geom_id in range(model.ngeom):
                        if model.geom_bodyid[geom_id] != body_id or model.geom_group[geom_id] != 2:
                            continue
                        mesh_id = model.geom_dataid[geom_id]
                        start = model.mesh_vertadr[mesh_id]
                        vertices = model.mesh_vert[start:start+model.mesh_vertnum[mesh_id]]
                        actual.append(vertices @ state.geom_xmat[geom_id].reshape(3, 3).T + state.geom_xpos[geom_id])
                    actual = np.vstack(actual)
                    np.testing.assert_allclose([actual.min(0), actual.max(0)],
                                               [expected.min(0), expected.max(0)], atol=1e-6, rtol=0)

    def test_visual_replacement_preserves_dynamics_and_collisions(self):
        profile = load_profile('ur5e')
        current = mujoco.MjModel.from_xml_path(str(profile.scene_path))
        baseline_data = deepcopy(profile.data)
        baseline_data['source']['mujoco_visuals'] = 'collision'
        with tempfile.TemporaryDirectory() as temporary:
            baseline_profile = TemporaryOutputProfile(ROOT, baseline_data, Path(temporary))
            with patch('prepare_robot.write_controller_config'), patch('builtins.print'):
                prepare(baseline_profile)
            baseline = mujoco.MjModel.from_xml_path(str(baseline_profile.scene_path))
            for field in ['body_pos', 'body_quat', 'body_mass', 'body_inertia', 'dof_armature', 'dof_damping',
                          'actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange']:
                np.testing.assert_allclose(getattr(current, field), getattr(baseline, field), atol=1e-10, rtol=0)
            for body_name in ['base_link_inertia', 'shoulder_link', 'upper_arm_link', 'forearm_link',
                              'wrist_1_link', 'wrist_2_link', 'wrist_3_link']:
                collision_geoms = []
                for model in [current, baseline]:
                    body_id = model.body(body_name).id
                    geoms = [i for i in range(model.ngeom)
                             if model.geom_bodyid[i] == body_id and model.geom_contype[i]]
                    self.assertEqual(len(geoms), 1)
                    collision_geoms.append(geoms[0])
                for field in ['geom_pos', 'geom_quat', 'geom_friction', 'geom_solref', 'geom_solimp',
                              'geom_contype', 'geom_conaffinity']:
                    np.testing.assert_allclose(getattr(current, field)[collision_geoms[0]],
                                               getattr(baseline, field)[collision_geoms[1]], atol=1e-10, rtol=0)
                for field in ['vert', 'face']:
                    arrays = []
                    for model, geom_id in zip([current, baseline], collision_geoms):
                        mesh_id = model.geom_dataid[geom_id]
                        start = getattr(model, f'mesh_{field}adr')[mesh_id]
                        count = getattr(model, f'mesh_{field}num')[mesh_id]
                        arrays.append(getattr(model, f'mesh_{field}')[start:start+count])
                    np.testing.assert_array_equal(arrays[0], arrays[1])
            for pose in [profile.home, profile.data['verification']['moveit_target_rad']]:
                states = []
                mass_matrices = []
                for model in [current, baseline]:
                    state = mujoco.MjData(model)
                    for name, value in zip(profile.joints, pose):
                        state.joint(name).qpos[0] = value
                    mujoco.mj_forward(model, state)
                    states.append(state)
                    mass = np.empty((model.nv, model.nv))
                    mujoco.mj_fullM(model, state, mass)
                    mass_matrices.append(mass)
                np.testing.assert_allclose(mass_matrices[0], mass_matrices[1], atol=1e-10, rtol=0)
                np.testing.assert_allclose(states[0].qfrc_bias, states[1].qfrc_bias, atol=1e-10, rtol=0)
                for model in [current, baseline]:
                    collision_meshes = [model.mesh(model.geom_dataid[i]).name
                        for i in range(model.ngeom)
                        if model.geom_contype[i] and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH]
                    self.assertEqual(sorted(collision_meshes),
                                     sorted(['base', 'shoulder', 'upperarm', 'forearm', 'wrist1', 'wrist2', 'wrist3']))


if __name__ == '__main__':
    unittest.main()
