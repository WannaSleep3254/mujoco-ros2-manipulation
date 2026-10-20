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

from mujoco_ros2_manipulation.model_build.collada import export_dae_visuals
from mujoco_ros2_manipulation.model_build.prepare import prepare
from mujoco_ros2_manipulation.profiles import ROOT, load_profile
from tests.helpers import TemporaryOutputProfile, axis_rotation, urdf_fk


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
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            uri = mesh.get('filename')
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
            with patch('mujoco_ros2_manipulation.model_build.prepare.write_controller_config'), patch('builtins.print'):
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
