"""Independent URDF FK / MuJoCo checks for workcell placement and geometry."""
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import yaml
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.profiles import load_profile
from mujoco_ros2_manipulation.scenes import apply_scene, load_scene, scene_fingerprint
from tests.helpers import urdf_fk


class SceneTests(unittest.TestCase):
    def fixture(self, existing_world):
        robot = ET.fromstring('''<robot name="fixture">
          <link name="base_link"/><link name="tip"><inertial><mass value="1"/>
          <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>
          <collision><geometry><box size="0.1 0.1 0.2"/></geometry></collision></link>
          <joint name="axis" type="revolute"><parent link="base_link"/><child link="tip"/>
          <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="10" velocity="1"/></joint></robot>''')
        if existing_world:
            ET.SubElement(robot, 'link', {'name': 'world'})
            joint = ET.SubElement(robot, 'joint', {'name': 'original_mount', 'type': 'fixed'})
            ET.SubElement(joint, 'parent', {'link': 'world'})
            ET.SubElement(joint, 'child', {'link': 'base_link'})
            ET.SubElement(joint, 'origin', {'xyz': '0.01 0 0'})
        return robot

    def test_workcell_pose_and_primitives_match_independent_fk(self):
        for existing_world in (False, True):
            for scene_id in ('default', 'pedestal'):
                with self.subTest(world=existing_world, scene=scene_id):
                    scene = deepcopy(load_scene(scene_id, ROOT))
                    scene['robot_pose'] = {'xyz': [0.5, -0.2, 0.7], 'rpy': [0.1, -0.2, 0.3]}
                    scene['objects'].append(dict(name='cylinder', shape='cylinder', size=[0.1, 0.4],
                                                xyz=[1, 1, 0.2], rpy=[0, 0.2, 0.1], rgba=[1, 0, 0, 1]))
                    robot = self.fixture(existing_world)
                    apply_scene(robot, scene, 'world')
                    ET.SubElement(ET.SubElement(robot, 'mujoco'), 'compiler', {'fusestatic': 'false', 'discardvisual': 'false'})
                    model = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding='unicode'))
                    state = mujoco.MjData(model)
                    state.joint('axis').qpos[0] = 0.4
                    mujoco.mj_forward(model, state)
                    expected = urdf_fk(robot, {'axis': 0.4})
                    for name, pose in expected.items():
                        body = state.body(name)
                        np.testing.assert_allclose(body.xpos, pose[:3, 3], atol=1e-8)
                        np.testing.assert_allclose(body.xmat.reshape(3, 3), pose[:3, :3], atol=1e-8)
                    for obj in scene['objects']:
                        body_id = model.body('workcell_' + obj['name']).id
                        geoms = [i for i in range(model.ngeom) if model.geom_bodyid[i] == body_id and model.geom_contype[i]]
                        self.assertEqual(len(geoms), 1)
                        expected_size = np.array(obj['size']) / 2 if obj['shape'] == 'box' else [obj['size'][0], obj['size'][1]/2]
                        np.testing.assert_allclose(model.geom_size[geoms[0]][:len(expected_size)], expected_size)

    def test_invalid_scenes_fail_before_writing_models(self):
        original = load_scene('pedestal', ROOT)
        for case in ('nan', 'negative', 'duplicate', 'shape', 'typo', 'color'):
            scene = deepcopy(original)
            if case == 'nan': scene['robot_pose']['xyz'][0] = float('nan')
            elif case == 'negative': scene['objects'][0]['size'][0] = -1
            elif case == 'duplicate': scene['objects'].append(scene['objects'][0])
            elif case == 'shape': scene['objects'][0]['shape'] = 'mesh'
            elif case == 'typo': scene['objects'][0]['rpg'] = [0, 0, 0]
            else: scene['objects'][0]['rgba'][0] = 2
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp); (root / 'config/scenes').mkdir(parents=True)
                (root / 'config/scenes/pedestal.yaml').write_text(yaml.safe_dump(scene))
                with self.assertRaises(ValueError): load_scene('pedestal', root)
        for bad in ('../escape', '/etc/passwd', 'missing'):
            with self.assertRaises(ValueError): load_scene(bad, ROOT)

    def test_scene_outputs_and_controller_paths_do_not_overlap(self):
        from mujoco_ros2_manipulation.profiles import write_controller_config
        default = load_profile('fr5')
        raised = load_profile('fr5', scene='pedestal')
        self.assertEqual(default.fingerprint, raised.fingerprint)
        self.assertNotEqual(default.output, raised.output)
        self.assertNotEqual(scene_fingerprint(default.scene_data), scene_fingerprint(raised.scene_data))
        with tempfile.TemporaryDirectory() as tmp:
            from dataclasses import replace
            (Path(tmp) / 'config').mkdir()
            (Path(tmp) / 'config/controller_defaults.yaml').write_bytes((ROOT / 'config/controller_defaults.yaml').read_bytes())
            self.assertNotEqual(write_controller_config(replace(default, root=Path(tmp))),
                                write_controller_config(replace(raised, root=Path(tmp))))

    def test_reserved_scene_names_do_not_overwrite_robot_links(self):
        robot = self.fixture(False)
        ET.SubElement(robot, 'link', {'name': 'workcell_floor'})
        with self.assertRaises(ValueError): apply_scene(robot, load_scene('default', ROOT), 'world')
