"""Tiny CPU fixtures only; no ROS process, GL renderer, or repository artifact."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

from mujoco_ros2_manipulation.operator_console import scene_view
from mujoco_ros2_manipulation.operator_console.scene_view import CompiledScene, SceneLimits


TETRA = '''<asset><mesh name="tetra" vertex="1 0 0 2 0 0 1 1 0 1 0 1"
 face="0 2 1 0 1 3 0 3 2 1 2 3"/></asset>'''


class CompiledSceneTests(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.addCleanup(self.directory.cleanup)

    def scene(self, source, names=(), home=None, **kwargs):
        path = self.root / "scene.xml"
        path.write_text(source)
        return CompiledScene(path, names, {} if home is None else home, **kwargs)

    def test_import_does_not_require_mujoco_numpy_ros_or_gl_runtime(self):
        module = str(Path(scene_view.__file__).resolve())
        source = '''import importlib.util,sys
for name in ('mujoco','numpy','rclpy','OpenGL'):
    sys.modules[name]=None
spec=importlib.util.spec_from_file_location('isolated_scene_view',sys.argv[1])
module=importlib.util.module_from_spec(spec)
sys.modules[spec.name]=module
spec.loader.exec_module(module)
assert module.EXPORT_VERSION=='compiled-scene-v1'
'''
        subprocess.run([sys.executable, "-c", source, module], check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def test_primitive_contract_half_sizes_groups_and_effective_material_rgba(self):
        scene = self.scene('''<mujoco><asset><material name="blue" rgba=".1 .2 .9 .8"/></asset>
          <worldbody><geom name="floor" type="plane" size="0 0 .1"/>
          <geom name="box" type="box" size=".1 .2 .3" group="3"/>
          <geom name="sphere" type="sphere" size=".2" group="1" material="blue"/>
          <geom name="cylinder" type="cylinder" size=".2 .4" group="2"/>
          <geom name="capsule" type="capsule" size=".1 .5"/>
          <geom name="hidden" type="sphere" size=".1" rgba="0 1 0 0"/>
          </worldbody></mujoco>''')
        manifest = scene.manifest()
        self.assertEqual(manifest["default_visible_groups"], [0, 1, 2])
        self.assertEqual(manifest["units"], dict(length="m", angle="rad", up_axis="Z", rotation="row-major"))
        by_name = {row["name"]: row for row in manifest["geoms"]}
        self.assertEqual(by_name["box"]["size"], [.1, .2, .3])
        self.assertFalse(by_name["box"]["default_visible"])
        self.assertTrue(by_name["sphere"]["default_visible"])
        self.assertEqual(by_name["cylinder"]["size"], [.2, .4, 0])
        for observed, expected in zip(by_name["sphere"]["rgba"], [.1, .2, .9, .8]):
            self.assertAlmostEqual(observed, expected, places=6)
        self.assertEqual(by_name["hidden"]["rgba"][-1], 0)
        self.assertEqual(manifest["assets"], [])
        self.assertNotIn(str(self.root), json.dumps(manifest, allow_nan=False))

    def test_compiled_mesh_vertices_and_geom_pose_recover_author_coordinates(self):
        scene = self.scene(f'<mujoco>{TETRA}<worldbody><geom type="mesh" mesh="tetra"/></worldbody></mujoco>')
        manifest = scene.manifest()
        asset = manifest["assets"][0]
        blob = scene.asset(asset["sha256"])
        self.assertEqual(hashlib.sha256(blob).hexdigest(), asset["sha256"])
        self.assertEqual(len(blob), asset["byte_length"])
        self.assertIsInstance(blob, bytes)
        position_layout = asset["layout"]["positions"]
        index_layout = asset["layout"]["indices"]
        self.assertEqual(index_layout["byte_offset"], position_layout["byte_length"])
        xyz = struct.unpack_from('<'+'f' * position_layout["count"]*3, blob, position_layout["byte_offset"])
        indices = struct.unpack_from('<'+'I' * index_layout["count"], blob, index_layout["byte_offset"])
        self.assertTrue(all(0 <= item < position_layout["count"] for item in indices))
        pose = scene.pose({})["geoms"][0]
        matrix, offset = pose["rotation"], pose["position"]
        world = [[sum(matrix[row*3+column]*xyz[vertex*3+column] for column in range(3))+offset[row]
                  for row in range(3)] for vertex in range(position_layout["count"])]
        for axis, expected in enumerate(((1, 2), (0, 1), (0, 1))):
            self.assertAlmostEqual(min(row[axis] for row in world), expected[0], places=6)
            self.assertAlmostEqual(max(row[axis] for row in world), expected[1], places=6)
        self.assertEqual(manifest["geoms"][0]["asset_sha256"], asset["sha256"])

    def test_pose_is_full_named_forward_kinematics_and_independent_between_calls(self):
        scene = self.scene('''<mujoco><worldbody><body pos="1 2 0"><joint name="q" axis="0 0 1"/>
          <geom name="tip" type="box" size=".1 .1 .1" pos="1 0 0"/></body></worldbody></mujoco>''', ["q"], {"q": 0})
        state = {"q": math.pi/2}
        first = scene.pose(state)
        original = deepcopy(first)
        zero = scene.pose({"q": 0})
        self.assertEqual(first, original)
        for actual, expected in zip(first["geoms"][0]["position"], [1, 3, 0]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(zero["geoms"][0]["position"], [2, 2, 0]):
            self.assertAlmostEqual(actual, expected)
        first["geoms"][0]["position"][0] = 99
        self.assertEqual(scene.pose(state), original)
        self.assertEqual(state, {"q": math.pi/2})
        for invalid in ({}, {"q": 0, "other": 0}, {"q": True}, {"q": float("nan")}, {"q": float("inf")}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                scene.pose(invalid)

    def test_identity_includes_compiled_includes_and_detached_manifest(self):
        source = '<mujoco><include file="body.xml"/></mujoco>'
        included = self.root / "body.xml"
        included.write_text('<mujoco><worldbody><geom type="sphere" size=".1" pos="1 0 0"/></worldbody></mujoco>')
        first = self.scene(source)
        initial = first.manifest()
        included.write_text('<mujoco><worldbody><geom type="sphere" size=".1" pos="2 0 0"/></worldbody></mujoco>')
        second = self.scene(source)
        self.assertEqual(first.model_sha256, second.model_sha256)
        self.assertNotEqual(initial["compiled_model_sha256"], second.manifest()["compiled_model_sha256"])
        self.assertNotEqual(first.scene_sha256, second.scene_sha256)
        initial["geoms"].clear()
        self.assertEqual(len(first.manifest()["geoms"]), 1)
        canonical = first.manifest()
        claimed = canonical.pop("scene_sha256")
        encoded = json.dumps(canonical, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode()
        self.assertEqual(hashlib.sha256(encoded).hexdigest(), claimed)
        self.assertEqual(first.pose({})["scene_sha256"], claimed)

    def test_unobserved_or_nonscalar_joints_and_unsupported_geometry_fail_closed(self):
        scalar = '<mujoco><worldbody><body><joint name="q"/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
        with self.assertRaisesRegex(ValueError, "Every model joint"):
            self.scene(scalar)
        with self.assertRaisesRegex(ValueError, "missing a named joint"):
            self.scene(scalar, ["other"], {"other": 0})
        for joint in ('<freejoint name="q"/>', '<joint name="q" type="ball"/>'):
            with self.subTest(joint=joint), self.assertRaisesRegex(ValueError, "scalar"):
                self.scene(f'<mujoco><worldbody><body>{joint}<geom type="sphere" size=".1"/></body></worldbody></mujoco>', ["q"], {"q": 0})
        with self.assertRaisesRegex(ValueError, "Unsupported geometry"):
            self.scene('<mujoco><worldbody><geom type="ellipsoid" size=".1 .2 .3"/></worldbody></mujoco>')

    def test_limits_and_owned_asset_lookup_fail_closed(self):
        with self.assertRaisesRegex(ValueError, "positive integer"):
            SceneLimits(max_geoms=True)
        source = '<mujoco><worldbody><geom type="sphere" size=".1"/><geom type="sphere" size=".2"/></worldbody></mujoco>'
        with self.assertRaisesRegex(ValueError, "resource limits"):
            self.scene(source, limits=SceneLimits(max_geoms=1))
        with self.assertRaisesRegex(ValueError, "resource limits"):
            self.scene(source, limits=SceneLimits(max_xml_bytes=1))
        scene = self.scene('<mujoco><worldbody/></mujoco>')
        for invalid in ('../secret', 'A'*64, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                scene.asset(invalid)
        with self.assertRaises(KeyError):
            scene.asset('0'*64)


if __name__ == '__main__':
    unittest.main()
