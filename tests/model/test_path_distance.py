"""Pinned native engine distances against analytic geometry, independent of robot meshes."""
from pathlib import Path
from contextlib import contextmanager
import ctypes as C
import shutil
from types import SimpleNamespace as NS
import tempfile
import unittest
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator


class NativeDistanceTests(unittest.TestCase):
    def test_signed_distance_nearest_points_frame_fk_and_visual_geometry_filter(self):
        if not (ROOT / "runtime/native/path_distance.so").is_file():
            self.skipTest("Native distance evaluator not built")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / "cell.xml").write_text(
                """<mujoco><worldbody>
              <body name="left_base"><joint name="l" type="slide" axis="1 0 0"/>
                <geom type="sphere" size=".1"/><body name="left_tip" pos=".1 0 0"/>
                <geom type="sphere" size="2" contype="0" conaffinity="0" mass="0"/></body>
              <body name="right_base" pos="1 0 0"><joint name="r" type="slide" axis="1 0 0"/>
                <geom type="sphere" size=".1"/><body name="right_tip" pos="-.1 0 0"/></body>
              </worldbody></mujoco>"""
            )
            instances = [
                NS(prefix=p, profile=NS(ros=dict(base_frame="base", tip_frame="tip")))
                for p in ("left_", "right_")
            ]
            cell = NS(root=ROOT, output=path, joints=["l", "r"], instances=instances)
            with DistanceEvaluator(cell) as evaluator:
                initial = evaluator.at(dict(l=0.0, r=0.0))
                self.assertAlmostEqual(initial["distance_m"], 0.8)
                self.assertAlmostEqual(initial["nearest_points_m"][0][0], 0.1)
                self.assertAlmostEqual(initial["nearest_points_m"][1][0], 0.9)
                self.assertEqual(initial["bodies"], ["left_base", "right_base"])
                self.assertAlmostEqual(
                    evaluator.at(dict(l=0.25, r=0.0))["tips_m"][0][0], 0.35
                )
                self.assertAlmostEqual(
                    evaluator.at(dict(l=0.9, r=0.0))["distance_m"], -0.1
                )
                with self.assertRaises(ValueError):
                    evaluator.at(dict(l=float("nan"), r=0.0))
                with self.assertRaises(ValueError):
                    evaluator.at(dict(l=0.0))
                self.assertEqual(evaluator.version, "3.12.0")
            with self.assertRaises(ValueError):
                evaluator.at(dict(l=0.0, r=0.0))

    def test_saved_ci_grazing_spline_is_rejected_before_execution(self):
        import gzip
        import json
        from mujoco_ros2_manipulation.workcells.config import load_workcell
        from mujoco_ros2_manipulation.workcells.interference import Spline
        from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path
        from mujoco_ros2_manipulation.workcells.timing import load_policy

        cell = load_workcell()
        if (
            not (ROOT / "runtime/native/path_distance.so").is_file()
            or not (cell.output / "cell.xml").is_file()
        ):
            self.skipTest("Native runtime and shared robot model not built")
        evidence = json.loads(
            gzip.decompress(
                (
                    ROOT / "docs/timing_coordination/ci-grazing-trajectories.json.gz"
                ).read_bytes()
            )
        )
        curve = Spline(**evidence["coordinated"])
        _, policy = load_policy(cell)
        threshold = policy["min_inter_arm_distance_m"] + policy["tracking_reserve_m"]
        with DistanceEvaluator(cell) as evaluator:
            full = evaluate_path([curve], evidence["initial_rad"], cell, evaluator)
            gate = evaluate_path(
                [curve],
                evidence["initial_rad"],
                cell,
                evaluator,
                reject_below=threshold,
            )
        self.assertGreater(full["minimum"]["distance_m"], 0.0)
        self.assertLess(full["minimum"]["distance_m"], 0.001)
        self.assertEqual(
            full["minimum"]["bodies"], ["left_wrist2_link", "right_wrist_3_link"]
        )
        self.assertFalse(gate["complete"])
        self.assertLess(gate["minimum"]["distance_m"], threshold)
        self.assertLess(gate["samples"], full["samples"])


class NativeEnvironmentDistanceTests(unittest.TestCase):
    @contextmanager
    def cell(self, environment="", *, contact="", bodies="", joints=None,
             mounted=False, assets=""):
        if not (ROOT / "runtime/native/path_distance.so").is_file():
            self.skipTest("Native distance evaluator not built")
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            if mounted:
                robot = '''<body name="left_base"><geom name="left_mount" type="sphere" size=".5"/>
                    <body name="left_moving" pos="0 0 1"><joint name="l" type="slide" axis="1 0 0"/>
                      <geom name="left_geom" type="sphere" size=".1"/><body name="left_tip"/>
                    </body></body>
                  <body name="right_base" pos="2 0 0"><geom name="right_mount" type="sphere" size=".5"/>
                    <body name="right_moving" pos="0 0 1"><joint name="r" type="slide" axis="1 0 0"/>
                      <geom name="right_geom" type="sphere" size=".1"/><body name="right_tip"/>
                    </body></body>'''
            else:
                robot = '''<body name="left_base"><joint name="l" type="slide" axis="1 0 0"/>
                    <geom name="left_geom" type="sphere" size=".1"/><body name="left_tip" pos=".1 0 0"/>
                  </body>
                  <body name="right_base" pos="1 0 0"><joint name="r" type="slide" axis="1 0 0"/>
                    <geom name="right_geom" type="sphere" size=".1"/><body name="right_tip" pos="-.1 0 0"/>
                  </body>'''
            (directory / "cell.xml").write_text(
                '<mujoco>' + assets + '<worldbody>' + robot + environment + bodies +
                '</worldbody>' + contact + '</mujoco>')
            instances = [NS(prefix=prefix, profile=NS(ros=dict(base_frame="base", tip_frame="tip")))
                         for prefix in ("left_", "right_")]
            yield NS(root=ROOT, output=directory, joints=joints or ["l", "r"], instances=instances)

    def test_world_body_and_fixed_body_distance_penetration_and_default_compatibility(self):
        world = '<geom name="wall" type="sphere" pos="2 0 0" size=".1"/>'
        fixed = '<body name="fixed_wall" pos="2 0 0"><geom name="wall" type="sphere" size=".1"/></body>'
        for environment, body in ((world, "world"), (fixed, "fixed_wall")):
            with self.subTest(owner=body), self.cell(environment) as cell:
                with DistanceEvaluator(cell) as plain, DistanceEvaluator(cell, environment=True) as evaluator:
                    initial = evaluator.at(dict(l=0.0, r=0.0))
                    environment_result = initial.pop("environment")
                    self.assertEqual(initial, plain.at(dict(l=0.0, r=0.0)))
                    self.assertNotIn("environment", plain.metadata())
                    self.assertAlmostEqual(environment_result["distance_m"], .8)
                    self.assertEqual(environment_result["bodies"], ["right_base", body])
                    self.assertAlmostEqual(environment_result["nearest_points_m"][0][0], 1.1)
                    self.assertAlmostEqual(environment_result["nearest_points_m"][1][0], 1.9)
                    self.assertEqual(environment_result["pair_count"], 2)
                    self.assertEqual(environment_result["scope"], "moving_robot_vs_fixed_environment")
                    self.assertAlmostEqual(evaluator.at(dict(l=0.0, r=.75))["environment"]["distance_m"], .05)
                    self.assertAlmostEqual(evaluator.at(dict(l=0.0, r=.9))["environment"]["distance_m"], -.1)
                    metadata = evaluator.metadata()["environment"]
                    self.assertEqual(metadata["moving_geom_count_by_arm"], [1, 1])
                    self.assertEqual(metadata["fixed_geom_count"], 1)
                    self.assertEqual(metadata["pair_count"], 2)

    def test_visual_masks_and_native_body_excludes_do_not_create_false_obstacles(self):
        environment = '''<body name="safe_wall" pos="3 0 0"><geom type="sphere" size=".1"/></body>
          <body name="excluded_wall" pos="1.1 0 0"><geom type="sphere" size=".1"/></body>
          <geom name="masked" type="sphere" pos="1.05 0 0" size="2" contype="2" conaffinity="2"/>
          <geom name="visual" type="sphere" pos="1 0 0" size="3" contype="0" conaffinity="0"/>'''
        contact = '''<contact><exclude body1="left_base" body2="excluded_wall"/>
          <exclude body1="right_base" body2="excluded_wall"/></contact>'''
        with self.cell(environment, contact=contact) as cell:
            with DistanceEvaluator(cell, environment=True) as evaluator:
                result = evaluator.at(dict(l=0.0, r=0.0))["environment"]
                self.assertAlmostEqual(result["distance_m"], 1.8)
                self.assertEqual(result["bodies"], ["right_base", "safe_wall"])
                self.assertEqual(result["pair_count"], 2)
                self.assertEqual(evaluator.metadata()["environment"]["fixed_geom_count"], 3)

    def test_only_static_robot_mounts_are_excluded_and_both_arms_are_measured(self):
        # The static mounting sphere intersects the fixed obstacle. The moving
        # shoulder remains .8m away; omitting the mount exclusion would be negative.
        environment = '<geom name="obstacle" type="sphere" size=".1"/>'
        with self.cell(environment, mounted=True) as cell:
            with DistanceEvaluator(cell, environment=True) as evaluator:
                result = evaluator.at(dict(l=0.0, r=0.0))["environment"]
                self.assertAlmostEqual(result["distance_m"], .8)
                self.assertEqual(result["bodies"], ["left_moving", "world"])
                self.assertEqual(evaluator.metadata()["environment"]["moving_geom_count_by_arm"], [1, 1])
        with self.cell('<geom type="sphere" pos=".5 0 0" size=".1"/>') as cell:
            with DistanceEvaluator(cell, environment=True) as evaluator:
                left = evaluator.at(dict(l=.25, r=0.0))["environment"]
                right = evaluator.at(dict(l=0.0, r=-.25))["environment"]
                self.assertEqual(left["bodies"][0], "left_base")
                self.assertEqual(right["bodies"][0], "right_base")
                self.assertAlmostEqual(left["distance_m"], .05)
                self.assertAlmostEqual(right["distance_m"], .05)

    def test_no_environment_and_no_eligible_pairs_are_explicit_errors(self):
        for environment, message in (
            ("", "No fixed environment"),
            ('<geom type="sphere" pos="2 0 0" size=".1" contype="2" conaffinity="2"/>', "No eligible environment"),
        ):
            with self.subTest(message=message), self.cell(environment) as cell:
                with DistanceEvaluator(cell) as plain:
                    self.assertAlmostEqual(plain.at(dict(l=0.0, r=0.0))["distance_m"], .8)
                with self.assertRaisesRegex(ValueError, message):
                    DistanceEvaluator(cell, environment=True)

    def test_outside_moving_body_mocap_and_unsupported_shapes_are_refused(self):
        wall = '<geom name="wall" type="sphere" pos="3 0 0" size=".1"/>'
        movable = '<body name="outside" pos="5 0 0"><joint name="e" type="slide"/><geom type="sphere" size=".1"/></body>'
        with self.cell(wall, bodies=movable, joints=["l", "r", "e"]) as cell:
            with self.assertRaisesRegex(ValueError, "Movable geometry outside"):
                DistanceEvaluator(cell, environment=True)
        mocap = '<body name="outside" mocap="true" pos="5 0 0"><geom type="sphere" size=".1"/></body>'
        with self.cell(wall, bodies=mocap) as cell:
            with self.assertRaisesRegex(ValueError, "Movable geometry outside"):
                DistanceEvaluator(cell, environment=True)
        hfield = '<geom type="hfield" hfield="terrain"/>'
        assets = '<asset><hfield name="terrain" nrow="2" ncol="2" size="1 1 .1 .1"/></asset>'
        with self.cell(hfield, assets=assets) as cell:
            with self.assertRaisesRegex(ValueError, "Unsupported collision geometry"):
                DistanceEvaluator(cell, environment=True)
        pair = '<contact><pair geom1="left_geom" geom2="wall"/></contact>'
        with self.cell(wall, contact=pair) as cell:
            with self.assertRaisesRegex(ValueError, "explicit contact geom pairs"):
                DistanceEvaluator(cell, environment=True)

    def test_environment_cannot_read_an_uninitialized_or_failed_fk_snapshot(self):
        with self.cell('<geom type="sphere" pos="2 0 0" size=".1"/>') as cell:
            with DistanceEvaluator(cell, environment=True) as evaluator:
                output = (C.c_double * 7)()
                pair = (C.c_int * 2)()
                self.assertEqual(evaluator.api.path_distance_environment_eval(evaluator.handle, output, pair), 0)
                result = evaluator.at(dict(l=0.0, r=0.0))["environment"]
                self.assertEqual(evaluator.api.path_distance_environment_eval(evaluator.handle, output, pair), 1)
                self.assertEqual(output[0], result["distance_m"])
                q = (C.c_double * 2)(float("nan"), 0)
                cross = (C.c_double * 13)()
                self.assertEqual(evaluator.api.path_distance_eval(evaluator.handle, q, 2, cross, pair), 0)
                self.assertEqual(evaluator.api.path_distance_environment_eval(evaluator.handle, output, pair), 0)
                for state in (dict(l=float("inf"), r=0.0), dict(l=0.0)):
                    with self.assertRaises(ValueError):
                        evaluator.at(state)
            with self.assertRaises(ValueError):
                evaluator.at(dict(l=0.0, r=0.0))
        with self.assertRaisesRegex(ValueError, "boolean"):
            DistanceEvaluator(None, environment=1)

    def test_extension_source_hash_is_independent_and_close_removes_cached_coverage(self):
        with self.cell('<geom type="sphere" pos="2 0 0" size=".1"/>') as cell:
            alias = cell.output / 'other-root'
            (alias / 'native').mkdir(parents=True)
            (alias / 'runtime/native').mkdir(parents=True)
            shutil.copy2(ROOT / 'native/path_distance.cpp', alias / 'native/path_distance.cpp')
            (alias / 'native/path_distance_environment.cpp').write_text('changed extension')
            (alias / 'runtime/native/path_distance.so').symlink_to(ROOT / 'runtime/native/path_distance.so')
            changed = NS(root=alias, output=cell.output, joints=cell.joints, instances=cell.instances)
            # The historical/default base source contract still accepts its
            # unchanged source. Only an explicit environment query needs the
            # new translation unit's independent provenance to match.
            with DistanceEvaluator(changed) as evaluator:
                self.assertAlmostEqual(evaluator.at(dict(l=0.0, r=0.0))["distance_m"], .8)
            with self.assertRaisesRegex(ValueError, "Stale environment distance"):
                DistanceEvaluator(changed, environment=True)
            evaluator = DistanceEvaluator(cell, environment=True)
            handle = evaluator.handle
            evaluator.at(dict(l=0.0, r=0.0))
            self.assertEqual(evaluator.metadata()["environment"]["source_sha256"],
                             evaluator.environment_source_sha256)
            evaluator.close()
            output, pair = (C.c_double * 7)(), (C.c_int * 2)()
            self.assertEqual(evaluator.api.path_distance_environment_eval(handle, output, pair), 0)

    def test_environment_pair_capacity_is_rejected_before_accepting_coverage(self):
        # 66 moving geoms x 63 fixed geoms = 4158 eligible pairs. A fixed
        # boundary prevents an accidental large model from expanding work until
        # Python notices the count only after native construction has finished.
        environment = ''.join(f'<geom type="sphere" pos="{10 + j} 0 0" size=".01"/>'
                              for j in range(63))
        with self.cell(environment) as cell:
            path = cell.output / 'cell.xml'
            xml = ET.parse(path)
            for name in ('left_base', 'right_base'):
                body = xml.find(f"worldbody/body[@name='{name}']")
                for index in range(32):
                    ET.SubElement(body, 'geom', dict(type='sphere', size='.01',
                        pos=f'0 0 {index / 100}'))
            xml.write(path)
            with self.assertRaisesRegex(ValueError, "exceeds 4096"):
                DistanceEvaluator(cell, environment=True)
