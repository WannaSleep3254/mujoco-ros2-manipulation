"""Native 3.12 full-state clearance against independent engine contacts.

All qpos edits below are isolated geometric fault inputs. They never write a
running simulation or claim that virtual held geometry is a physical grasp.
"""
from contextlib import contextmanager
from copy import deepcopy
import ctypes as C
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.grasping.ros_model import build_model
from mujoco_ros2_manipulation.grasping.ros_target import load_ros_grasp_target
from mujoco_ros2_manipulation.grasping.ros_distance import GraspDistanceEvaluator


# Use the native collision engine as an independent contact oracle, rather than
# restating the evaluator's pair-selection implementation in Python.
INSPECT = r'''
#include <mujoco/mujoco.h>
#include <iostream>
#include <iomanip>
#include <fstream>
int main(int argc,char**argv) {
  if(argc<2)return 2;
  char error[2048]{};
  mjModel*m=mj_loadXML(argv[1],nullptr,error,2048);
  if(!m){std::cerr<<error;return 3;}
  mjData*d=mj_makeData(m);
  if(m->nkey)mj_resetDataKeyframe(m,d,0);
  if(argc>2){std::ifstream input(argv[2]);for(int i=0;i<m->nq;i++)if(!(input>>d->qpos[i]))return 4;}
  mj_forward(m,d);
  std::cout<<std::setprecision(17)<<"{\"native_version\":\""<<mj_versionString()<<"\",\"qpos\":[";
  for(int i=0;i<m->nq;i++){if(i)std::cout<<',';std::cout<<d->qpos[i];}
  std::cout<<"],\"geoms\":[";
  for(int i=0;i<m->ngeom;i++) {
    if(i)std::cout<<',';
    const char*n=mj_id2name(m,mjOBJ_GEOM,i);
    std::cout<<"{\"id\":"<<i<<",\"name\":\""<<(n?n:"")<<"\",\"body\":\""
      <<mj_id2name(m,mjOBJ_BODY,m->geom_bodyid[i])<<"\",\"position\":["
      <<d->geom_xpos[3*i]<<','<<d->geom_xpos[3*i+1]<<','<<d->geom_xpos[3*i+2]<<"]}";
  }
  std::cout<<"],\"contacts\":[";
  for(int i=0;i<d->ncon;i++) {
    if(i)std::cout<<',';
    auto&c=d->contact[i];
    std::cout<<"{\"geoms\":["<<c.geom[0]<<','<<c.geom[1]<<"],\"distance_m\":"<<c.dist<<'}';
  }
  std::cout<<"]}";
  mj_deleteData(d);mj_deleteModel(m);
}
'''


class _BuiltNativeTarget:
    """Keep real model validation, but use a per-test native library build."""
    def __init__(self, target, root):
        self.original, self.root = target, root

    def __getattr__(self, name):
        return getattr(self.original, name)


class RosGraspDistanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vendor = ROOT / 'runtime/root/opt/ros/humble/opt/mujoco_vendor'
        library = cls.vendor / 'lib/libmujoco.so.3.12.0'
        if not library.is_file():
            raise unittest.SkipTest('Native MuJoCo 3.12 runtime required')
        base = load_workcell('fr5_ur5e')
        if not base.output.exists() and not base.output.is_symlink():
            raise unittest.SkipTest('Generated FR5 + UR5e workcell required')
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.folder = Path(cls.temporary.name)
        cls.model = cls.folder / 'model'
        build_model(base, cls.model)
        target = load_ros_grasp_target(base, cls.model, file_sha256(cls.model / 'grasp.json'))
        cls.native_root = cls.folder / 'isolated-native'
        for relative in ('native/grasp_distance.cpp', 'native/grasp_policy.hpp'):
            destination = cls.native_root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / relative, destination)
        cls.shared = cls.native_root / 'runtime/native/grasp_distance.so'
        cls.shared.parent.mkdir(parents=True)
        common = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
                  '-I' + str(cls.vendor / 'include')]
        link = [str(library), '-Wl,-rpath,' + str(cls.vendor / 'lib')]
        subprocess.run([*common, '-shared', '-fPIC',
            '-DMANIPULATION_GRASP_DISTANCE_SHA256="' + file_sha256(ROOT / 'native/grasp_distance.cpp') + '"',
            '-DMANIPULATION_GRASP_POLICY_SHA256="' + file_sha256(ROOT / 'native/grasp_policy.hpp') + '"',
            str(ROOT / 'native/grasp_distance.cpp'), *link, '-o', str(cls.shared)],
            check=True, capture_output=True)
        cls.probe = cls.folder / 'native-inspect'
        source = cls.folder / 'inspect.cpp'
        source.write_text(INSPECT)
        subprocess.run([*common, str(source), *link, '-o', str(cls.probe)], check=True, capture_output=True)
        cls.target = _BuiltNativeTarget(target, cls.native_root)
        cls.initial = cls.engine(cls.model / 'cell.xml')
        cls.qpos = cls.initial['qpos']
        cls.catalog = {geom['name']: geom for geom in cls.initial['geoms'] if geom['name']}
        cls.block_address = next(entry['qpos_address'] for entry in target.manifest['qpos_state_contract']['layout']
                                 if entry['name'] == 'fr5_block_free')

    @classmethod
    def engine(cls, model, qpos=None):
        command = [str(cls.probe), str(model)]
        if qpos is not None:
            path = cls.folder / 'isolated-qpos.txt'
            path.write_text(' '.join(map(str, qpos)))
            command.append(str(path))
        return json.loads(subprocess.run(command, check=True, capture_output=True, timeout=15).stdout)

    def variant(self, name, edit):
        tree = ET.parse(self.model / 'cell.xml')
        edit(tree.getroot())
        path = self.folder / (name + '.xml')
        tree.write(path, encoding='unicode')
        return path

    def test_native_id_catalogue_retains_unnamed_geometries_and_rejects_closed_handle(self):
        with GraspDistanceEvaluator(self.target) as evaluator:
            names = evaluator.geom_names()
            self.assertEqual(names, tuple(row['name'] for row in self.initial['geoms']))
            self.assertEqual(len(names), 49)
            self.assertIn('', names)
        with self.assertRaises(ValueError):
            evaluator.geom_names()

    @contextmanager
    def raw(self, model):
        # Exercise the native parser's explicit unsupported-model boundary with
        # isolated XML variants; production still requires the checked manifest.
        with GraspDistanceEvaluator(self.target) as evaluator:
            error = C.create_string_buffer(2048)
            handle = evaluator.api.grasp_distance_open(str(model).encode(), error, len(error))
            try:
                yield evaluator.api, handle, error.value.decode()
            finally:
                if handle:
                    evaluator.api.grasp_distance_close(handle)

    def raw_distance(self, model, qpos, phase):
        with self.raw(model) as (api, handle, error):
            self.assertTrue(handle, error)
            output, pair, tcp = (C.c_double * 7)(), (C.c_int * 2)(), (C.c_double * 7)()
            status = api.grasp_distance_eval(handle, (C.c_double * 21)(*qpos), 21,
                                            phase, None, output, pair, tcp)
            self.assertEqual(status, 1)
            return output[0], list(pair)

    def test_full_state_uses_noncontiguous_arm_addresses_and_preserves_unnamed_geometry_ids(self):
        original = deepcopy(self.qpos)
        with GraspDistanceEvaluator(self.target) as evaluator:
            result = evaluator.at(self.qpos, phase=0)
            arms = {name: self.qpos[evaluator.layout[name]['qpos_address']] for name in self.target.joints}
            same = evaluator.at(self.qpos, phase=0, arm_state=arms)
            self.assertEqual(self.initial['native_version'], '3.12.0')
            self.assertEqual(len(self.qpos), 21)
            self.assertEqual(evaluator.layout['right_shoulder_pan_joint']['qpos_address'], 8)
            self.assertAlmostEqual(result['distance_m'], same['distance_m'], places=12)
            self.assertEqual(result['tcp_pose'], same['tcp_pose'])
            self.assertEqual(len(result['geom_ids']), 2)
            self.assertTrue(all(type(value) is int and value >= 0 for value in result['geom_ids']))
            self.assertTrue(any(name.startswith('geom:') for name in result['geoms']))
            self.assertGreater(result['distance_m'], .01)
            self.assertGreater(result['pair_count'], 100)
            self.assertFalse(evaluator.metadata()['physical_attachment'])
        self.assertEqual(self.qpos, original)

    def test_resting_block_support_is_allowed_but_transport_rejects_it_against_native_contacts(self):
        block, table = self.catalog['fr5_block_geom']['id'], self.catalog['fr5_table_geom']['id']
        contacts = [contact for contact in self.initial['contacts'] if set(contact['geoms']) == {block, table}]
        self.assertTrue(contacts)
        with GraspDistanceEvaluator(self.target) as evaluator:
            for phase in (0, 1, 2, 3, 5, 6, 7):
                with self.subTest(phase=phase):
                    self.assertGreater(evaluator.at(self.qpos, phase=phase)['distance_m'], .01)
            transport = evaluator.at(self.qpos, phase=4)
            self.assertEqual(set(transport['geom_ids']), {block, table})
            self.assertLess(transport['distance_m'], 0)
            self.assertAlmostEqual(transport['distance_m'], min(c['distance_m'] for c in contacts), delta=1e-6)

    def test_bilateral_finger_contacts_are_admitted_only_in_the_declared_grasp_phases(self):
        qpos = list(self.qpos)
        qpos[6] = qpos[7] = .025
        qpos[self.block_address + 2] += .08
        observation = self.engine(self.model / 'cell.xml', qpos)
        block = self.catalog['fr5_block_geom']['id']
        fingers = {self.catalog[name]['id'] for name in ('fr5_left_finger_geom', 'fr5_right_finger_geom')}
        actual = {other for contact in observation['contacts'] for other in contact['geoms'] if other != block}
        self.assertEqual(actual, fingers)
        self.assertLess(min(c['distance_m'] for c in observation['contacts']), -.005)
        with GraspDistanceEvaluator(self.target) as evaluator:
            for phase in (0, 1, 7):
                self.assertLess(evaluator.at(qpos, phase=phase)['distance_m'], -.005)
            for phase in (2, 3, 4, 5, 6):
                self.assertGreater(evaluator.at(qpos, phase=phase)['distance_m'], .01)

    def test_unexpected_block_contact_with_either_arm_is_never_an_allowed_grasp_pair(self):
        block = self.catalog['fr5_block_geom']['id']
        for body in ('left_forearm_link', 'right_forearm_link'):
            # Avoid a perfectly coincident support-vector initialization in the
            # native convex collider; use a stated tiny geometric fault offset.
            geom = next(g for g in self.initial['geoms'] if g['body'] == body)
            qpos = list(self.qpos)
            qpos[self.block_address:self.block_address + 3] = [round(v, 5) for v in geom['position']]
            actual = self.engine(self.model / 'cell.xml', qpos)['contacts']
            self.assertTrue(any(block in contact['geoms'] and contact['distance_m'] < -.001 for contact in actual))
            with self.subTest(body=body), GraspDistanceEvaluator(self.target) as evaluator:
                for phase in (0, 2, 4, 7):
                    result = evaluator.at(qpos, phase=phase)
                    self.assertIn(block, result['geom_ids'])
                    self.assertLess(result['distance_m'], -.001)

    def test_masks_and_explicit_body_exclusion_match_independent_native_contact_generation(self):
        forearm = next(g for g in self.initial['geoms'] if g['body'] == 'left_forearm_link')
        qpos = list(self.qpos)
        qpos[self.block_address:self.block_address + 3] = [round(v, 5) for v in forearm['position']]
        block = self.catalog['fr5_block_geom']['id']
        baseline = self.engine(self.model / 'cell.xml', qpos)
        self.assertTrue(any(block in c['geoms'] for c in baseline['contacts']))
        def mask(root):
            geom = root.find(".//geom[@name='fr5_block_geom']")
            geom.set('contype', '0'); geom.set('conaffinity', '0')
        def exclude(root):
            ET.SubElement(root.find('contact'), 'exclude', body1='fr5_grasp_block', body2='left_forearm_link')
        for name, edit in (('block-mask-off', mask), ('block-arm-excluded', exclude)):
            with self.subTest(case=name):
                model = self.variant(name, edit)
                self.assertFalse(any(block in c['geoms'] for c in self.engine(model, qpos)['contacts']))
                distance, pair = self.raw_distance(model, qpos, 2)
                self.assertNotIn(block, pair)
                self.assertGreater(distance, .01)

    def test_fixed_world_geometry_welded_to_world_does_not_add_false_fixed_fixed_collisions(self):
        def add_overlap(root):
            ET.SubElement(root.find('worldbody'), 'geom', name='fixed_pedestal_overlap', type='box',
                          pos='-0.65 0 0.2', size='0.05 0.05 0.05', contype='1', conaffinity='1')
        model = self.variant('fixed-world-overlap', add_overlap)
        contacts = self.engine(model)['contacts']
        self.assertEqual(len(contacts), len(self.initial['contacts']))
        distance, pair = self.raw_distance(model, self.qpos, 0)
        self.assertGreater(distance, .01)
        self.assertNotIn(len(self.initial['geoms']), pair)

    def test_held_block_prediction_follows_candidate_tcp_and_keeps_live_input_untouched(self):
        original = deepcopy(self.qpos)
        held = [0., 0., -.04, 1., 0., 0., 0.]
        with GraspDistanceEvaluator(self.target) as evaluator:
            clear = evaluator.at(self.qpos, phase=4, held_relative_pose=held)
            self.assertGreater(clear['distance_m'], .01)
            arms = {name: self.qpos[evaluator.layout[name]['qpos_address']] for name in self.target.joints}
            arms['left_j2'] += .1
            lowered = evaluator.at(self.qpos, phase=4, arm_state=arms, held_relative_pose=held)
            self.assertEqual(set(lowered['geoms']), {'fr5_table_geom', 'fr5_block_geom'})
            self.assertLess(lowered['distance_m'], -.02)
            self.assertLess(lowered['tcp_pose'][2], clear['tcp_pose'][2] - .06)
            self.assertEqual(lowered['prediction'], 'held_block_swept_geometry')
            for phase in (0, 1, 2, 6, 7):
                with self.subTest(phase=phase), self.assertRaises(ValueError):
                    evaluator.at(self.qpos, phase=phase, held_relative_pose=held)
        self.assertEqual(self.qpos, original)

    def test_malformed_units_layout_limits_and_unusable_phases_fail_before_returning_clearance(self):
        with GraspDistanceEvaluator(self.target) as evaluator:
            for qpos in (self.qpos[:12], self.qpos[:-1], [True] + self.qpos[1:],
                         [float('nan')] + self.qpos[1:]):
                with self.subTest(qpos=qpos[:2]), self.assertRaises(ValueError):
                    evaluator.at(qpos, phase=0)
            invalid = list(self.qpos); invalid[self.block_address + 3] = 2.
            with self.assertRaises(ValueError): evaluator.at(invalid, phase=0)
            invalid = list(self.qpos); invalid[6] = .035
            with self.assertRaises(ValueError): evaluator.at(invalid, phase=2)
            for phase in (-1, 8, True, 1.):
                with self.subTest(phase=phase), self.assertRaises(ValueError):
                    evaluator.at(self.qpos, phase=phase)
            with self.assertRaises(ValueError):
                evaluator.at(self.qpos, phase=0, arm_state={'fr5_left_finger_slide': .02})
            evaluator.close()
            with self.assertRaises(ValueError): evaluator.at(self.qpos, phase=0)

    def test_explicit_pairs_and_mocap_are_rejected_instead_of_silently_using_other_filter_rules(self):
        def pair(root):
            ET.SubElement(root.find('contact'), 'pair', geom1='fr5_table_geom', geom2='fr5_block_geom')
        def mocap(root):
            body = ET.SubElement(root.find('worldbody'), 'body', name='unsupported_mocap', mocap='true', pos='3 3 3')
            ET.SubElement(body, 'geom', name='unsupported_mocap_geom', type='sphere', size='.01')
        for name, edit in (('explicit-pair', pair), ('mocap', mocap)):
            with self.subTest(case=name), self.raw(self.variant(name, edit)) as (_, handle, error):
                self.assertFalse(handle)
                self.assertTrue(error)


if __name__ == '__main__':
    unittest.main()
