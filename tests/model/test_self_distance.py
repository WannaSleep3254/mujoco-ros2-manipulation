"""Native self scope, independent analytic contacts and preserved CI states."""
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
import ctypes as C
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from threading import Event, Thread
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator


class NativeSelfDistanceTests(unittest.TestCase):
    @contextmanager
    def cell(self, *, edit=None, contact='', option='', assets='', outside='', static=False):
        if not (ROOT/'runtime/native/path_distance.so').is_file():
            self.skipTest('Native distance evaluator not built')
        with tempfile.TemporaryDirectory() as folder:
            directory=Path(folder)
            robot=[]
            for prefix, x in (('left_',0),('right_',3)):
                root_joint='' if static else f'<joint name="{prefix}root" type="slide" axis="1 0 0"/>'
                robot.append(f'''<body name="{prefix}base" pos="{x} 0 0">{root_joint}
                  <geom name="{prefix}base_geom" type="sphere" size=".1"/>
                  <geom type="sphere" size="5" mass="0" contype="0" conaffinity="0"/>
                  <body name="{prefix}middle" pos="0 0 .5">
                    <joint name="{prefix}middle" type="hinge"/>
                    <geom name="{prefix}middle_geom" type="sphere" size=".01"/>
                    <body name="{prefix}tip" pos="0 0 .5">
                      <joint name="{prefix}tip" type="slide" axis="0 0 1"/>
                      <geom name="{prefix}tip_geom" type="sphere" size=".1"/>
                    </body>
                  </body></body>''')
            tree=ET.fromstring('<mujoco>'+option+assets+'<worldbody>'+''.join(robot)+outside+'</worldbody>'+contact+'</mujoco>')
            if edit:edit(tree)
            ET.ElementTree(tree).write(directory/'cell.xml')
            instances=[NS(prefix=p,profile=NS(ros=dict(base_frame='base',tip_frame='tip')))
                       for p in ('left_','right_')]
            joints=[j.get('name') for j in tree.findall('.//joint')]
            yield NS(root=ROOT,output=directory,joints=joints,instances=instances)

    def zero(self, cell):
        return dict.fromkeys(cell.joints,0.)

    def test_self_distance_analytic_penetration_and_cross_output_is_exactly_preserved(self):
        wall='<geom name="wall" type="sphere" size=".1" pos="6 0 0"/>'
        with self.cell(outside=wall) as cell:
            with DistanceEvaluator(cell) as plain, DistanceEvaluator(cell,environment=True,self_collision=True) as combined:
                state=self.zero(cell)
                result=combined.at(state);extra=result.pop('self_collision');result.pop('environment')
                self.assertEqual(result,plain.at(state))
                self.assertNotIn('self_collision',plain.metadata())
                self.assertEqual(set(extra),{'distance_m','geom_ids','bodies','nearest_points_m','pair_count','scope'})
                self.assertAlmostEqual(extra['distance_m'],.8)
                self.assertEqual(extra['scope'],'within_each_arm')
                self.assertEqual(extra['pair_count'],2)
                self.assertEqual(extra['bodies'],['left_base','left_tip'])
                self.assertAlmostEqual(extra['nearest_points_m'][0][2],.1)
                self.assertAlmostEqual(extra['nearest_points_m'][1][2],.9)
                state['left_tip']=-.9
                self.assertAlmostEqual(combined.at(state)['self_collision']['distance_m'],-.1)
                self.assertGreater(plain.at(state)['distance_m'],2.)
                metadata=combined.metadata()['self_collision']
                self.assertEqual(metadata['geom_count_by_arm'],[3,3])
                self.assertEqual(metadata['pair_count_by_arm'],[1,1])
                self.assertEqual(metadata['source_sha256'],hashlib.sha256((ROOT/'native/path_distance_self.cpp').read_bytes()).hexdigest())
                self.assertEqual(metadata['library_sha256'],combined.library_sha256)

    def test_unchanged_legacy_c_abi_library_has_bit_exact_cross_results(self):
        # Build only the immutable historical source. This also proves the
        # Python default still works with a library that has no new symbols.
        source=ROOT/'native/path_distance.cpp'
        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(),
                         '31f95dbec670468baa054a0b41efc2e54a3c4e7b09893a1d15e1c9ca3a1bb30b')
        with self.cell() as cell,tempfile.TemporaryDirectory() as folder:
            old=Path(folder);(old/'native').mkdir();(old/'runtime/native').mkdir(parents=True)
            shutil.copyfile(source,old/'native/path_distance.cpp')
            vendor=ROOT/'runtime/root/opt/ros/humble/opt/mujoco_vendor'
            subprocess.run(['g++','-std=c++17','-shared','-fPIC','-O2',
                '-DMANIPULATION_DISTANCE_SHA256="'+hashlib.sha256(source.read_bytes()).hexdigest()+'"',
                '-I'+str(vendor/'include'),str(source),str(vendor/'lib/libmujoco.so.3.12.0'),
                '-Wl,-rpath,'+str(vendor/'lib'),'-o',str(old/'runtime/native/path_distance.so')],
                capture_output=True,check=True,timeout=30)
            legacy_cell=NS(root=old,output=cell.output,joints=cell.joints,instances=cell.instances)
            with DistanceEvaluator(legacy_cell) as legacy,DistanceEvaluator(cell,self_collision=True) as current:
                for i in range(12):
                    state=self.zero(cell);state['left_tip']=-i/20;state['right_middle']=i/30
                    result=current.at(state);result.pop('self_collision')
                    self.assertEqual(result,legacy.at(state))

    def test_same_body_weld_masks_exclusions_and_parent_filter_match_native_scope(self):
        def duplicates(tree):
            for prefix in ('left_','right_'):
                base=tree.find(f"worldbody/body[@name='{prefix}base']")
                ET.SubElement(base,'geom',dict(type='sphere',size='.1'))
                welded=ET.SubElement(base,'body',dict(name=prefix+'welded'))
                ET.SubElement(welded,'geom',dict(type='sphere',size='.1'))
        with self.cell(edit=duplicates) as cell:
            with DistanceEvaluator(cell,self_collision=True) as evaluator:
                self.assertAlmostEqual(evaluator.at(self.zero(cell))['self_collision']['distance_m'],.8)
                self.assertEqual(evaluator.metadata()['self_collision']['pair_count_by_arm'],[3,3])
        with self.cell(option='<option><flag filterparent="disable"/></option>') as cell:
            with DistanceEvaluator(cell,self_collision=True) as evaluator:
                self.assertAlmostEqual(evaluator.at(self.zero(cell))['self_collision']['distance_m'],.39)
                self.assertEqual(evaluator.metadata()['self_collision']['pair_count_by_arm'],[3,3])
        # The native world-weld exception does not remove the moving shoulder.
        with self.cell(static=True) as cell:
            with DistanceEvaluator(cell,self_collision=True) as evaluator:
                self.assertAlmostEqual(evaluator.at(self.zero(cell))['self_collision']['distance_m'],.39)
                self.assertEqual(evaluator.metadata()['self_collision']['pair_count_by_arm'],[2,2])
        for kind in ('masked','excluded'):
            def remove_candidate(tree):
                for prefix in ('left_','right_'):
                    if kind=='masked':
                        tip=tree.find(f".//geom[@name='{prefix}tip_geom']")
                        tip.set('contype','2');tip.set('conaffinity','2')
                    else:
                        contact=tree.find('contact')
                        if contact is None:contact=ET.SubElement(tree,'contact')
                        ET.SubElement(contact,'exclude',dict(body1=prefix+'base',body2=prefix+'tip'))
            with self.subTest(kind=kind),self.cell(edit=remove_candidate) as cell:
                with self.assertRaisesRegex(ValueError,'No eligible self-collision pairs'):
                    DistanceEvaluator(cell,self_collision=True)

    def test_explicit_pairs_mocap_and_unsupported_geometry_are_not_silently_omitted(self):
        pair='<contact><pair geom1="left_base_geom" geom2="left_tip_geom"/></contact>'
        mocap='<body mocap="true"><geom type="sphere" size=".1"/></body>'
        for contact,outside in ((pair,''),('',mocap)):
            with self.cell(contact=contact,outside=outside) as cell:
                with self.assertRaisesRegex(ValueError,'does not support'):
                    DistanceEvaluator(cell,self_collision=True)
        def hfield(tree):
            base=tree.find("worldbody/body[@name='left_base']")
            ET.SubElement(base,'geom',dict(type='hfield',hfield='terrain'))
        with self.cell(static=True,edit=hfield,
                assets='<asset><hfield name="terrain" nrow="2" ncol="2" size="1 1 .1 .1"/></asset>') as cell:
            with self.assertRaisesRegex(ValueError,'Unsupported self-collision'):
                DistanceEvaluator(cell,self_collision=True)

    def test_capacity_is_rejected_before_coverage_is_accepted(self):
        def many_geoms(tree):
            for body_name in ('left_base','left_tip'):
                body=tree.find(f".//body[@name='{body_name}']")
                for i in range(64):ET.SubElement(body,'geom',dict(type='sphere',size='.01',pos=f'0 {i/100} 0'))
        with self.cell(edit=many_geoms) as cell:
            with self.assertRaisesRegex(ValueError,'exceeds 4096'):
                DistanceEvaluator(cell,self_collision=True)

    def test_invalid_fk_python_state_and_close_invalidate_both_extension_snapshots(self):
        with self.cell(outside='<geom type="sphere" size=".1" pos="6 0 0"/>') as cell:
            evaluator=DistanceEvaluator(cell,self_collision=True,environment=True)
            output,pair=(C.c_double*7)(),(C.c_int*2)();handle=evaluator.handle
            def self_ready():return evaluator.api.path_distance_self_eval(handle,output,pair)
            def env_ready():return evaluator.api.path_distance_environment_eval(handle,output,pair)
            self.assertEqual((self_ready(),env_ready()),(0,0))
            for invalid in (None,{},dict(self.zero(cell),left_tip=float('nan')),
                            dict(self.zero(cell),left_tip=10**10000)):
                evaluator.at(self.zero(cell));self.assertEqual((self_ready(),env_ready()),(1,1))
                with self.assertRaises(ValueError):evaluator.at(invalid)
                self.assertEqual((self_ready(),env_ready()),(0,0))
            evaluator.at(self.zero(cell))
            q=(C.c_double*len(cell.joints))(*([0.]*len(cell.joints)));cross=(C.c_double*13)()
            self.assertEqual(evaluator.api.path_distance_eval(handle,q,len(cell.joints)-1,cross,pair),0)
            self.assertEqual((self_ready(),env_ready()),(0,0))
            evaluator.at(self.zero(cell))
            self.assertEqual(evaluator.api.path_distance_eval(handle,None,len(cell.joints),cross,pair),0)
            self.assertEqual((self_ready(),env_ready()),(0,0))
            evaluator.at(self.zero(cell));evaluator.close();evaluator.close()
            self.assertEqual((self_ready(),env_ready()),(0,0))
            with self.assertRaises(ValueError):evaluator.at(self.zero(cell))
        with self.assertRaisesRegex(ValueError,'boolean'):DistanceEvaluator(None,self_collision=1)

    def test_changed_root_cannot_reuse_original_pair_coverage(self):
        with self.cell() as cell:
            with DistanceEvaluator(cell,self_collision=True) as evaluator:
                evaluator.at(self.zero(cell))
                roots=(C.c_char_p*2)(b'left_middle',b'right_base')
                counts,error=(C.c_int*4)(),C.create_string_buffer(2048)
                self.assertEqual(evaluator.api.path_distance_self_enable(evaluator.handle,roots,counts,error,len(error)),0)
                self.assertIn(b'differs from the base',error.value)
                output,pair=(C.c_double*7)(),(C.c_int*2)()
                self.assertEqual(evaluator.api.path_distance_self_eval(evaluator.handle,output,pair),0)
                with self.assertRaisesRegex(ValueError,'self-collision distance evaluation failed'):
                    evaluator.at(self.zero(cell))

    def test_source_identity_is_opt_in_and_aggregator_hooks_are_also_pinned(self):
        with self.cell() as cell:
            alias=cell.output/'alias';(alias/'native').mkdir(parents=True);(alias/'runtime/native').mkdir(parents=True)
            (alias/'runtime/native/path_distance.so').symlink_to(ROOT/'runtime/native/path_distance.so')
            for name in ('path_distance.cpp','path_distance_environment.cpp','path_distance_self.cpp'):
                shutil.copyfile(ROOT/'native'/name,alias/'native'/name)
            changed=NS(root=alias,output=cell.output,joints=cell.joints,instances=cell.instances)
            for name,message in (('path_distance_self.cpp','Stale self-collision distance'),
                                 ('path_distance_environment.cpp','Stale self-collision snapshot')):
                original=(alias/'native'/name).read_bytes();(alias/'native'/name).write_bytes(original+b'\n// changed')
                with DistanceEvaluator(changed) as legacy:self.assertGreater(legacy.at(self.zero(cell))['distance_m'],0)
                with self.assertRaisesRegex(ValueError,message):DistanceEvaluator(changed,self_collision=True)
                (alias/'native'/name).write_bytes(original)

    def test_two_handles_parallel_registry_lifecycle_and_single_handle_close_lock(self):
        with self.cell() as cell:
            def worker(offset):
                for _ in range(20):
                    with DistanceEvaluator(cell,self_collision=True) as evaluator:
                        state=self.zero(cell);state['left_tip']=-offset
                        self.assertAlmostEqual(evaluator.at(state)['self_collision']['distance_m'],.8-offset)
            with ThreadPoolExecutor(max_workers=4) as pool:
                list(pool.map(worker,[0.,.1,.2,.3]))
            evaluator=DistanceEvaluator(cell,self_collision=True)
            entered,release,close_attempt,closed=Event(),Event(),Event(),Event()
            real_eval=evaluator.api.path_distance_eval;real_close=evaluator.api.path_distance_close
            def pause(*args):
                entered.set();self.assertTrue(release.wait(2))
                if closed.is_set():raise AssertionError('native close overlapped in-flight FK')
                return real_eval(*args)
            def record_close(*args):closed.set();return real_close(*args)
            errors=[]
            def evaluate():
                try:evaluator.at(self.zero(cell))
                except BaseException as error:errors.append(error)
            def close():close_attempt.set();evaluator.close()
            with patch.object(evaluator.api,'path_distance_eval',pause),patch.object(evaluator.api,'path_distance_close',record_close):
                first=Thread(target=evaluate);second=Thread(target=close);first.start()
                self.assertTrue(entered.wait(1));second.start();self.assertTrue(close_attempt.wait(1))
                self.assertFalse(closed.wait(.05));release.set();first.join(2);second.join(2)
                self.assertFalse(first.is_alive());self.assertFalse(second.is_alive())
            self.assertEqual(errors,[]);self.assertTrue(closed.is_set())

    def test_saved_ci_nominal_self_penetration_is_separate_from_cross_arm_clearance(self):
        from mujoco_ros2_manipulation.workcells.config import load_workcell
        from mujoco_ros2_manipulation.workcells.integrity import check_workcell
        cell=load_workcell('fr5_ur5e')
        if not (ROOT/'runtime/native/path_distance.so').is_file():self.skipTest('Native evaluator required')
        if not cell.output.exists() and not cell.output.is_symlink():
            self.skipTest('Generated shared workcell required')
        # A present but partial/changed shared model is an integrity failure,
        # rather than the intentionally absent single-profile prerequisite.
        check_workcell(cell)
        fixture=json.loads((ROOT/'tests/fixtures/native-self-clearance-ci137.json').read_text())
        self.assertEqual(list(cell.joints),fixture['joint_names'])
        with DistanceEvaluator(cell) as cross,DistanceEvaluator(cell,self_collision=True) as checked:
            worst=dict(zip(cell.joints,fixture['states']['failed_minimum']['positions_rad']))
            legacy=cross.at(worst);actual=checked.at(worst);own=actual.pop('self_collision')
            self.assertEqual(actual,legacy);self.assertGreater(legacy['distance_m'],.03)
            self.assertLess(own['distance_m'],-.002)
            self.assertEqual(set(own['bodies']),{'left_forearm_link','left_wrist3_link'})
            for name in ('home','case_start','passed_curve_start','passed_minimum'):
                state=dict(zip(cell.joints,fixture['states'][name]['positions_rad']))
                self.assertGreater(checked.at(state)['self_collision']['distance_m'],.01)


if __name__=='__main__':unittest.main()
