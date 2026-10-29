"""Real generated-workcell file/asset binding for relative dispatch preparation."""
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator
from mujoco_ros2_manipulation.workcells.self_clearance import self_evaluator_identity
from tests.unit.test_self_legacy_dispatch import load_dispatch
from tests.unit import test_self_legacy_dispatch as legacy_fixture


class NativeReservationModelTests(unittest.TestCase):
    def test_957_knot_twelve_joint_native_model_keeps_exact_curves_and_binds_full_source_hash_once(self):
        cell=load_workcell('fr5_ur5e')
        if not cell.output.exists() and not cell.output.is_symlink():self.skipTest('Generated FR5 + UR5e workcell required')
        fixture=legacy_fixture.LegacySelfDispatchTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        node=fixture.node;node.cell=cell;node.commanded=dict(zip(cell.joints,cell.home));node.clock_last=15.25
        del node._self_model_binding
        fixture.namespace.update(check_workcell=check_workcell,DistanceEvaluator=DistanceEvaluator,
            self_evaluator_identity=self_evaluator_identity)
        commands=[]
        for inst in cell.instances:
            points=[]
            for i in range(957):
                ns=i*20_000_000
                points.append(NS(positions=[node.commanded[n] for n in inst.joints],velocities=[0.]*6,
                    accelerations=[0.]*6,effort=[],time_from_start=NS(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)))
            commands.append((inst,NS(joint_names=list(inst.joints),points=points,header=NS(stamp=NS(sec=0,nanosec=0),frame_id='world'))))
        original=node._self_command_key(commands,headers=False)
        with patch.dict(os.environ,{'GIT_OPTIONAL_LOCKS':'0'}):
            start,record=fixture.fresh(commands)
        self.assertEqual(start,16.29);self.assertEqual(record['reservation_mint_count'],1)
        self.assertEqual(node._self_command_key(node.sent[0][0],headers=False),original)
        self.assertTrue(record['native_self_clearance']['complete'])
        self.assertGreaterEqual(record['native_self_clearance']['samples'],957)
        self.assertEqual(sum(x['stage']=='full_native_policy_hash_binding' for x in record['bounded_stage_costs']),1)
        self.assertTrue(record['latest_sync']['clock_time_advanced']);self.assertTrue(record['latest_sync']['state_time_advanced'])

    def test_real_generated_model_metadata_and_declared_assets_are_bound_before_reservation(self):
        cell=load_workcell('fr5_ur5e')
        if not cell.output.exists() and not cell.output.is_symlink():
            self.skipTest('Generated FR5 + UR5e workcell required')
        cls,namespace=load_dispatch();namespace['check_workcell']=check_workcell
        node=cls();node.cell=cell
        with patch.dict(os.environ,{'GIT_OPTIONAL_LOCKS':'0'}):
            binding=node._self_model_binding()
        self.assertEqual(binding['xml_sha256'],file_sha256(cell.output/'cell.xml'))
        self.assertEqual(binding['manifest_sha256'],file_sha256(cell.output/'source.json'))
        self.assertIn(str(cell.output/'cell.xml'),binding['stats'])
        self.assertIn(str(cell.output/'source.json'),binding['stats'])
        self.assertTrue(any('visual_meshes' in p for p in binding['stats']))
        self.assertEqual(node._self_model_binding(binding),binding)
        # Exercise replacement detection on isolated copies, never live assets.
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'asset';path.write_bytes(b'original')
            copied=dict(binding,stats=node._self_file_stats([str(path)]))
            replacement=Path(folder)/'replacement';replacement.write_bytes(b'changed')
            replacement.replace(path)
            with self.assertRaisesRegex(RuntimeError,'model files changed'):
                node._self_model_binding(copied)
