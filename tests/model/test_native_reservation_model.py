"""Real generated-workcell file/asset binding for relative dispatch preparation."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from tests.unit.test_self_legacy_dispatch import load_dispatch


class NativeReservationModelTests(unittest.TestCase):
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
