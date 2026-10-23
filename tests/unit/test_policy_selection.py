"""Timing policy selection binds a named file to its owning workcell."""
from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import yaml

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.timing import load_policy


class PolicySelectionTests(unittest.TestCase):
    def test_default_and_explicit_base_selection_are_identical(self):
        cell=SimpleNamespace(root=ROOT,id='fr5_ur5e')
        self.assertEqual(load_policy(cell),load_policy(cell,'dual_arm_timing'))
        path,policy=load_policy(cell)
        self.assertEqual(path,ROOT/'config/planning/dual_arm_timing.yaml')
        self.assertEqual(policy['min_inter_arm_distance_m']+policy['tracking_reserve_m'],.03)

    def test_post_policy_is_owned_by_the_post_cell(self):
        cell=SimpleNamespace(root=ROOT,id='fr5_ur5e_eval_post')
        path,policy=load_policy(cell,'eval_post_timing')
        self.assertEqual(path.stem,'eval_post_timing')
        self.assertEqual(policy['workcell'],cell.id)
        self.assertEqual(policy['case'],'eval_post')
        with self.assertRaisesRegex(ValueError,'Invalid timing policy'):
            load_policy(cell)
        with self.assertRaisesRegex(ValueError,'Invalid timing policy'):
            load_policy(SimpleNamespace(root=ROOT,id='fr5_ur5e'),'eval_post_timing')

    def test_names_cannot_escape_the_policy_directory(self):
        cell=SimpleNamespace(root=ROOT,id='fr5_ur5e')
        for value in ('../dual_arm_timing','/tmp/policy','x/y','x.yaml','',True,None,123,'UPPER'):
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'policy name'):
                load_policy(cell,value)

    def test_selected_policy_still_enforces_full_schema_and_search_bounds(self):
        original=load_policy(SimpleNamespace(root=ROOT,id='fr5_ur5e'))[1]
        changes=[dict(workcell='other'),dict(sample_dt_sec=.001),dict(tracking_reserve_m=-.01),dict(max_delay_sec=float('nan')),dict(extra=1)]
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);directory=root/'config/planning';directory.mkdir(parents=True)
            for change in changes:
                value=deepcopy(original);value.update(change)
                (directory/'chosen.yaml').write_text(yaml.safe_dump(value))
                with self.subTest(change=change),self.assertRaises(ValueError):
                    load_policy(SimpleNamespace(root=root,id='fr5_ur5e'),'chosen')
