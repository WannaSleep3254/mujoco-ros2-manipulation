"""Frozen target checks over actual composed XML and CPU geometry fixtures."""
from copy import deepcopy
from dataclasses import FrozenInstanceError
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.workcells import execution_target as targets
from mujoco_ros2_manipulation.workcells.environment_assets import canonical, sha
from mujoco_ros2_manipulation.workcells.environment_overlay import check_overlay, check_overlay_identity
from tests.unit import test_environment_overlay as overlay_tests


class ExecutionTargetPairTests(unittest.TestCase):
    def test_base_path_has_no_new_validation_or_worker_and_pair_is_mandatory(self):
        with patch.object(targets, 'check_overlay_identity') as identity, patch.object(targets, '_preflight') as cpu:
            self.assertIsNone(targets.load_execution_target(object()))
            for directory, digest in ((Path('/tmp'), None), (None, 'a'*64),
                                      (Path('/tmp'), 'A'*64), (Path('/tmp'), 'bad')):
                with self.assertRaises(ValueError):
                    targets.load_execution_target(object(), directory, digest)
            identity.assert_not_called()
            cpu.assert_not_called()


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'CPU MuJoCo fixture required')
class ExecutionTargetTests(unittest.TestCase):
    def setUp(self):
        self.fixture = overlay_tests.EnvironmentOverlayTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.base = self.fixture.cell
        self.base.instances = ()
        self.directory, self.report = self.fixture.build()
        self.digest = sha((self.directory/'overlay.json').read_bytes())
        real_identity = lambda directory, base, **kwargs: check_overlay_identity(
            directory, base, base_validator=self.fixture.validate_base, **kwargs)
        self.identity = patch.object(targets, 'check_overlay_identity', side_effect=real_identity).start()
        self.cpu = patch.object(targets, '_preflight', side_effect=lambda directory, base, digest, report:
            check_overlay(directory, base, expected_sha256=digest, base_validator=self.fixture.validate_base)).start()
        self.addCleanup(patch.stopall)

    def load(self, digest=None):
        return targets.load_execution_target(self.base, self.directory, self.digest if digest is None else digest)

    def test_target_is_separate_frozen_detached_and_preflight_runs_only_at_load(self):
        target = self.load()
        self.assertIs(target.base, self.base)
        self.assertEqual(target.output, self.directory)
        self.assertEqual(target.id, self.report['id'])
        self.assertEqual(target.model_path, self.directory/'cell.xml')
        self.assertEqual(target.joints, self.base.joints)
        self.assertEqual(target.instances, self.base.instances)
        self.assertEqual(target.home, self.base.home)
        self.assertEqual(target.root, self.base.root)
        self.assertEqual(target.fingerprint, self.report['fingerprint'])
        self.assertNotEqual(target.id, self.base.id)
        self.assertFalse((target.output/'source.json').exists())
        with self.assertRaises(FrozenInstanceError):
            target.output = self.base.output
        with patch('mujoco_ros2_manipulation.workcells.environment_overlay.geometry_check',
                   side_effect=AssertionError('Repeated target validation must be engine-free')):
            description = target.validate()
            self.assertEqual(target.validate(), description)
        self.cpu.assert_called_once()
        self.assertEqual(description['manifest_sha256'], self.digest)
        self.assertEqual(description['base_workcell_id'], self.base.id)
        self.assertEqual(description['outputs_sha256']['cell.xml'], self.report['outputs']['cell.xml'])
        self.assertNotIn(str(self.fixture.root), json.dumps(description))
        description['outputs_sha256']['cell.xml'] = 'mutated'
        self.assertNotEqual(target.description()['outputs_sha256']['cell.xml'], 'mutated')
        self.assertEqual({p.name:p.read_bytes() for p in self.base.output.iterdir()}, self.fixture.before)

    def test_bad_hash_fails_before_cpu_or_any_target_can_be_returned(self):
        with self.assertRaisesRegex(ValueError, 'manifest SHA mismatch'):
            self.load('0'*64)
        self.cpu.assert_not_called()

    def test_changed_output_or_rehashed_manifest_cannot_reuse_target(self):
        target = self.load()
        path = self.directory/'cell.xml'
        path.write_bytes(path.read_bytes()+b'\n<!-- changed after startup -->\n')
        with self.assertRaisesRegex(ValueError, 'outputs|inventory'):
            target.validate()
        changed = deepcopy(self.report)
        changed['outputs']['cell.xml'] = sha(path.read_bytes())
        (self.directory/'overlay.json').write_bytes(canonical(changed)+b'\n')
        with self.assertRaisesRegex(ValueError, 'manifest SHA mismatch'):
            target.validate()
        self.cpu.assert_called_once()

    def test_change_during_owned_cpu_preflight_returns_no_target(self):
        def mutate(directory, *_):
            path = directory/'cell.srdf'
            path.write_bytes(path.read_bytes()+b'\n')
        self.cpu.side_effect = mutate
        with self.assertRaisesRegex(ValueError, 'outputs|inventory'):
            self.load()

    def test_direct_unpreflighted_construction_does_not_authorize_execution(self):
        target = targets.ExecutionTarget(self.base, self.directory, self.digest, canonical(self.report))
        with self.assertRaisesRegex(ValueError, 'startup CPU preflight'):
            target.validate()

    def test_symlink_selection_is_not_an_execution_target(self):
        linked = self.fixture.root/'alias'
        linked.symlink_to(self.directory, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'unlinked directory'):
            targets.load_execution_target(self.base, linked, self.digest)
        self.cpu.assert_not_called()
