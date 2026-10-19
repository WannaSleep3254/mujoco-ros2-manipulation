"""Reject stale inputs, altered outputs and unsuccessful model publication."""

from copy import deepcopy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.model_build.integrity import GENERATOR_FILES, ensure_inputs_unchanged, input_snapshot, make_integrity
from mujoco_ros2_manipulation.profiles import RobotProfile, check_generated_model, load_profile


class IntegrityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        for name in GENERATOR_FILES:
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('generator input: ' + name)
        self.vendor = self.root / 'external/vendor'
        self.vendor.mkdir(parents=True)
        for name in ('robot.urdf', 'robot.srdf', 'mesh.stl'):
            (self.vendor / name).write_text('source: ' + name)
        for command in (['init', '--quiet'], ['add', '.'],
                        ['-c', 'user.name=Test', '-c', 'user.email=test@example.invalid',
                         'commit', '--quiet', '-m', 'fixture']):
            subprocess.run(['git', '-C', str(self.vendor), *command], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        data = deepcopy(load_profile('fr5').data)
        data['source'].update(repository_path='external/vendor', urdf='external/vendor/robot.urdf',
                              srdf='external/vendor/robot.srdf', commit=subprocess.check_output(
                                  ['git', '-C', str(self.vendor), 'rev-parse', 'HEAD'], text=True).strip())
        self.profile = RobotProfile(self.root, data)
        self.write_build(self.profile)

    def write_build(self, profile, marker='original'):
        profile.output.mkdir(parents=True, exist_ok=True)
        profile.scene_path.write_text(f'<mujoco model="{marker}"/>')
        profile.urdf_path.write_text(f'<robot name="{marker}"/>')
        assets = profile.output / 'visual_meshes'
        assets.mkdir(exist_ok=True)
        (assets / 'part.obj').write_text('v 0 0 0\n')
        report = {'robot_profile_sha256': profile.fingerprint,
                  'integrity': make_integrity(profile, [self.vendor / 'mesh.stl'], profile.output)}
        (profile.output / 'source.json').write_text(json.dumps(report))
        return report

    def test_complete_manifest_passes(self):
        check_generated_model(self.profile)

    def test_inputs_changed_after_read_are_rejected_before_publication(self):
        snapshot = input_snapshot(self.profile)
        ensure_inputs_unchanged(self.profile, snapshot)
        (self.root / GENERATOR_FILES[0]).write_text('changed after input read')
        with self.assertRaisesRegex(ValueError, 'inputs changed during generation'):
            ensure_inputs_unchanged(self.profile, snapshot)

    def test_output_changes_and_missing_assets_are_rejected(self):
        for path in (self.profile.scene_path, self.profile.urdf_path,
                     self.profile.output / 'visual_meshes/part.obj'):
            with self.subTest(path=path.name):
                original = path.read_bytes()
                path.write_bytes(original + b'changed')
                with self.assertRaisesRegex(ValueError, 'generated output changed'):
                    check_generated_model(self.profile)
                path.write_bytes(original)
        (self.profile.output / 'visual_meshes/part.obj').unlink()
        with self.assertRaises(ValueError):
            check_generated_model(self.profile)

    def test_generator_and_configuration_changes_are_rejected(self):
        for name in (GENERATOR_FILES[0], GENERATOR_FILES[3], 'requirements.txt',
                     'config/controller_defaults.yaml'):
            with self.subTest(input=name):
                path = self.root / name
                original = path.read_bytes()
                path.write_bytes(original + b'changed')
                with self.assertRaisesRegex(ValueError, 'inputs changed'):
                    check_generated_model(self.profile)
                path.write_bytes(original)

    def test_dirty_manufacturer_sources_are_rejected(self):
        for name in ('robot.urdf', 'robot.srdf', 'mesh.stl'):
            with self.subTest(source=name):
                path = self.vendor / name
                original = path.read_bytes()
                path.write_bytes(original + b'changed')
                with self.assertRaisesRegex(ValueError, 'tracked changes'):
                    check_generated_model(self.profile)
                path.write_bytes(original)

    def test_unexpected_generated_asset_is_rejected(self):
        (self.profile.output / 'visual_meshes/extra.obj').write_text('v 1 2 3')
        with self.assertRaisesRegex(ValueError, 'inventory changed'):
            check_generated_model(self.profile)

    def test_legacy_malformed_and_escaping_manifests_are_rejected(self):
        path = self.profile.output / 'source.json'
        original = json.loads(path.read_text())
        bad_input = deepcopy(original)
        bad_input['integrity']['inputs']['files']['../outside'] = '0' * 64
        bad_output = deepcopy(original)
        bad_output['integrity']['outputs']['../outside'] = '0' * 64
        missing = deepcopy(original)
        missing['integrity']['outputs'].pop('fr5.xml')
        for report in ({'robot_profile_sha256': self.profile.fingerprint}, [],
                       bad_input, bad_output, missing):
            with self.subTest(report=report):
                path.write_text(json.dumps(report))
                with self.assertRaises(ValueError):
                    check_generated_model(self.profile)
        path.write_text('{invalid')
        with self.assertRaises(ValueError):
            check_generated_model(self.profile)

    def test_failed_staged_generation_preserves_previous_outputs(self):
        from mujoco_ros2_manipulation.model_build.prepare import prepare
        before = {p.relative_to(self.profile.output): p.read_bytes()
                  for p in self.profile.output.rglob('*') if p.is_file()}

        def fail(staged, destination):
            staged.scene_path.write_text('partially generated')
            raise ValueError('injected compilation failure')

        with patch('mujoco_ros2_manipulation.model_build.prepare._prepare', side_effect=fail):
            with self.assertRaisesRegex(ValueError, 'compilation failure'):
                prepare(self.profile)
        after = {p.relative_to(self.profile.output): p.read_bytes()
                 for p in self.profile.output.rglob('*') if p.is_file()}
        self.assertEqual(before, after)
        check_generated_model(self.profile)

    def test_interrupted_publication_cannot_pass_integrity_check(self):
        from mujoco_ros2_manipulation.model_build.prepare import prepare
        import os
        replace = os.replace

        def publish(source, target):
            if str(target).endswith('.urdf'):
                raise OSError('injected publication failure')
            replace(source, target)

        def build(staged, destination):
            return self.write_build(staged, 'new-build')

        with patch('mujoco_ros2_manipulation.model_build.prepare._prepare', side_effect=build), patch('mujoco_ros2_manipulation.model_build.prepare.os.replace', side_effect=publish):
            with self.assertRaisesRegex(OSError, 'publication failure'):
                prepare(self.profile)
        with self.assertRaisesRegex(ValueError, 'generated output changed'):
            check_generated_model(self.profile)

    def test_regeneration_recovers_from_damaged_metadata(self):
        from mujoco_ros2_manipulation.model_build.prepare import prepare
        path = self.profile.output / 'source.json'
        for contents in ('{invalid', '[]', '{"integrity": null}',
                         '{"integrity": {"outputs": []}}'):
            with self.subTest(metadata=contents):
                path.write_text(contents)
                with patch('mujoco_ros2_manipulation.model_build.prepare._prepare', side_effect=lambda staged, destination: self.write_build(staged)), \
                     patch('mujoco_ros2_manipulation.model_build.prepare.write_controller_config'), patch('builtins.print'):
                    prepare(self.profile)
                check_generated_model(self.profile)
