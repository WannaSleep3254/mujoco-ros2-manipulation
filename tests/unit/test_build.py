"""Build selections, source pins and cached dependencies fail closed."""
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml
from mujoco_ros2_manipulation.build import (ROOT, build_plan, load_manifest, local_path,
                                           resolve_source, verify_deb, prepare_debs, fetch_repository)
from mujoco_ros2_manipulation.profiles import load_profile


class BuildTests(unittest.TestCase):
    def test_model_selection_derives_packages_and_shared_repository(self):
        plan = build_plan(['fr5', 'fr10'])
        self.assertEqual(plan['repositories'], ['fairino'])
        self.assertEqual([p['name'] for p in plan['packages']],
                         ['fairino10_v6_moveit2_config', 'fairino5_v6_moveit2_config', 'fairino_description'])
        self.assertEqual(build_plan(['ur5e'])['repositories'], ['ur_description'])
        for name in plan['robots']:
            source = load_profile(name).data['source']
            pin = load_manifest()['repositories'][source['repository']]
            self.assertEqual(source['commit'], pin['commit'])

    def test_mixed_duplicate_source_pins_rejected(self):
        with self.assertRaises(ValueError):
            resolve_source({'repository': 'fairino', 'commit': '0'*40}, ROOT)

    def test_paths_and_malformed_locks_rejected(self):
        for path in ('../escape', '/tmp/external'):
            with self.assertRaises(ValueError): local_path(ROOT, path)
        manifest = load_manifest()
        for case in ('commit', 'deb', 'path', 'platform'):
            data = deepcopy(manifest)
            if case == 'commit': data['repositories']['fairino']['commit'] = 'main'
            elif case == 'deb': data['runtime']['debs'][0]['sha256'] = 'wrong'
            elif case == 'path': data['packages']['ur5e_moveit_config']['path'] = '../outside'
            else: data['platform']['architecture'] = 'arm64'
            with self.subTest(case=case), tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);(root/'config').mkdir();(root/'config/build.yaml').write_text(yaml.safe_dump(data))
                with self.assertRaises(ValueError): load_manifest(root)

    def test_modified_deb_cannot_reach_extraction(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); cache=root/'runtime/debs'; cache.mkdir(parents=True)
            entry=load_manifest()['runtime']['debs'][0]
            (cache/entry['file']).write_bytes(b'corrupt')
            with patch('mujoco_ros2_manipulation.build.run') as run:
                with self.assertRaisesRegex(ValueError, 'SHA-256'): prepare_debs(root, load_manifest())
                run.assert_not_called()

    def test_wrong_existing_checkout_never_rewritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); (root/'vendor').mkdir()
            with patch('mujoco_ros2_manipulation.build.subprocess.check_output', return_value='wrong'), \
                 patch('mujoco_ros2_manipulation.build.run') as run:
                with self.assertRaisesRegex(ValueError, 'left unchanged'):
                    fetch_repository(root, dict(path='vendor', commit='0'*40))
                run.assert_not_called()
