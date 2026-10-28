"""Build selections, source pins and cached dependencies fail closed."""
from copy import deepcopy
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml
from mujoco_ros2_manipulation.build import (ROOT, build_plan, load_manifest, local_path,
                                           resolve_source, verify_deb, prepare_debs, fetch_repository,
                                           colcon, RUNTIME_PATCH_KEYS)
from mujoco_ros2_manipulation.profiles import load_profile


class BuildTests(unittest.TestCase):
    def test_both_shutdown_patches_are_required_confined_and_ordered_last(self):
        manifest=load_manifest()
        self.assertEqual(RUNTIME_PATCH_KEYS, ('patch','monitor_patch','contact_stop_patch',
            'clock_watchdog_patch','control_loop_shutdown_patch','activation_shutdown_patch'))
        self.assertEqual(manifest['runtime']['control_loop_shutdown_patch'],
            'patches/mujoco_ros2_control-0.1.2-ordered-shutdown.patch')
        self.assertEqual(manifest['runtime']['activation_shutdown_patch'],
            'patches/mujoco_ros2_control-0.1.2-activation-shutdown.patch')
        self.assertEqual(len(set(manifest['runtime'][key] for key in RUNTIME_PATCH_KEYS)),6)
        for key in ('control_loop_shutdown_patch','activation_shutdown_patch'):
            for value in (None,'../outside.patch','/tmp/outside.patch'):
                with self.subTest(key=key,value=value),tempfile.TemporaryDirectory() as temporary:
                    root=Path(temporary);(root/'config').mkdir()
                    changed=deepcopy(manifest)
                    if value is None:del changed['runtime'][key]
                    else:changed['runtime'][key]=value
                    (root/'config/build.yaml').write_text(yaml.safe_dump(changed))
                    with self.assertRaises(ValueError):load_manifest(root)

    def test_previously_published_ordered_shutdown_patch_is_immutable(self):
        path=ROOT/'patches/mujoco_ros2_control-0.1.2-ordered-shutdown.patch'
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
            '5cd77481de464aa850b73dd4533c088da8138231d64cdacdf24a79a506077fbf')

    def test_runtime_colcon_binds_both_actual_native_header_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);(root/'native').mkdir()
            monitor=b'fixed monitor fixture\n';shutdown=b'fixed ordered shutdown fixture\n'
            (root/'native/step_monitor.hpp').write_bytes(monitor)
            (root/'native/control_loop_shutdown.hpp').write_bytes(shutdown)
            with patch('mujoco_ros2_manipulation.build.run') as run:
                colcon(root,[dict(name='mujoco_ros2_control',path=str(root/'source'))],runtime=True)
            command=run.call_args.args[0]
            self.assertIn('-DMANIPULATION_MONITOR_INCLUDE_DIR='+str(root/'native'),command)
            self.assertIn('-DMANIPULATION_SHUTDOWN_INCLUDE_DIR='+str(root/'native'),command)
            self.assertIn('-DMANIPULATION_MONITOR_SHA256='+hashlib.sha256(monitor).hexdigest(),command)
            self.assertIn('-DMANIPULATION_SHUTDOWN_SHA256='+hashlib.sha256(shutdown).hexdigest(),command)
            self.assertIn('-DCMAKE_BUILD_TYPE=RelWithDebInfo',command)
            self.assertIn('-DBUILD_TESTING=OFF',command)
            (root/'native/control_loop_shutdown.hpp').unlink()
            with patch('mujoco_ros2_manipulation.build.run') as run:
                with self.assertRaises(FileNotFoundError):
                    colcon(root,[dict(name='mujoco_ros2_control',path=str(root/'source'))],runtime=True)
                run.assert_not_called()

    def test_model_only_colcon_does_not_require_or_inject_native_shutdown_headers(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            with patch('mujoco_ros2_manipulation.build.run') as run:
                colcon(root,[dict(name='model_configuration',path=str(root/'model'))])
            command=run.call_args.args[0]
            self.assertFalse(any('MANIPULATION_SHUTDOWN' in str(item) or 'MANIPULATION_MONITOR' in str(item)
                                 for item in command))

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
