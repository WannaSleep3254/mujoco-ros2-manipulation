"""Build selections, source pins and cached dependencies fail closed."""
from copy import deepcopy
import hashlib
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml
from mujoco_ros2_manipulation.build import (ROOT, build_plan, load_manifest, local_path,
                                           resolve_source, verify_deb, prepare_debs, fetch_repository,
                                           colcon, RUNTIME_PATCH_KEYS, native_distance_command, runtime_plan,
                                           check_action_server_abi, check_action_clock_runtime)
from mujoco_ros2_manipulation.profiles import load_profile


class BuildTests(unittest.TestCase):
    def test_runtime_plan_builds_only_the_pinned_action_package_and_native_owners(self):
        manifest = load_manifest()
        plan = runtime_plan(manifest)
        self.assertEqual(plan['repositories'], ['mujoco_ros2_control', 'rclcpp'])
        self.assertEqual([item['name'] for item in plan['packages']],
                         ['rclcpp_action', 'mujoco_ros2_control', 'manipulation_grasp_plugin'])
        self.assertEqual(plan['packages'][0]['path'], str(ROOT/'external/rclcpp/rclcpp_action'))
        self.assertEqual(manifest['repositories']['rclcpp']['commit'],
                         'c07d2a5e9974bb9788c9dbb76cbda1c73973a5be')
        self.assertEqual(build_plan(['fr5'])['repositories'], ['fairino'])

    def test_action_overlay_requires_its_pinned_package_and_confined_patch(self):
        manifest = load_manifest()
        for field, value in (('action_server_package', None), ('action_server_package', 'rclcpp'),
                             ('action_server_patch', '../outside.patch'),
                             ('action_server_patch', '/tmp/outside.patch')):
            with self.subTest(field=field, value=value), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                (root/'config').mkdir()
                changed = deepcopy(manifest)
                if value is None:
                    del changed['runtime'][field]
                else:
                    changed['runtime'][field] = value
                (root/'config/build.yaml').write_text(yaml.safe_dump(changed))
                with self.assertRaises(ValueError):
                    load_manifest(root)

    def test_action_overlay_rejects_a_different_installed_or_source_cpp_abi(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, underlay = root/'source', root/'underlay'
            files = (source/'rclcpp/package.xml', source/'rclcpp_action/package.xml',
                     underlay/'share/rclcpp/package.xml', underlay/'share/rclcpp_action/package.xml')
            for path in files:
                path.parent.mkdir(parents=True)
                path.write_text('<package><version>16.0.21</version></package>')
            check_action_server_abi(source, underlay)
            for path in files:
                with self.subTest(path=str(path)):
                    path.write_text('<package><version>different</version></package>')
                    with self.assertRaisesRegex(ValueError, '16.0.21'):
                        check_action_server_abi(source, underlay)
                    path.write_text('<package><version>16.0.21</version></package>')
            files[0].write_text('<invalid')
            with self.assertRaisesRegex(ValueError, 'ABI versions'):
                check_action_server_abi(source, underlay)

    def test_only_selected_action_library_gets_its_compiled_patch_identity(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            digest = 'a'*64
            package = dict(name='rclcpp_action', path=str(root/'external/rclcpp/rclcpp_action'))
            with patch('mujoco_ros2_manipulation.build.run') as run:
                colcon(root, [package], cmake_args=['-DMANIPULATION_ACTION_CLOCK_SHA256='+digest])
            command = run.call_args.args[0]
            self.assertIn('-DMANIPULATION_ACTION_CLOCK_SHA256='+digest, command)
            selected = command[command.index('--packages-select')+1:command.index('--cmake-args')]
            self.assertEqual(selected, ['rclcpp_action'])
            self.assertIn('-DBUILD_TESTING=OFF', command)
            self.assertEqual(command[command.index('--allow-overriding')+1], 'rclcpp_action')

    def test_action_runtime_rejects_missing_stale_or_shadowed_library(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'config').mkdir()
            manifest = load_manifest()
            (root/'config/build.yaml').write_text(yaml.safe_dump(manifest))
            source = root/manifest['runtime']['action_server_patch']
            source.parent.mkdir()
            source.write_bytes(b'current isolated action clock source patch')
            binary = root/'ros_ws/install/rclcpp_action/lib/librclcpp_action.so'
            binary.parent.mkdir(parents=True)
            with patch.dict('os.environ', {'LD_LIBRARY_PATH': str(binary.parent)}):
                with self.assertRaisesRegex(ValueError, 'Build the isolated'):
                    check_action_clock_runtime(root)
                binary.write_bytes(b'old system library')
                with self.assertRaisesRegex(ValueError, 'Build the isolated'):
                    check_action_clock_runtime(root)
                raw = b'ELF fixture '+hashlib.sha256(source.read_bytes()).hexdigest().encode()
                binary.write_bytes(raw)
                self.assertIsNone(check_action_clock_runtime(root))
                shadow = root/'shadow'
                shadow.mkdir()
                (shadow/binary.name).write_bytes(raw)
                with patch.dict('os.environ', {'LD_LIBRARY_PATH': str(shadow)+':'+str(binary.parent)}):
                    with self.assertRaisesRegex(ValueError, 'different action clock library'):
                        check_action_clock_runtime(root)
                with patch.dict('os.environ', {'LD_LIBRARY_PATH': ''}):
                    with self.assertRaisesRegex(ValueError, 'Source manipulation_env.sh'):
                        check_action_clock_runtime(root)
                source.write_bytes(b'changed patch')
                with self.assertRaisesRegex(ValueError, 'Build the isolated'):
                    check_action_clock_runtime(root)

    def test_action_runtime_matches_loader_empty_entries_and_unset_search_path(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/'config').mkdir()
            manifest = load_manifest()
            (root/'config/build.yaml').write_text(yaml.safe_dump(manifest))
            source = root/manifest['runtime']['action_server_patch']
            source.parent.mkdir()
            source.write_bytes(b'fixed action clock patch')
            binary = root/'ros_ws/install/rclcpp_action/lib/librclcpp_action.so'
            binary.parent.mkdir(parents=True)
            binary.write_bytes(hashlib.sha256(source.read_bytes()).hexdigest().encode())
            (root/binary.name).write_bytes(b'stale library found through current directory')
            previous = Path.cwd()
            try:
                os.chdir(root)
                for search in (':'+str(binary.parent), str(root/'empty')+'::'+str(binary.parent), ''):
                    with self.subTest(search=search), patch.dict(os.environ, {'LD_LIBRARY_PATH': search}):
                        with self.assertRaisesRegex(ValueError, 'different action clock library'):
                            check_action_clock_runtime(root)
                with patch.dict(os.environ, {'LD_LIBRARY_PATH': str(binary.parent)+':'}):
                    self.assertIsNone(check_action_clock_runtime(root))
                # An unset search path does not imply cwd, even if cwd has the correct library.
                os.chdir(binary.parent)
                with patch.dict(os.environ, clear=True):
                    with self.assertRaisesRegex(ValueError, 'Source manipulation_env.sh'):
                        check_action_clock_runtime(root)
            finally:
                os.chdir(previous)

    def test_native_distance_build_pins_each_scope_and_requires_new_sidecar(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);(root/'native').mkdir()
            source_files=('path_distance.cpp','path_distance_environment.cpp','path_distance_self.cpp')
            macros=('MANIPULATION_DISTANCE_SHA256','MANIPULATION_ENVIRONMENT_DISTANCE_SHA256',
                    'MANIPULATION_SELF_DISTANCE_SHA256')
            for name in source_files:(root/'native'/name).write_text('independent '+name)
            vendor=root/'vendor';output=root/'path_distance.so'
            command=native_distance_command(root,vendor,output)
            for name,macro in zip(source_files,macros):
                digest=hashlib.sha256((root/'native'/name).read_bytes()).hexdigest()
                self.assertIn('-D'+macro+'="'+digest+'"',command)
            self.assertIn(vendor/'lib/libmujoco.so.3.12.0',command)
            self.assertIn(root/'native/path_distance_environment.cpp',command)
            self.assertEqual(command[-2:],['-o',output])
            (root/'native/path_distance_self.cpp').unlink()
            with self.assertRaises(FileNotFoundError):native_distance_command(root,vendor,output)

    def test_both_shutdown_patches_are_required_confined_and_ordered_last(self):
        manifest=load_manifest()
        self.assertEqual(RUNTIME_PATCH_KEYS, ('patch','monitor_patch','contact_stop_patch',
            'clock_watchdog_patch','control_loop_shutdown_patch','activation_shutdown_patch','grasp_policy_patch'))
        self.assertEqual(manifest['runtime']['control_loop_shutdown_patch'],
            'patches/mujoco_ros2_control-0.1.2-ordered-shutdown.patch')
        self.assertEqual(manifest['runtime']['activation_shutdown_patch'],
            'patches/mujoco_ros2_control-0.1.2-activation-shutdown.patch')
        self.assertEqual(len(set(manifest['runtime'][key] for key in RUNTIME_PATCH_KEYS)),7)
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
            policy=b'fixed grasp contact policy fixture\n'
            (root/'native/grasp_policy.hpp').write_bytes(policy)
            with patch('mujoco_ros2_manipulation.build.run') as run:
                colcon(root,[dict(name='mujoco_ros2_control',path=str(root/'source'))],runtime=True)
            command=run.call_args.args[0]
            self.assertIn('-DMANIPULATION_MONITOR_INCLUDE_DIR='+str(root/'native'),command)
            self.assertIn('-DMANIPULATION_SHUTDOWN_INCLUDE_DIR='+str(root/'native'),command)
            self.assertIn('-DMANIPULATION_MONITOR_SHA256='+hashlib.sha256(monitor).hexdigest(),command)
            self.assertIn('-DMANIPULATION_GRASP_POLICY_SHA256='+hashlib.sha256(policy).hexdigest(),command)
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
