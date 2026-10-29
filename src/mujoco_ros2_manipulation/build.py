"""Manifest-driven source fetching, locked deb extraction and colcon builds."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import tempfile

import yaml
from mujoco_ros2_manipulation.paths import ROOT


RUNTIME_PATCH_KEYS = ('patch', 'monitor_patch', 'contact_stop_patch', 'clock_watchdog_patch',
                      'control_loop_shutdown_patch', 'activation_shutdown_patch', 'grasp_policy_patch')


def local_path(root, name):
    if not isinstance(name, str) or Path(name).is_absolute():
        raise ValueError(f'Expected workspace-relative path: {name}')
    result = (root / name).resolve()
    if not result.is_relative_to(root.resolve()):
        raise ValueError(f'Path escapes workspace: {name}')
    return result


def load_manifest(root=ROOT):
    data = yaml.safe_load((root / 'config/build.yaml').read_text())
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('Expected build schema_version: 1')
    try:
        if data['platform'] != dict(os='ubuntu', version='22.04', ros_distro='humble', architecture='amd64'):
            raise ValueError('Only Ubuntu 22.04 / ROS Humble / amd64 is currently validated')
        for name, repo in data['repositories'].items():
            local_path(root, repo['path'])
            if not re.fullmatch('[0-9a-f]{40}', repo['commit']) or not repo['url'].startswith('https://github.com/'):
                raise ValueError(f'Invalid source lock: {name}')
        for name, package in data['packages'].items():
            base = root
            if 'repository' in package:
                base = local_path(root, data['repositories'][package['repository']]['path'])
            local_path(base, package['path'])
        runtime = data['runtime']
        for key in RUNTIME_PATCH_KEYS:
            local_path(root, runtime[key])
        if runtime['repository'] not in data['repositories'] or runtime['package'] not in data['packages']:
            raise ValueError('Unknown runtime repository/package')
        seen = set()
        for deb in runtime['debs']:
            if deb['package'] in seen or not re.fullmatch('[0-9a-f]{64}', deb['sha256']):
                raise ValueError('Duplicate or invalid deb lock')
            seen.add(deb['package'])
            if Path(deb['file']).name != deb['file'] or not deb['file'].endswith('.deb') or deb['architecture'] != 'amd64':
                raise ValueError('Invalid deb filename/architecture')
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError(f'Malformed build manifest: {error}') from error
    return data


def resolve_source(source, root):
    """Normalize profile references without duplicating manufacturer source pins."""
    if 'repository' not in source:
        # Compatibility for explicit temporary fixture profiles only.
        return dict(source)
    if {'repository_path', 'repository_url', 'commit'} & source.keys():
        raise ValueError('Use source.repository without duplicate source pins')
    if source['repository'] not in load_manifest(root)['repositories']:
        raise ValueError(f'Unknown source repository: {source["repository"]}')
    repo = load_manifest(root)['repositories'][source['repository']]
    return dict(source, repository_path=repo['path'], repository_url=repo['url'], commit=repo['commit'])


def build_plan(robot_ids=None, root=ROOT):
    from mujoco_ros2_manipulation.profiles import load_profile
    manifest = load_manifest(root)
    available = sorted(path.stem for path in (root / 'config/robots').glob('*.yaml'))
    profiles = [load_profile(name, require_enabled=False, root=root) for name in (robot_ids or available)]
    profiles = [profile for profile in profiles if profile.data['enabled']]
    if not profiles:
        raise ValueError('No enabled robots selected')
    names = set()
    repositories = set()
    for profile in profiles:
        repositories.add(profile.data['source']['repository'])
        names.add(profile.ros['moveit_package'])
        names.update(profile.data['source']['mesh_packages'])
    packages = []
    for name in sorted(names):
        definition = manifest['packages'][name]
        base = root
        if 'repository' in definition:
            repositories.add(definition['repository'])
            base = local_path(root, manifest['repositories'][definition['repository']]['path'])
        packages.append({'name': name, 'path': str(local_path(base, definition['path']))})
    return {'robots': [profile.id for profile in profiles], 'repositories': sorted(repositories), 'packages': packages}


def run(command, **kwargs):
    print('+ ' + ' '.join(map(str, command)), flush=True)
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def fetch_repository(root, repo):
    path = local_path(root, repo['path'])
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        # Failed downloads never leave a partial checkout at the final path.
        with tempfile.TemporaryDirectory(prefix='.fetch-', dir=path.parent) as temporary:
            run(['git', 'init', '--quiet', temporary])
            run(['git', '-C', temporary, 'remote', 'add', 'origin', repo['url']])
            run(['git', '-C', temporary, 'fetch', '--depth', '1', 'origin', repo['commit']])
            run(['git', '-C', temporary, 'checkout', '--detach', 'FETCH_HEAD'])
            os.rename(temporary, path)
    current = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    if current != repo['commit']:
        raise ValueError(f'Source revision mismatch: {path}; existing checkout left unchanged')
    return path


def verify_deb(path, entry):
    if hashlib.sha256(path.read_bytes()).hexdigest() != entry['sha256']:
        raise ValueError(f'Deb SHA-256 mismatch: {path.name}')
    for field, expected in [('Package', entry['package']), ('Version', entry['version']),
                            ('Architecture', entry['architecture'])]:
        actual = subprocess.check_output(['dpkg-deb', '-f', str(path), field], text=True).strip()
        if actual != expected:
            raise ValueError(f'Deb {field} mismatch: {path.name}')


def prepare_debs(root, manifest, download=False):
    cache = root / 'runtime/debs'
    cache.mkdir(parents=True, exist_ok=True)
    entries = manifest['runtime']['debs']
    # Validate the whole set before extracting any package. Unlisted cache files are ignored.
    for entry in entries:
        path = cache / entry['file']
        if not path.is_file() and download:
            with tempfile.TemporaryDirectory(prefix='.download-', dir=cache) as temp:
                run(['apt-get', 'download', f"{entry['package']}={entry['version']}"], cwd=temp)
                matches = list(Path(temp).glob('*.deb'))
                if len(matches) != 1:
                    raise ValueError('Expected one downloaded deb')
                verify_deb(matches[0], entry)
                os.replace(matches[0], path)
        if not path.is_file():
            raise ValueError(f'Missing locked deb: {path.name}; use --download-debs')
        verify_deb(path, entry)
    for entry in entries:
        run(['dpkg-deb', '-x', cache / entry['file'], root / 'runtime/root'])


def apply_patch_series(source, patches):
    """Recognize exact patch prefixes even when later patches change earlier hunks."""
    source = Path(source)
    patches = [Path(p).resolve() for p in patches]
    names = set()
    for patch in patches:
        for row in subprocess.check_output(['git', 'apply', '--numstat', str(patch)], text=True).splitlines():
            name = row.split('\t', 2)[2]
            local_path(source, name)
            names.add(name)
    def snapshot(root):
        return {name: (root / name).read_bytes() if (root / name).is_file() else None for name in names}
    current = snapshot(source)
    with tempfile.TemporaryDirectory(prefix='runtime-patch-check-') as temporary:
        shadow = Path(temporary)
        for name in names:
            original = subprocess.run(['git', '-C', str(source), 'show', 'HEAD:' + name], capture_output=True)
            if original.returncode == 0:
                path = shadow / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(original.stdout)
        prefixes = [snapshot(shadow)]
        for patch in patches:
            subprocess.run(['git', 'apply', '--check', str(patch)], cwd=shadow, check=True, capture_output=True)
            subprocess.run(['git', 'apply', str(patch)], cwd=shadow, check=True, capture_output=True)
            prefixes.append(snapshot(shadow))
    matched = [index for index, expected in enumerate(prefixes) if current == expected]
    if not matched:
        raise ValueError('Runtime source has incompatible changes; left unchanged')
    for patch in patches[max(matched):]:
        run(['git', '-C', source, 'apply', '--check', patch])
        run(['git', '-C', source, 'apply', patch])


def colcon(root, packages, runtime=False):
    command = ['colcon', '--log-base', 'ros_ws/log', 'build', '--base-paths']
    command += [entry['path'] for entry in packages]
    command += ['--build-base', 'ros_ws/build', '--install-base', 'ros_ws/install', '--packages-select']
    command += [entry['name'] for entry in packages]
    command += ['--cmake-args', '-DBUILD_TESTING=OFF']
    if runtime:
        command += ['-DCMAKE_BUILD_TYPE=RelWithDebInfo',
                    f'-DMANIPULATION_MONITOR_INCLUDE_DIR={root / "native"}',
                    '-DMANIPULATION_MONITOR_SHA256=' + hashlib.sha256((root / 'native/step_monitor.hpp').read_bytes()).hexdigest(),
                    '-DMANIPULATION_GRASP_POLICY_SHA256=' + hashlib.sha256((root / 'native/grasp_policy.hpp').read_bytes()).hexdigest(),
                    f'-DMANIPULATION_SHUTDOWN_INCLUDE_DIR={root / "native"}',
                    '-DMANIPULATION_SHUTDOWN_SHA256=' + hashlib.sha256((root / 'native/control_loop_shutdown.hpp').read_bytes()).hexdigest()]
    run(['bash', '-c', 'source "$1"\nexport CMAKE_PREFIX_PATH="$2/runtime/root/usr:$2/runtime/root/opt/ros/humble:${CMAKE_PREFIX_PATH:-}"\nshift 2\nexec "$@"',
         'manipulation-build', root / 'manipulation_env.sh', root, *command], cwd=root)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--component', choices=['all', 'runtime', 'models'], default='all')
    parser.add_argument('--robots', nargs='+')
    parser.add_argument('--workcell', help='build all models required by a shared workcell')
    parser.add_argument('--scene', default='default')
    parser.add_argument('--dry-run', action='store_true', help='validate and show the build plan without writes/network')
    parser.add_argument('--fetch-only', action='store_true')
    parser.add_argument('--download-debs', action='store_true')
    args = parser.parse_args()
    try:
        manifest = load_manifest()
        cell = None
        if args.workcell:
            if args.robots or args.scene != 'default' or args.component == 'runtime':
                raise ValueError('--workcell requires all/models and cannot be combined with --robots or --scene')
            from mujoco_ros2_manipulation.workcells.config import load_workcell
            cell = load_workcell(args.workcell)
            args.robots = sorted({inst.profile.id for inst in cell.instances})
        plan = build_plan(args.robots)
        if cell is not None:
            plan['workcell'] = cell.id
        from mujoco_ros2_manipulation.scenes import load_scene
        load_scene(args.scene, ROOT)
        if args.component == 'runtime':
            runtime = manifest['runtime']
            definition = manifest['packages'][runtime['package']]
            source = local_path(ROOT, manifest['repositories'][runtime['repository']]['path'])
            plan = dict(robots=[], repositories=[runtime['repository']], packages=[
                dict(name=runtime['package'], path=str(local_path(source, definition['path'])))])
        if args.component in ('all', 'runtime'):
            plan['repositories'] = sorted(set(plan['repositories']) | {manifest['runtime']['repository']})
        if args.dry_run:
            print(json.dumps(dict(component=args.component, scene=args.scene, **plan), indent=2))
            return 0
        if platform.machine() != 'x86_64':
            raise ValueError('Locked native runtime requires amd64')
        for name in plan['repositories']:
            fetch_repository(ROOT, manifest['repositories'][name])
        if args.fetch_only:
            return 0
        if args.component in ('all', 'runtime'):
            runtime = manifest['runtime']
            source = local_path(ROOT, manifest['repositories'][runtime['repository']]['path'])
            apply_patch_series(source, [local_path(ROOT, runtime[key])
                for key in RUNTIME_PATCH_KEYS])
            prepare_debs(ROOT, manifest, args.download_debs)
            native = ROOT / 'runtime/native'
            native.mkdir(parents=True, exist_ok=True)
            run(['gcc', '-shared', '-fPIC', '-O2', '-Wall', '-Wextra', '-Werror', '-o', native / 'moveit_plugin_lifetime.so',
                 ROOT / 'native/moveit_plugin_lifetime.c', '-ldl', '-pthread'])
            vendor = ROOT / 'runtime/root/opt/ros/humble/opt/mujoco_vendor'
            run(['g++', '-std=c++17', '-shared', '-fPIC', '-O2', '-Wall', '-Wextra', '-Werror',
                 '-DMANIPULATION_DISTANCE_SHA256="' + hashlib.sha256((ROOT / 'native/path_distance.cpp').read_bytes()).hexdigest() + '"',
                 '-DMANIPULATION_ENVIRONMENT_DISTANCE_SHA256="' + hashlib.sha256((ROOT / 'native/path_distance_environment.cpp').read_bytes()).hexdigest() + '"',
                 '-I' + str(vendor / 'include'), ROOT / 'native/path_distance_environment.cpp',
                 vendor / 'lib/libmujoco.so.3.12.0', '-Wl,-rpath,' + str(vendor / 'lib'),
                 '-o', native / 'path_distance.so'])
            run(['g++', '-std=c++17', '-shared', '-fPIC', '-O2', '-Wall', '-Wextra', '-Werror',
                 '-DMANIPULATION_GRASP_DISTANCE_SHA256="' + hashlib.sha256((ROOT / 'native/grasp_distance.cpp').read_bytes()).hexdigest() + '"',
                 '-DMANIPULATION_GRASP_POLICY_SHA256="' + hashlib.sha256((ROOT / 'native/grasp_policy.hpp').read_bytes()).hexdigest() + '"',
                 '-I' + str(vendor / 'include'), ROOT / 'native/grasp_distance.cpp',
                 vendor / 'lib/libmujoco.so.3.12.0', '-Wl,-rpath,' + str(vendor / 'lib'),
                 '-o', native / 'grasp_distance.so'])
            definition = manifest['packages'][runtime['package']]
            colcon(ROOT, [dict(name=runtime['package'], path=str(local_path(source, definition['path']))),
                          dict(name='manipulation_grasp_plugin', path=str(local_path(ROOT,
                              manifest['packages']['manipulation_grasp_plugin']['path'])))], True)
        if args.component in ('all', 'models'):
            # Reject dirty manufacturer sources, including mesh edits, before colcon.
            from mujoco_ros2_manipulation.model_build.integrity import repository_commit
            from mujoco_ros2_manipulation.profiles import load_profile
            for name in plan['robots']:
                repository_commit(load_profile(name, scene=args.scene))
            colcon(ROOT, plan['packages'])
            python = ROOT / '.venv/bin/python'
            for name in plan['robots']:
                run([python, ROOT / 'prepare_robot.py', '--robot', name, '--scene', args.scene], cwd=ROOT)
            if cell is not None:
                run([python, ROOT / 'prepare_workcell.py', '--workcell', cell.id], cwd=ROOT)
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f'Build failed: {error}\n')
    return 0
