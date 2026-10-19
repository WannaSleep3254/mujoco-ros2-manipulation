"""Content manifests for generated models; usable from the ROS system Python."""

import hashlib
import json
from pathlib import Path
import re
import subprocess


GENERATOR_FILES = ('prepare_robot.py', 'robot_config.py', 'collada_visuals.py',
                   'model_integrity.py', 'requirements.txt', 'config/controller_defaults.yaml')


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def repository_commit(profile):
    repository = profile.source_path('repository_path')
    try:
        commit = subprocess.check_output(
            ['git', '-C', str(repository), 'rev-parse', 'HEAD'], text=True,
            stderr=subprocess.PIPE, timeout=10).strip()
        dirty = subprocess.check_output(
            ['git', '-C', str(repository), 'status', '--porcelain', '--untracked-files=no'],
            text=True, stderr=subprocess.PIPE, timeout=10).strip()
    except (OSError, subprocess.SubprocessError) as error:
        raise ValueError(f'{profile.id}: cannot inspect manufacturer checkout: {error}') from error
    if commit != profile.data['source']['commit']:
        raise ValueError(f'{profile.id}: manufacturer source revision changed')
    if dirty:
        raise ValueError(f'{profile.id}: manufacturer checkout has tracked changes; preserve or commit them separately')
    return commit


def required_inputs(profile):
    paths = {profile.path(name) for name in GENERATOR_FILES}
    paths.update(profile.source_path(name) for name in ('urdf', 'srdf'))
    if profile.data['source']['format'] == 'xacro':
        paths.update(profile.path(name) for name in ('manipulation_env.sh', 'ros2_env.sh'))
    return paths


def input_snapshot(profile, dependencies=()):
    paths = required_inputs(profile) | {Path(path).resolve() for path in dependencies}
    files = {}
    for path in sorted(paths):
        relative = path.relative_to(profile.root.resolve()).as_posix()
        checked = profile.path(relative)
        files[relative] = file_sha256(checked)
    return {'manufacturer_commit': repository_commit(profile), 'files': files}


def output_snapshot(directory, robot_id):
    directory = Path(directory)
    paths = [directory / f'{robot_id}.xml', directory / f'{robot_id}.ros2_control.urdf']
    paths += sorted((directory / 'visual_meshes').glob('*.obj'))
    return {path.relative_to(directory).as_posix(): file_sha256(path) for path in paths}


def ensure_inputs_unchanged(profile, snapshot):
    current = input_snapshot(profile, [profile.path(name) for name in snapshot['files']])
    if current != snapshot:
        raise ValueError('Model inputs changed during generation; retry after saving inputs')


def make_integrity(profile, dependencies, directory):
    return {'schema_version': 1, 'inputs': input_snapshot(profile, dependencies),
            'outputs': output_snapshot(directory, profile.id)}


def check_model_integrity(profile, metadata):
    """Fail closed on legacy, malformed, changed, or partially published builds."""
    hint = f'run .venv/bin/python prepare_robot.py --robot {profile.id}'
    try:
        integrity = metadata['integrity']
        if integrity['schema_version'] != 1:
            raise ValueError('unsupported integrity schema')
        inputs = integrity['inputs']
        files, outputs = inputs['files'], integrity['outputs']
        if not isinstance(files, dict) or not files or not isinstance(outputs, dict):
            raise ValueError('invalid file manifest')
        required = {path.relative_to(profile.root).as_posix() for path in required_inputs(profile)}
        if not required.issubset(files):
            raise ValueError('missing generator/source inputs')
        for name, digest in [*files.items(), *outputs.items()]:
            if not isinstance(name, str) or not isinstance(digest, str) or not re.fullmatch('[0-9a-f]{64}', digest):
                raise ValueError('invalid manifest path or digest')
        current = input_snapshot(profile, [profile.path(name) for name in files])
        if current != inputs:
            raise ValueError('generator, source, dependency or configuration inputs changed')
        expected = {f'{profile.id}.xml', f'{profile.id}.ros2_control.urdf'}
        if not expected.issubset(outputs):
            raise ValueError('missing model output digests')
        for name, digest in outputs.items():
            path = (profile.output / name).resolve()
            if Path(name).is_absolute() or not path.is_relative_to(profile.output.resolve()):
                raise ValueError('output path escapes model directory')
            if file_sha256(path) != digest:
                raise ValueError(f'generated output changed: {name}')
        # Additional/replaced visual assets must also invalidate a build.
        if output_snapshot(profile.output, profile.id) != outputs:
            raise ValueError('generated asset inventory changed')
    except (KeyError, TypeError, ValueError, OSError) as error:
        raise ValueError(f'{profile.id}: model integrity check failed ({error}); {hint}') from error


def read_metadata(path):
    try:
        result = json.loads(Path(path).read_text())
        if not isinstance(result, dict):
            raise ValueError('expected a JSON object')
        return result
    except (OSError, ValueError) as error:
        raise ValueError(f'Invalid model metadata: {path}: {error}') from error
