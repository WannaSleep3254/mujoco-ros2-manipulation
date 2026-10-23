"""Startup-only local overlay catalog; this parent never imports a scene engine.

Only a trusted local CLI calls load_scene_catalog. Browser callers select the
returned declared IDs; they cannot provide a catalog, path, checker or converter.
"""
from copy import deepcopy
import hashlib
from importlib.util import find_spec
import json
import math
import os
from pathlib import Path
import re
import secrets
import stat
import subprocess
import tempfile

from .scene_host import SceneSource

MAX_CATALOG_BYTES = 64 * 1024
MAX_CATALOG_ENTRIES = 8
MAX_RECEIPT_BYTES = 64 * 1024
WORKER_MODULE = 'mujoco_ros2_manipulation.workcells.environment_check_worker'
RECEIPT_KIND = 'environment-overlay-check-v1'


def _require(value, message):
    if not value:
        raise ValueError(message)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
                      allow_nan=False).encode('ascii')


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _pairs(items):
    result = {}
    for key, value in items:
        _require(key not in result, 'Duplicate local scene catalog field')
        result[key] = value
    return result


def _json(raw):
    def number(text):
        value = float(text)
        _require(math.isfinite(value), 'Nonfinite local scene metadata')
        return value
    return json.loads(raw, object_pairs_hook=_pairs, parse_float=number,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite local scene metadata')))


def _regular(path, maximum):
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0))
    try:
        info = os.fstat(descriptor)
        _require(stat.S_ISREG(info.st_mode) and info.st_size <= maximum, 'Expected bounded regular scene metadata')
        with os.fdopen(descriptor, 'rb', closefd=False) as stream:
            raw = stream.read(maximum + 1)
        _require(len(raw) <= maximum, 'Scene metadata grew beyond its byte limit')
        return raw
    finally:
        os.close(descriptor)


def _identity(directory, base, expected_sha256):
    # A pure checker rehashes the immutable bytes/dependencies. The full CPU
    # check is deliberately confined to the separately owned .venv worker.
    from ..workcells.environment_overlay import check_overlay_identity
    return check_overlay_identity(directory, base, expected_sha256=expected_sha256)


def _worker_source_sha256():
    spec = find_spec(WORKER_MODULE)
    _require(spec is not None and spec.origin is not None, 'Local overlay checker is unavailable')
    return _sha(_regular(Path(spec.origin), 1024 * 1024))


def _receipt(report, *, token, manifest_sha256, worker_sha256):
    return dict(schema_version=1, artifact_kind=RECEIPT_KIND, request_token=token,
        worker_sha256=worker_sha256, overlay_manifest_sha256=manifest_sha256,
        overlay_id=report['id'], overlay_fingerprint=report['fingerprint'],
        base_workcell_id=report['base_workcell_id'],
        base_workcell_sha256=report['inputs']['base_workcell_sha256'],
        base_manifest_sha256=report['inputs']['base_manifest_sha256'],
        report_sha256=_sha(_canonical(report)), geometry_check=deepcopy(report['geometry_check']))


def _stop_owned(process):
    """Only this direct Popen child is signaled; never discover or kill by name."""
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)
    _require(process.poll() is not None, 'Owned overlay checker did not stop')


def _full_check(directory, base, manifest_sha256, before, python, timeout_sec):
    token = secrets.token_hex(16)
    worker_sha256 = _worker_source_sha256()
    expected = _receipt(before, token=token, manifest_sha256=manifest_sha256, worker_sha256=worker_sha256)
    with tempfile.TemporaryDirectory(prefix='environment-check-') as temporary:
        owned = Path(temporary)
        receipt = owned / 'receipt.json'
        process = None
        with (owned / 'checker.log').open('wb') as log:
            try:
                process = subprocess.Popen([str(python), '-m', WORKER_MODULE,
                    '--workspace', str(base.root), '--base-workcell', base.id,
                    '--overlay', str(directory), '--manifest-sha256', manifest_sha256,
                    '--request-token', token, '--worker-sha256', worker_sha256,
                    '--receipt', str(receipt), '--parent-pid', str(os.getpid()),
                    '--cpu-limit-sec', str(int(math.ceil(timeout_sec)) + 2)],
                    cwd=base.root, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True, env=dict(os.environ, MUJOCO_ROS2_WORKSPACE=str(base.root),
                        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
                process.wait(timeout=timeout_sec)
                _require(process.returncode == 0, 'Overlay CPU validation failed')
                checked = _json(_regular(receipt, MAX_RECEIPT_BYTES))
                _require(type(checked) is dict and type(checked.get('schema_version')) is int
                    and checked == expected, 'Overlay CPU receipt differs from the frozen artifact')
            except BaseException as error:
                if process is not None:
                    try:
                        _stop_owned(process)
                    except Exception as cleanup:
                        raise RuntimeError('Owned overlay checker cleanup failed') from error
                raise


def _entries(path):
    path = Path(path).absolute()
    value = _json(_regular(path, MAX_CATALOG_BYTES))
    _require(type(value) is dict and set(value) == {'schema_version', 'overlays'}
        and type(value['schema_version']) is int and value['schema_version'] == 1
        and type(value['overlays']) is list and 1 <= len(value['overlays']) <= MAX_CATALOG_ENTRIES,
        'Expected a bounded explicit local overlay catalog')
    entries, seen = [], set()
    for item in value['overlays']:
        _require(type(item) is dict and set(item) == {'directory', 'manifest_sha256'}, 'Invalid overlay catalog entry')
        raw, digest = item['directory'], item['manifest_sha256']
        _require(isinstance(raw, str) and 0 < len(raw) <= 4096 and '\0' not in raw
            and isinstance(digest, str) and re.fullmatch('[0-9a-f]{64}', digest), 'Invalid overlay path or SHA256')
        candidate = Path(raw)
        if not candidate.is_absolute():
            candidate = path.parent / candidate
        _require(candidate.is_dir() and not candidate.is_symlink(), 'Expected an existing unlinked overlay directory')
        directory = candidate.resolve(strict=True)
        _require(directory not in seen, 'Duplicate overlay directory')
        seen.add(directory)
        entries.append((directory, digest))
    return entries


def _source(directory, digest, report, base):
    _require(type(report) is dict and type(report.get('schema_version')) is int and report['schema_version'] == 1
        and report.get('artifact_kind') == 'environment-overlay-v1'
        and isinstance(report.get('id'), str) and re.fullmatch('[a-z][a-z0-9_]{0,79}', report['id'])
        and report['id'] != base.id and report.get('base_workcell_id') == base.id
        and report.get('joint_names') == list(base.joints) and report.get('home_rad') == list(base.home),
        'Overlay source does not describe the checked base joints')
    frozen = deepcopy(report)
    token = dict(schema_version=1, artifact_kind='environment-overlay-v1', manifest_sha256=digest,
        fingerprint=report['fingerprint'], report_sha256=_sha(_canonical(report)))
    def validate():
        current = _identity(directory, base, digest)
        _require(current == frozen, 'Registered overlay changed; explicitly validate a new immutable artifact')
        return deepcopy(token)
    return SceneSource('scene:' + report['id'], '설계 환경 · ' + report['id'], directory / 'cell.xml',
        tuple(report['joint_names']), dict(zip(report['joint_names'], report['home_rad'])), validate)


def load_scene_catalog(path, base_cell, *, python=None, timeout_sec=30.0):
    """Fully preflight explicit local entries, then return detached preview sources.

    There is no web path input, source discovery, model build, ROS launch or
    command. A failure returns no partial catalog; each owned CPU child is reaped.
    """
    _require(type(timeout_sec) in (int, float) and math.isfinite(timeout_sec) and .1 <= timeout_sec <= 60,
        'Overlay CPU timeout must be 0.1..60 seconds')
    python = Path(python) if python is not None else base_cell.root / '.venv/bin/python'
    _require(python.is_file(), 'The local overlay checking Python is unavailable')
    sources, identifiers = [], set()
    for directory, digest in _entries(path):
        before = _identity(directory, base_cell, digest)
        source = _source(directory, digest, before, base_cell)
        _require(source.id not in identifiers, 'Duplicate declared overlay source ID')
        _full_check(directory, base_cell, digest, before, python, float(timeout_sec))
        after = _identity(directory, base_cell, digest)
        _require(after == before, 'Overlay changed while CPU validation was running')
        source.validate()
        identifiers.add(source.id)
        sources.append(source)
    return tuple(sources)
