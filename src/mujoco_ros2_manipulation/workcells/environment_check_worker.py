"""One owned .venv CPU overlay preflight; no ROS, GL, physics steps or commands."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import stat
import sys
import uuid

MAX_RECEIPT_BYTES = 64 * 1024


def _require(value, message):
    if not value:
        raise ValueError(message)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
        allow_nan=False).encode('ascii')


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _load_base(root, identifier):
    from .config import load_workcell
    return load_workcell(identifier, root=root)


def _check_full(directory, base, digest):
    from .environment_overlay import check_overlay
    return check_overlay(directory, base, expected_sha256=digest)


def _parent(expected):
    _require(type(expected) is int and expected > 1 and os.getppid() == expected,
        'Overlay checker lost its direct parent')


def check_and_receipt(args):
    _parent(args.parent_pid)
    _require(re.fullmatch('[0-9a-f]{64}', args.manifest_sha256)
        and re.fullmatch('[0-9a-f]{64}', args.worker_sha256)
        and re.fullmatch('[0-9a-f]{32}', args.request_token), 'Invalid owned checker identity')
    _require(_sha(Path(__file__).read_bytes()) == args.worker_sha256, 'Another checker implementation was selected')
    directory = Path(args.overlay).resolve(strict=True)
    receipt = Path(args.receipt).absolute()
    parent = receipt.parent.resolve(strict=True)
    _require(parent.is_dir() and stat.S_IMODE(parent.stat().st_mode) == 0o700
        and not receipt.exists() and not receipt.is_symlink()
        and not parent.is_relative_to(directory), 'Receipt requires a new private owned directory')
    base = _load_base(Path(args.workspace).resolve(strict=True), args.base_workcell)
    report = _check_full(directory, base, args.manifest_sha256)
    _parent(args.parent_pid)
    result = dict(schema_version=1, artifact_kind='environment-overlay-check-v1', request_token=args.request_token,
        worker_sha256=args.worker_sha256, overlay_manifest_sha256=args.manifest_sha256,
        overlay_id=report['id'], overlay_fingerprint=report['fingerprint'],
        base_workcell_id=report['base_workcell_id'], base_workcell_sha256=report['inputs']['base_workcell_sha256'],
        base_manifest_sha256=report['inputs']['base_manifest_sha256'], report_sha256=_sha(_canonical(report)),
        geometry_check=deepcopy(report['geometry_check']))
    raw = _canonical(result) + b'\n'
    _require(len(raw) <= MAX_RECEIPT_BYTES, 'Overlay checker receipt exceeded its bound')
    temporary = parent / ('.receipt-' + uuid.uuid4().hex)
    try:
        with temporary.open('xb') as stream:
            os.chmod(temporary, 0o600)
            stream.write(raw)
        _parent(args.parent_pid)
        # The parent supplied an absent path in its private 0700 temporary folder.
        # Exclusive publication prevents replacing an unexpected pre-existing file.
        os.link(temporary, receipt, follow_symlinks=False)
    finally:
        temporary.unlink(missing_ok=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('workspace', 'base-workcell', 'overlay', 'manifest-sha256', 'request-token', 'worker-sha256', 'receipt'):
        parser.add_argument('--' + name, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--cpu-limit-sec', type=int, required=True)
    args = parser.parse_args()
    try:
        _require(1 <= args.cpu_limit_sec <= 62, 'Invalid checker CPU limit')
        resource.setrlimit(resource.RLIMIT_CPU, (args.cpu_limit_sec, args.cpu_limit_sec + 1))
        resource.setrlimit(resource.RLIMIT_AS, (2 * 1024**3, 2 * 1024**3))
        resource.setrlimit(resource.RLIMIT_FSIZE, (1024 * 1024, 1024 * 1024))
        check_and_receipt(args)
    except Exception as error:
        # Detailed engine exceptions can contain design filenames/private paths.
        print('Environment overlay CPU check failed: ' + type(error).__name__, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
