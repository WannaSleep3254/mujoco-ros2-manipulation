"""Atomic verification history and latest-result compatibility aliases."""

from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, value):
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write('\n')
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class VerificationReport:
    def __init__(self, root, robot_id, mode, domain):
        self.base = Path(root) / 'runtime' / 'validation'
        self.run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ') + '-' + uuid.uuid4().hex[:8]
        self.directory = self.base / 'runs' / self.run_id
        self.directory.mkdir(parents=True)
        self.path = self.directory / 'result.json'
        self.log_path = self.directory / 'launch.log'
        self.log_path.touch()
        self.latest = self.base / f'{robot_id}_{mode}.json'
        self.data = {'schema_version': 2, 'run_id': self.run_id, 'robot_id': robot_id,
                     'ros_domain_id': domain, 'status': 'running', 'stage': 'preflight',
                     'started_utc': utc_now(), 'completed_utc': None,
                     'motion_checks_passed': False, 'shutdown_clean': False,
                     'error': None, 'cleanup_errors': [], 'log_path': str(self.log_path)}
        with self.lock():
            if self.latest.is_file():
                try:
                    previous = json.loads(self.latest.read_text())
                except ValueError:
                    previous = {}
                if not isinstance(previous, dict) or not previous.get('run_id'):
                    legacy = self.base / 'runs' / ('legacy-' + uuid.uuid4().hex)
                    legacy.mkdir()
                    shutil.copy2(self.latest, legacy / 'result.json')
                    old_log = self.base / f'{robot_id}_{mode}.log'
                    if old_log.is_file():
                        shutil.copy2(old_log, legacy / 'launch.log')
            atomic_json(self.path, self.data)
            atomic_json(self.latest, self.data)
            alias = self.base / f'{robot_id}_{mode}.log'
            temporary = self.base / f'.{self.run_id}.log'
            temporary.symlink_to(self.log_path.relative_to(self.base))
            os.replace(temporary, alias)

    @contextmanager
    def lock(self):
        with (self.base / '.reports.lock').open('a') as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            yield

    def update(self, **fields):
        self.data.update(fields)
        with self.lock():
            atomic_json(self.path, self.data)
            current = json.loads(self.latest.read_text())
            # A slower older run must not replace a newer run's latest result.
            if current.get('run_id') == self.run_id:
                atomic_json(self.latest, self.data)

    def stage(self, name):
        self.update(stage=name)
