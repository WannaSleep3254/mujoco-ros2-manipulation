"""Server-side HTTP body accounting only; not client latency/decoding/paint."""
import json
import os
from pathlib import Path
import stat
import sys
import threading
import time

from mujoco_ros2_manipulation.operator_console.http_server import Handler
from mujoco_ros2_manipulation.operator_console.__main__ import main

LOG = Path('/tmp/viewer-ab-benchmark-20261026/http.jsonl')
_get, _respond = Handler.do_GET, Handler.respond
_log_lock = threading.Lock()
_log_fd = None


class _TimedWriter:
    def __init__(self, stream, sample):
        self.stream, self.sample = stream, sample

    def __getattr__(self, name):
        return getattr(self.stream, name)

    def write(self, data):
        if not self.sample['headers_done']:
            return self.stream.write(data)
        started = time.monotonic()
        try:
            result = self.stream.write(data)
            count = len(data) if result is None else result
            self.sample['body_write_bytes'] += count
            self.sample['body_write_completed'] = count == len(data)
            return result
        finally:
            self.sample['body_write_wall_sec'] += time.monotonic() - started


def _do_get(self):
    self._benchmark_get_started = time.monotonic()
    return _get(self)


def _logged_respond(self, data, status=200, mime='application/json; charset=utf-8'):
    started = time.monotonic()
    started_unix = time.time()
    body = data if isinstance(data, bytes) else (json.dumps(data, ensure_ascii=False, allow_nan=False)+'\n').encode()
    sample = dict(headers_done=False, body_write_bytes=0, body_write_wall_sec=0., body_write_completed=False)
    stream, end_headers = self.wfile, self.end_headers
    def finished_headers():
        result = end_headers()
        sample['headers_done'] = True
        return result
    self.wfile, self.end_headers = _TimedWriter(stream, sample), finished_headers
    try:
        return _respond(self, body, status=status, mime=mime)
    finally:
        ended = time.monotonic()
        ended_unix = time.time()
        self.wfile, self.end_headers = stream, end_headers
        get_started = getattr(self, '_benchmark_get_started', None) if self.command == 'GET' else None
        record = dict(method=self.command, path=self.path.partition('?')[0], status=status, mime=mime,
            is_probe=self.headers.get('X-Viewer-Benchmark-Probe') == '1', body_bytes=len(body),
            get_started_monotonic=get_started, respond_started_monotonic=started,
            respond_started_unix=started_unix, respond_finished_unix=ended_unix,
            respond_finished_monotonic=ended, respond_wall_sec=ended-started,
            get_wall_sec=None if get_started is None else ended-get_started,
            **{key:sample[key] for key in ('body_write_bytes','body_write_wall_sec','body_write_completed')})
        try:
            raw = (json.dumps(record, allow_nan=False, separators=(',', ':'))+'\n').encode()
            with _log_lock:
                offset = 0
                while offset < len(raw):
                    written = os.write(_log_fd, raw[offset:])
                    if written <= 0:
                        raise OSError('Incomplete benchmark log write')
                    offset += written
        except Exception:
            try:
                sys.stderr.write('HTTP benchmark accounting failed\n')
            except Exception:
                pass


if __name__ == '__main__':
    LOG.parent.mkdir(parents=True, exist_ok=True)
    _log_fd = os.open(LOG, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NONBLOCK | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    if not stat.S_ISREG(os.fstat(_log_fd).st_mode):
        raise ValueError('HTTP benchmark log must be a regular file')
    os.fchmod(_log_fd, 0o600)
    Handler.do_GET, Handler.respond = _do_get, _logged_respond
    try:
        raise SystemExit(main())
    finally:
        Handler.do_GET, Handler.respond = _get, _respond
        os.close(_log_fd)
