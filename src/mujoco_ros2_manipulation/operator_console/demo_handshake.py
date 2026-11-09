"""Bounded cancellation IPC for one exact owned comparison worker."""
import json
import os
import stat
from pathlib import Path
import time

CANCEL_GRACE_SEC = 45
IPC_LIMIT = 4096


def bounded_regular_bytes(path, maximum):
    """Never block on a FIFO or follow a replaced sidecar outside its owner."""
    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(descriptor, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or not 0 <= info.st_size <= maximum:
            raise ValueError('Comparison record is not a bounded regular file')
        raw = stream.read(maximum + 1)
    if len(raw) > maximum:
        raise ValueError('Comparison record grew beyond its bounded size')
    return raw


def read_handshake(path, kind, job_id, generation, identity, *, now_ns=None):
    """An absent record is pending. Invalid or stale ownership is a real error."""
    if kind not in ('ready', 'cancel'):
        raise ValueError('Unknown comparison worker IPC kind')
    try:
        raw = bounded_regular_bytes(Path(path), IPC_LIMIT)
    except FileNotFoundError:
        return None
    value = json.loads(raw)
    stamps = {'ready_monotonic_ns'} if kind == 'ready' else {'requested_monotonic_ns', 'deadline_monotonic_ns'}
    fields = {'schema_version', 'kind', 'job_id', 'generation', 'pid', 'start_ticks'} | stamps
    if (type(value) is not dict or set(value) != fields or type(value['schema_version']) is not int or value['schema_version'] != 1
            or value['kind'] != kind or value['job_id'] != job_id or value['generation'] != generation
            or identity is None or type(value['pid']) is not int or value['pid'] <= 1
            or type(value['start_ticks']) is not int or value['start_ticks'] <= 0
            or any(value[name] != identity[name] for name in ('pid', 'start_ticks'))
            or any(type(value[name]) is not int or value[name] <= 0 for name in stamps)):
        raise ValueError('Comparison worker IPC does not match this job, generation and process')
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    if kind == 'ready':
        valid_time = value['ready_monotonic_ns'] <= now_ns
    else:
        valid_time = (value['requested_monotonic_ns'] <= now_ns
            and value['deadline_monotonic_ns'] - value['requested_monotonic_ns'] == CANCEL_GRACE_SEC * 1_000_000_000)
    if not valid_time:
        raise ValueError('Comparison worker IPC has an invalid original deadline or timestamp')
    return value
