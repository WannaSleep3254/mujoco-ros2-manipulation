"""Defer termination until the current ROS callback has returned to Python."""
from contextlib import contextmanager
from dataclasses import dataclass
import signal


@dataclass
class StopRequest:
    requested: bool = False


@contextmanager
def cooperative_shutdown():
    """Set a main-thread flag, without raising inside a C extension response decoder."""
    stop = StopRequest()
    previous = {}

    def request_stop(signum, frame):
        stop.requested = True

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, request_stop)
        yield stop
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
