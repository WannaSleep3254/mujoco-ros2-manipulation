"""Exact, owned-process crash injection; ordinary verification never permits crashes."""

from pathlib import Path
import os
import re
import signal


def process_failures(log):
    return [
        dict(node=name, pid=int(pid), exit_code=int(code))
        for name, pid, code in re.findall(
            r"\[ERROR\] \[([^\]]+)\]: process has died \[pid (\d+), exit code (-?\d+)",
            log,
        )
        if int(code) != 0
    ]


class ExpectedCrash:
    def __init__(self, target):
        if target not in ("move_group", "ros2_control_node"):
            raise ValueError("Unsupported crash target")
        self.target = target
        self.expected = None

    def inject(self, log, group):
        if self.expected is not None:
            raise RuntimeError("Only one crash may be injected per launch")
        matches = re.findall(
            r"\[INFO\] \[("
            + re.escape(self.target)
            + r"-\d+)\]: process started with pid \[(\d+)\]",
            log,
        )
        if len(matches) != 1:
            raise RuntimeError("Crash target is missing or ambiguous in this launch")
        name, pid = matches[0]
        pid = int(pid)
        # Bind the signal to this process instance, not a potentially reused PID.
        fd = os.pidfd_open(pid)
        try:
            if (
                os.getpgid(pid) != group
                or Path(f"/proc/{pid}/exe").resolve().name != self.target
            ):
                raise RuntimeError(
                    "Refusing to signal an unowned or mismatched process"
                )
            signal.pidfd_send_signal(fd, signal.SIGKILL)
        finally:
            os.close(fd)
        self.expected = dict(node=name, pid=pid, exit_code=-signal.SIGKILL)
        return dict(self.expected)

    def verify(self, log, returncode):
        failures = process_failures(log)
        if self.expected is None or failures != [self.expected]:
            raise RuntimeError(
                f"Expected exactly the injected crash; observed {failures}"
            )
        if returncode != 0:
            raise RuntimeError(
                f"Launch did not complete orderly shutdown: {returncode}"
            )
        return failures
