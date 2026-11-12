"""Keep ROS launch SIGINT handling cooperative without interrupting its tasks."""

import asyncio
import signal


def main(argv=None):
    from ros2cli.cli import main as ros_main

    previous = signal.getsignal(signal.SIGINT)
    if previous is not signal.default_int_handler:
        return ros_main(argv=argv)

    from launch.utilities.signal_management import AsyncSafeSignalManager

    startup_signal_pending = False
    startup_signal_replaying = False

    def replay_startup_signal(signum):
        nonlocal startup_signal_pending, startup_signal_replaying
        startup_signal_replaying = True
        try:
            signal.raise_signal(signum)
        finally:
            startup_signal_replaying = False
            startup_signal_pending = False

    def cooperative_sigint(signum, frame):
        nonlocal startup_signal_pending
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            signal.default_int_handler(signum, frame)
            return
        # Humble installs its wakeup FD before registering __current. Queue
        # one delivery during startup. The FD-before-registration gap can
        # deliver twice; LaunchService ignores the second SIGINT.
        manager = getattr(AsyncSafeSignalManager, '_AsyncSafeSignalManager__current', None)
        if manager is None:
            if startup_signal_replaying:
                signal.default_int_handler(signum, frame)
            elif not startup_signal_pending:
                startup_signal_pending = True
                loop.call_soon(replay_startup_signal, signum)

    signal.signal(signal.SIGINT, cooperative_sigint)
    try:
        return ros_main(argv=argv)
    finally:
        signal.signal(signal.SIGINT, previous)


if __name__ == '__main__':
    raise SystemExit(main())
