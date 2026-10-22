"""Real termination signals defer shutdown; genuine exceptions remain failures."""
import os
import signal
import unittest

from mujoco_ros2_manipulation.verification.signals import cooperative_shutdown


class CooperativeShutdownTests(unittest.TestCase):
    def test_real_sigint_and_sigterm_set_flag_without_interrupting_callback(self):
        original = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        for signum in original:
            with self.subTest(signum=signum):
                with cooperative_shutdown() as stop:
                    self.assertFalse(stop.requested)
                    os.kill(os.getpid(), signum)
                    self.assertTrue(stop.requested)
                    # This represents finishing response decoding and node cleanup.
                    os.kill(os.getpid(), signum)
                    self.assertTrue(stop.requested)
                self.assertEqual(original, {s: signal.getsignal(s) for s in original})

    def test_real_decoder_errors_propagate_even_after_stop_request(self):
        previous = signal.getsignal(signal.SIGINT)
        with self.assertRaisesRegex(SystemError, "genuine decoder failure"):
            with cooperative_shutdown():
                os.kill(os.getpid(), signal.SIGINT)
                raise SystemError("genuine decoder failure")
        self.assertEqual(signal.getsignal(signal.SIGINT), previous)

    def test_setup_failure_restores_handlers(self):
        previous = signal.getsignal(signal.SIGTERM)
        with self.assertRaisesRegex(RuntimeError, "node setup failed"):
            with cooperative_shutdown():
                raise RuntimeError("node setup failed")
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous)
