"""Compile the real C++ gate and force ordering/frozen-clock failure schedules."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from mujoco_ros2_manipulation.paths import ROOT

@unittest.skipUnless(shutil.which("g++"), "A C++17 compiler is needed for native gate verification")
class ControlLoopShutdownNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory(prefix="control-loop-shutdown-")
        cls.binary = Path(cls.folder.name)/"gate-test"
        result = subprocess.run(["g++", "-std=c++17", "-O1", "-pthread", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT/"native"), str(ROOT/"tests/native/control_loop_shutdown_test.cpp"), "-o", str(cls.binary)],
            capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout+result.stderr)
    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()
    def check_case(self, case):
        result = subprocess.run([str(self.binary), case], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertIn("PASS "+case, result.stdout)
    def test_teardown_waits_for_a_forcibly_held_update(self): self.check_case("held_update")
    def test_unsafe_old_order_exposes_overlap_under_the_same_schedule(self): self.check_case("unsafe_order_control")
    def test_stop_checked_before_each_read_update_and_write(self): self.check_case("cycle_boundaries")
    def test_never_started_clock_cannot_block_stop(self): self.check_case("missing_clock")
    def test_paused_clock_cannot_block_stop(self): self.check_case("paused_clock")
    def test_backwards_clock_rejected(self): self.check_case("backwards_clock")
    def test_clock_exception_is_explicit(self): self.check_case("clock_error")
    def test_stop_has_priority_over_a_ready_clock(self): self.check_case("stop_before_ready")
    def test_simulation_time_target_retained(self): self.check_case("ready_clock")
    def test_shutdown_before_thread_creation_is_safe(self): self.check_case("no_thread")

    def test_pending_management_callback_gets_both_rt_list_acknowledgements(self): self.check_case("drain_pending_management")
    def test_idle_group_keeps_rt_owner_available_until_spin_finishes(self): self.check_case("drain_late_callback")
    def test_shutdown_drain_waits_for_executor_cancellation(self): self.check_case("drain_cancel_ack")
    def test_missing_clock_has_no_drain_control_cycle(self): self.check_case("drain_missing_clock")
    def test_final_executor_completion_between_read_and_update_is_respected(self): self.check_case("drain_finished_during_read")
    def test_context_cleanup_waits_for_callback_drain_and_rt_join(self): self.check_case("drain_before_cleanup")
    def test_drain_rejects_zero_wall_period(self): self.check_case("drain_invalid_period")

    def test_management_group_missing_duplicate_type_and_ready_contracts(self): self.check_case("management_group_validation")
