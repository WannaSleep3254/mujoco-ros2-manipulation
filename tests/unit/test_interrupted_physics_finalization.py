"""Interrupted observations retain strict native checks without completed payloads."""
import ast
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.operator_console.demo_worker import DemoCancelled
from mujoco_ros2_manipulation.verification.diagnostics import finalize_physics
from mujoco_ros2_manipulation.verification.reports import VerificationReport
from mujoco_ros2_manipulation.workcells.contact_demo import OBSTACLE_BODY
from tests.unit.test_diagnostics import clean_report
from tests.unit import test_verification_reports as verification_fixtures


def require(value, message):
    if not value:
        raise RuntimeError(message)


def load_demo_finalizer():
    """Run the real finalizer without importing ROS-only verifier classes."""
    source = Path(__file__).resolve().parents[2] / "src/mujoco_ros2_manipulation/workcells/verify_contact_demo.py"
    tree = ast.parse(source.read_text(), filename=str(source))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "finalize_demo")
    namespace = dict(finalize_physics=finalize_physics, file_sha256=file_sha256,
                     require=require, OBSTACLE_BODY=OBSTACLE_BODY)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])), str(source), "exec"), namespace)
    return namespace["finalize_demo"]


def native_report(root, run_id, *, stopped=True):
    value = clean_report()
    value.update(run_id=run_id, monitor_source_sha256=file_sha256(root / "native/step_monitor.hpp"),
                 stop_enabled=True, stop_latched=stopped)
    if stopped:
        value.update(contact_steps=1, contacts=1, active_contacts=1,
            stop_reason="contact", stop_step=100, hold_ticks=20, hold_time_sec=.24,
            stop_time_sec=.2, last_state_time_sec=.2, stopped_qpos=[0.] * 12,
            pairs=[dict(occurrences=1, bodies=["left_wrist2_link", OBSTACLE_BODY], min_distance_m=-.0002)])
    return value


class InterruptedFinalizationTests(unittest.TestCase):
    finalize_demo = staticmethod(load_demo_finalizer())

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        header = self.root / "native/step_monitor.hpp"
        header.parent.mkdir()
        header.write_text("native fixture source")
        self.report = VerificationReport(self.root, "contact", "headless", 201)
        (self.report.directory / "contact_test.xml").write_text("<mujoco/>")
        self.cell = NS(root=self.root, data=dict(physics=dict(timestep=.002)))
        self.args = NS(contact_stop_test=True, clock_stop_test=False, control=False)
        self.native = native_report(self.root, self.report.run_id)
        self.write()

    def write(self):
        (self.report.directory / "physics.json").write_text(json.dumps(self.native))

    def interrupt(self, error_type="DemoCancelled"):
        self.report.update(error=dict(type=error_type, message="original interruption"),
                           failure_stage="approach_fixed_obstacle")

    def test_latched_cancellation_retains_native_proof_without_reading_missing_motion_payload(self):
        self.interrupt()
        original = deepcopy(self.report.data["error"])
        self.finalize_demo(self.report, self.cell, self.args)
        value = json.loads(self.report.path.read_text())
        self.assertEqual(value["error"], original)
        self.assertEqual(value["failure_stage"], "approach_fixed_obstacle")
        self.assertEqual(value["physics"], self.native)
        self.assertTrue(value["physics_verification"]["native_checks_passed"])
        self.assertEqual(value["physics_verification"]["observed_condition"], "contact_stop")
        self.assertTrue(value["physics_verification"]["interrupted"])
        self.assertTrue(value["physics_verification"]["incomplete"])
        self.assertFalse(value["physics_checks_passed"])
        self.assertFalse(value["contact_demo_complete"])
        self.assertFalse(value["fixed_obstacle_contact_checks_passed"])
        self.assertNotIn("native_remaining_motion_at_contact_sec", value)
        self.assertNotIn("diagnostics_error", value)

    def test_early_cancellation_checks_only_actual_zero_contact_observation(self):
        self.interrupt()
        self.native = native_report(self.root, self.report.run_id, stopped=False)
        self.write()
        self.finalize_demo(self.report, self.cell, self.args)
        verification = self.report.data["physics_verification"]
        self.assertEqual(verification["observed_condition"], "zero_contact")
        self.assertTrue(verification["native_checks_passed"])
        self.assertFalse(self.report.data["physics_checks_passed"])
        self.assertFalse(self.report.data["fixed_obstacle_contact_checks_passed"])

    def test_interrupted_clock_test_distinguishes_observed_clock_stop_from_early_cancel(self):
        self.interrupt("KeyboardInterrupt")
        self.args.contact_stop_test, self.args.clock_stop_test = False, True
        self.native.update(stop_reason="clock_stall", contacts=0, contact_steps=0,
                           active_contacts=0, pairs=[])
        self.write()
        verification = finalize_physics(self.report, self.cell, self.args)
        self.assertEqual(verification["observed_condition"], "clock_stop")
        self.assertTrue(verification["native_checks_passed"])
        self.assertFalse(self.report.data["physics_checks_passed"])
        self.native = native_report(self.root, self.report.run_id, stopped=False)
        self.write()
        verification = finalize_physics(self.report, self.cell, self.args)
        self.assertEqual(verification["observed_condition"], "zero_contact")
        self.native = native_report(self.root, self.report.run_id)
        self.write()
        with self.assertRaisesRegex(ValueError, "zero-contact latched stop"):
            finalize_physics(self.report, self.cell, self.args)

    def test_interruption_does_not_relax_native_identity_coverage_or_latched_hold(self):
        self.interrupt()
        original = deepcopy(self.native)
        for field, value in (("run_id", "foreign"), ("monitor_source_sha256", "old"),
                             ("invalid_values", 1), ("engine_steps", 99),
                             ("timestep_sec", .003), ("hold_ticks", 0)):
            with self.subTest(field=field):
                self.native = dict(deepcopy(original), **{field: value})
                self.write()
                with self.assertRaises(ValueError):
                    self.finalize_demo(self.report, self.cell, self.args)
                self.assertEqual(self.report.data["error"]["type"], "DemoCancelled")
                self.assertFalse(self.report.data["physics_checks_passed"])
                self.assertFalse(self.report.data["physics_verification"]["native_checks_passed"])

    def test_interrupted_demo_does_not_hide_an_unexpected_contact_pair(self):
        self.interrupt()
        self.native["pairs"][0]["bodies"] = ["left_wrist2_link", "unexpected_obstacle"]
        self.write()
        with self.assertRaisesRegex(RuntimeError, "Unexpected or deep contact"):
            self.finalize_demo(self.report, self.cell, self.args)
        self.assertEqual(self.report.data["error"]["type"], "DemoCancelled")
        self.assertNotIn("contact_demo_complete", self.report.data)

    def test_completed_demo_still_requires_contact_and_complete_motion_payload(self):
        with self.assertRaisesRegex(KeyError, "motion_end_clock_sec"):
            self.finalize_demo(self.report, self.cell, self.args)
        self.report.update(motion_checks_passed=True, motion_start_clock_sec=0., motion_end_clock_sec=8.)
        self.finalize_demo(self.report, self.cell, self.args)
        self.assertTrue(self.report.data["physics_checks_passed"])
        self.assertTrue(self.report.data["fixed_obstacle_contact_checks_passed"])
        self.assertTrue(self.report.data["contact_demo_complete"])
        self.assertFalse(self.report.data["physics_verification"]["interrupted"])
        self.native = native_report(self.root, self.report.run_id, stopped=False)
        self.write()
        with self.assertRaisesRegex(ValueError, "complete latched stop"):
            self.finalize_demo(self.report, self.cell, self.args)

    def test_keyboard_interrupt_remains_distinct_from_demo_cancellation_and_other_failures(self):
        self.interrupt("KeyboardInterrupt")
        self.finalize_demo(self.report, self.cell, self.args)
        self.assertEqual(self.report.data["error"]["type"], "KeyboardInterrupt")
        self.assertTrue(self.report.data["physics_verification"]["interrupted"])
        self.interrupt("RuntimeError")
        verification = finalize_physics(self.report, self.cell, self.args)
        self.assertFalse(verification["interrupted"])
        self.assertEqual(self.report.data["error"]["type"], "RuntimeError")

    def test_completed_control_and_interrupted_control_have_distinct_pass_flags(self):
        self.args.control, self.args.contact_stop_test = True, False
        self.native = native_report(self.root, self.report.run_id, stopped=False)
        self.write()
        self.finalize_demo(self.report, self.cell, self.args)
        self.assertTrue(self.report.data["control_no_contact_checks_passed"])
        self.interrupt()
        self.finalize_demo(self.report, self.cell, self.args)
        self.assertFalse(self.report.data["control_no_contact_checks_passed"])
        self.assertFalse(self.report.data["physics_checks_passed"])

    def test_runner_preserves_original_cancellation_and_clean_completed_finalization(self):
        case = verification_fixtures.LifecycleTests()
        case.setUp()
        self.addCleanup(case.doCleanups)
        case.profile.root = case.root
        case.profile.data = dict(physics=dict(timestep=.002))
        case.args.contact_stop_test, case.args.control = True, False
        header = case.root / "native/step_monitor.hpp"
        header.parent.mkdir()
        header.write_text("native fixture source")
        case.node.run.side_effect = DemoCancelled("original cancel")

        def spawn(*args, **kwargs):
            directory = Path(kwargs["stdout"].name).parent
            run_id = json.loads((directory / "result.json").read_text())["run_id"]
            (directory / "physics.json").write_text(json.dumps(native_report(case.root, run_id)))
            (directory / "contact_test.xml").write_text("<mujoco/>")
            return case.process

        code, value = case.run_case(spawn=spawn, finalize=self.finalize_demo)
        self.assertEqual(code, 130)
        self.assertEqual(value["status"], "interrupted")
        self.assertEqual(value["error"], dict(type="DemoCancelled", message="original cancel"))
        self.assertTrue(value["shutdown_clean"])
        self.assertTrue(value["process_group_gone"])
        self.assertEqual(value["cleanup_errors"], [])
        self.assertFalse(value["motion_checks_passed"])
        self.assertFalse(value["physics_checks_passed"])
        self.assertNotIn("diagnostics_error", value)
        finalize = value["cleanup_diagnostics"]["steps"][-1]
        self.assertEqual(finalize["name"], "finalize")
        self.assertEqual(finalize["status"], "completed")
