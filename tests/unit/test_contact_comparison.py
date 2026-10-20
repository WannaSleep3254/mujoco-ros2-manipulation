"""Reject weak or unmatched demonstrations of contact-caused interruption."""
from copy import deepcopy
import hashlib
import json
import unittest
from mujoco_ros2_manipulation.workcells.contact_comparison import compare_runs


def reports():
    commands = [
        dict(
            joints=["joint"],
            points=[
                dict(seconds=0, positions=[0]),
                dict(seconds=4, positions=[2]),
                dict(seconds=8, positions=[0]),
            ],
        )
    ]
    common = dict(
        robot_profile_sha256="profile",
        model_manifest_sha256="model",
        status="passed",
        motion_checks_passed=True,
        physics_checks_passed=True,
        shutdown_clean=True,
        process_group_gone=True,
        commanded_trajectories=commands,
        command_sha256=hashlib.sha256(
            json.dumps(commands, sort_keys=True).encode()
        ).hexdigest(),
        contact_injection_requests=0,
        verifier_cancel_requests=0,
        commanded_duration_sec=8,
        first_joint_displacement_rad={"left": 0.98, "right": 0.98},
    )
    contact = dict(
        deepcopy(common),
        run_id="contact",
        control_without_obstacle=False,
        goal_terminal_statuses=[5, 5],
        post_stop_drift_rad=0,
        physics=dict(steps=100, stop_step=100, stop_enabled=True),
        native_remaining_motion_at_contact_sec=6,
        native_contact_motion_elapsed_sec=2,
    )
    control = dict(
        deepcopy(common),
        run_id="control",
        control_without_obstacle=True,
        goal_terminal_statuses=[4, 4],
        physics=dict(contacts=0, stop_latched=False, stop_enabled=True),
        first_joint_peak_displacement_rad={"left": 1.99, "right": 1.99},
        first_joint_displacement_rad={"left": 0, "right": 0},
    )
    return contact, control


class ContactComparisonTests(unittest.TestCase):
    def test_identical_command_proves_interruption_and_successful_control(self):
        summary = compare_runs(*reports())
        self.assertTrue(summary["identical_commands"])
        self.assertEqual(summary["remaining_command_sec_at_contact"], 6)
        self.assertTrue(summary["control_returned_to_start"])

    def test_different_or_tampered_commands_are_not_a_control(self):
        contact, control = reports()
        control["commanded_trajectories"][0]["points"][1]["positions"] = [1]
        with self.assertRaises(ValueError):
            compare_runs(contact, control)
        control["command_sha256"] = hashlib.sha256(
            json.dumps(control["commanded_trajectories"], sort_keys=True).encode()
        ).hexdigest()
        with self.assertRaises(ValueError):
            compare_runs(contact, control)

    def test_normal_completion_near_end_contact_or_failed_control_are_rejected(self):
        for target, key, value in [
            (0, "goal_terminal_statuses", [4, 4]),
            (0, "native_remaining_motion_at_contact_sec", 0.2),
            (1, "goal_terminal_statuses", [5, 5]),
            (1, "first_joint_displacement_rad", {"left": 1, "right": 1}),
        ]:
            pair = list(reports())
            pair[target][key] = value
            with self.assertRaises(ValueError):
                compare_runs(*pair)
        contact, control = reports()
        control["physics"]["contacts"] = 1
        with self.assertRaises(ValueError):
            compare_runs(contact, control)
