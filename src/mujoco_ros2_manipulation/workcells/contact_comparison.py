"""Compare matched contact and no-obstacle runs without ROS or GUI dependencies."""
import hashlib
import json


def compare_runs(contact, control):
    for report in (contact, control):
        if (
            not all(
                report.get(k) is True
                for k in (
                    "motion_checks_passed",
                    "shutdown_clean",
                    "process_group_gone",
                    "physics_checks_passed",
                )
            )
            or report.get("status") != "passed"
        ):
            raise ValueError("Both runs must pass motion, physics and shutdown checks")
        commands = report["commanded_trajectories"]
        digest = hashlib.sha256(
            json.dumps(commands, sort_keys=True).encode()
        ).hexdigest()
        if digest != report["command_sha256"]:
            raise ValueError("Command digest does not match the recorded command")
        if (
            report.get("contact_injection_requests") != 0
            or report.get("verifier_cancel_requests") != 0
        ):
            raise ValueError(
                "Comparison must use autonomous physical contact and cancellation"
            )
    if any(
        contact.get(key) != control.get(key)
        for key in ("robot_profile_sha256", "model_manifest_sha256")
    ):
        raise ValueError("Contact and control must use the same robot/workcell model")
    if not all(r["physics"].get("stop_enabled") is True for r in (contact, control)):
        raise ValueError(
            "The same contact stop interlock must remain enabled in both runs"
        )
    if contact["command_sha256"] != control["command_sha256"]:
        raise ValueError("Contact and control must execute identical complete commands")
    if (
        contact.get("control_without_obstacle") is not False
        or control.get("control_without_obstacle") is not True
    ):
        raise ValueError("Expected one obstacle run and one no-obstacle control")
    if contact["goal_terminal_statuses"] != [5, 5] or control[
        "goal_terminal_statuses"
    ] != [4, 4]:
        raise ValueError("Expected canceled contact goals and successful control goals")
    if (
        contact["post_stop_drift_rad"] != 0
        or contact["physics"]["steps"] != contact["physics"]["stop_step"]
    ):
        raise ValueError("Contact run did not remain frozen")
    if control["physics"]["contacts"] != 0 or control["physics"]["stop_latched"]:
        raise ValueError("Control must complete with zero contacts and no stop latch")
    remaining = contact["native_remaining_motion_at_contact_sec"]
    if remaining < 4.5:
        raise ValueError(
            "Insufficient command remains after contact to demonstrate interruption"
        )
    extra = {
        arm: control["first_joint_peak_displacement_rad"][arm]
        - contact["first_joint_displacement_rad"][arm]
        for arm in contact["first_joint_displacement_rad"]
    }
    if any(v < 0.7 for v in extra.values()) or any(
        v > 0.01 for v in control["first_joint_displacement_rad"].values()
    ):
        raise ValueError("Control must move substantially past contact and return")
    return dict(
        contact_run_id=contact["run_id"],
        control_run_id=control["run_id"],
        command_sha256=contact["command_sha256"],
        identical_commands=True,
        commanded_motion_duration_sec=contact["commanded_duration_sec"],
        native_contact_motion_elapsed_sec=contact["native_contact_motion_elapsed_sec"],
        remaining_command_sec_at_contact=remaining,
        control_additional_outbound_rad=extra,
        contact_goal_statuses=contact["goal_terminal_statuses"],
        control_goal_statuses=control["goal_terminal_statuses"],
        control_returned_to_start=True,
        control_contacts=0,
        contact_post_stop_drift_rad=0,
        additional_contact_physics_steps=0,
    )
