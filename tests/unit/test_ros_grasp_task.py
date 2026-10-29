from copy import deepcopy
from types import SimpleNamespace as NS
import unittest

from mujoco_ros2_manipulation.grasping.ros_task import (
    checked_window, contact_evidence, trajectory_samples, RosGraspTaskError,
    kinematics_state_limits,
    latest_action_terminal, cancelled_hold_observation,
    admitted_action_submission,
    validate_dispatch_geometry,
)
from mujoco_ros2_manipulation.grasping.contract import GraspLifecycle
from mujoco_ros2_manipulation.workcells.pose_adapter import MoveItPoseAdapter
from tests.unit.test_ros_grasp_observation import sample


def trajectory(positions=None, velocities=None):
    positions = positions or [[0.]*6, [.1]*6]
    velocities = velocities or [[], []]
    return NS(joint_names=["left_j"+str(i) for i in range(1, 7)], points=[
        NS(positions=p, velocities=v, accelerations=[], time_from_start=NS(sec=i, nanosec=0))
        for i, (p, v) in enumerate(zip(positions, velocities))])


class RosGraspTaskTests(unittest.TestCase):
    def test_contact_evidence_requires_both_fingers_and_retains_unverified_force_timing(self):
        value = sample()
        value["contacts"] = [dict(geom1_name="fr5_block_geom", geom2_name=name, normal_force_n=force)
                             for name, force in (("fr5_left_finger_geom", .3), ("fr5_left_finger_geom", .4),
                                                 ("fr5_right_finger_geom", .1), ("fr5_table_geom", .8))]
        result = contact_evidence(value)
        self.assertAlmostEqual(result["finger_force_n"][0], .7)
        self.assertFalse(result["bilateral_contact"])
        self.assertAlmostEqual(result["table_force_n"], .8)
        self.assertFalse(result["force_timing_verified"])
        self.assertAlmostEqual(result["open_gap_m"], .07-sum(value["finger_positions_m"].values()))
        value["contacts"][2]["normal_force_n"] = .3
        self.assertTrue(contact_evidence(value)["bilateral_contact"])

    def test_relative_drift_detects_slip_while_common_world_motion_does_not(self):
        value = sample()
        reference = ((0., 0., -.1), (1., 0., 0., 0.))
        result = contact_evidence(value, reference)
        self.assertAlmostEqual(result["relative_position_error_m"], 0.)
        for key in ("block", "tcp"):
            value[key]["position"][0] += .5
        self.assertAlmostEqual(contact_evidence(value, reference)["relative_position_error_m"], 0.)
        value["block"]["position"][0] += .01
        self.assertAlmostEqual(contact_evidence(value, reference)["relative_position_error_m"], .01)

    def test_evidence_window_cannot_count_held_duplicate_or_missing_frames(self):
        values = [dict(sequence=i+1, time_sec=i*.05) for i in range(6)]
        self.assertAlmostEqual(checked_window(values, .2), .25)
        for bad in (values[:2], [values[0], values[0], values[-1]],
                    [values[0], values[1], dict(sequence=9, time_sec=.4)]):
            with self.assertRaises(RosGraspTaskError):
                checked_window(bad, .2)

    def test_actual_cubic_controller_spline_is_sampled_without_mutating_trajectory(self):
        path = trajectory(positions=[[0.]*6, [.1]*6], velocities=[[.2]*6, [-.1]*6])
        original = deepcopy(path)
        start = {name: 0. for name in path.joint_names}
        start["right_j1"] = .3
        limits = {name: dict(lower=-1., upper=1.) for name in start}
        curve, samples = trajectory_samples(path, start, path.joint_names, limits)
        middle = next(state for stamp, state in samples if stamp == .5)
        self.assertAlmostEqual(middle["left_j1"], .0875)  # cubic, not the .05 linear chord
        self.assertTrue(all(state["right_j1"] == .3 for _, state in samples))
        self.assertEqual(path, original)
        self.assertEqual(curve.duration, 1.)

    def test_nonselected_commands_and_spline_limit_overshoot_are_rejected(self):
        path = trajectory()
        start = {name: 0. for name in path.joint_names}
        limits = {name: dict(lower=-.01, upper=.2) for name in start}
        with self.assertRaisesRegex(RosGraspTaskError, "selected FR5"):
            trajectory_samples(path, start, path.joint_names[:-1], limits)
        overshoot = trajectory(positions=[[0.]*6, [.1]*6], velocities=[[2.]*6, [-2.]*6])
        with self.assertRaisesRegex(RosGraspTaskError, "crosses a joint limit"):
            trajectory_samples(overshoot, start, path.joint_names, limits)

    def test_nanometric_observed_finger_roundoff_preserves_raw_seed_and_command_limits(self):
        manifest = dict(arm_joint_limits={"left_j1": dict(lower=-1., upper=1.)},
                        finger_joint_limits={name: dict(lower=0., upper=.034) for name in
                            ("fr5_left_finger_slide", "fr5_right_finger_slide")})
        original = deepcopy(manifest)
        limits = kinematics_state_limits(manifest)
        names = list(limits)
        adapter = MoveItPoseAdapter(None, None, None, None, None, names, limits)
        seed = {name: 0. for name in names}
        seed["fr5_left_finger_slide"] = -8.4e-9
        self.assertEqual(adapter._state(seed)["fr5_left_finger_slide"], -8.4e-9)
        self.assertEqual(manifest, original)
        self.assertEqual(manifest["finger_joint_limits"]["fr5_left_finger_slide"], dict(lower=0., upper=.034))
        seed["fr5_left_finger_slide"] = -2e-6
        with self.assertRaisesRegex(ValueError, "out-of-limit"):
            adapter._state(seed)

    def test_pending_or_cancelled_transport_never_displays_prior_lift_success(self):
        actions = [dict(stage="lift", terminal_observed=True, terminal_status=4, terminal_error_code=0),
                   dict(stage="transport", terminal_observed=False)]
        value = latest_action_terminal(actions)
        self.assertEqual(value["stage"], "transport")
        self.assertIsNone(value["status"])
        actions[-1].update(terminal_observed=True, terminal_status=5, terminal_error_code=0)
        self.assertEqual(latest_action_terminal(actions)["status"], 5)

    def test_cancelled_stop_keeps_attachment_but_does_not_reconfirm_physical_grasp(self):
        life = GraspLifecycle("a"*64)
        life.approach_complete()
        life.contacts_confirmed(left_force_n=1., right_force_n=1., observed_sec=.2)
        life.lift_confirmed(fixture_sha256="a"*64, lift_m=.08, stable_sec=.3,
                            relative_position_error_m=.001, relative_angle_error_rad=.01, bilateral_fraction=1.)
        life.attach(object_id=life.object_id, owner=life.owner, link=life.link, fixture_sha256="a"*64)
        life.carry()
        life.cancel()
        with self.assertRaises(RosGraspTaskError):
            cancelled_hold_observation(life, stop_observed=False)
        value = cancelled_hold_observation(life, stop_observed=True)
        self.assertEqual(value["physical_grasp_before_cancel"], "confirmed")
        self.assertEqual(life.physical_grasp, "unconfirmed")
        self.assertEqual(life.planning_attachment, "attached")
        self.assertTrue(any(event["event"] == "physical_grasp_confirmed" for event in life.events))

    def test_cancel_during_preparation_callback_submits_zero_new_goals(self):
        cancelled, sent, identities = [False], [], []
        def check():
            if cancelled[0]:
                raise RosGraspTaskError("cancelled")
        def prepare():
            cancelled[0] = True  # same boundary as save/on_update or SIGINT
        with self.assertRaises(RosGraspTaskError):
            admitted_action_submission(check, prepare, lambda: identities.append(True), lambda: sent.append(True))
        self.assertEqual(sent, [])
        self.assertEqual(identities, [])

    def test_changed_target_or_source_after_preparation_submits_zero_new_goals(self):
        sent = []
        for message in ("target changed", "native source changed"):
            with self.subTest(message=message):
                def identity():
                    raise ValueError(message)
                with self.assertRaises(ValueError):
                    admitted_action_submission(lambda: None, lambda: None, identity, lambda: sent.append(True))
        self.assertEqual(sent, [])

    def test_cancel_arriving_during_identity_rpc_is_checked_again_before_send(self):
        cancelled, sent = [False], []
        def check():
            if cancelled[0]:
                raise RosGraspTaskError("late cancel")
        with self.assertRaises(RosGraspTaskError):
            admitted_action_submission(check, lambda: None, lambda: cancelled.__setitem__(0, True), lambda: sent.append(True))
        self.assertEqual(sent, [])

    def test_free_block_moving_during_plan_preparation_submits_zero_goals(self):
        value = sample()
        anchor = deepcopy(value)
        sent = []
        def prepare():
            value["block"]["position"][0] += .01
        def geometry():
            validate_dispatch_geometry(value, anchor_arms=anchor["arm_positions_rad"],
                anchor_fingers=anchor["finger_positions_m"], world_block_pose=(
                    tuple(anchor["block"]["position"]), tuple(anchor["block"]["quaternion_wxyz"])))
        with self.assertRaisesRegex(RosGraspTaskError, "free block moved"):
            admitted_action_submission(lambda: None, prepare, geometry, lambda: sent.append(True))
        self.assertEqual(sent, [])

    def test_lost_bilateral_contact_before_verification_lift_submits_zero_goals(self):
        value = sample()
        anchor = deepcopy(value)
        value["contacts"] = [dict(geom1_name="fr5_block_geom", geom2_name=name, normal_force_n=.4)
            for name in ("fr5_left_finger_geom", "fr5_right_finger_geom")]
        sent = []
        def prepare():
            value["contacts"].pop()
        def geometry():
            validate_dispatch_geometry(value, anchor_arms=anchor["arm_positions_rad"],
                anchor_fingers=anchor["finger_positions_m"], held_pose=[0., 0., -.1, 1., 0., 0., 0.])
        with self.assertRaisesRegex(RosGraspTaskError, "held block/contact changed"):
            admitted_action_submission(lambda: None, prepare, geometry, lambda: sent.append(True))
        self.assertEqual(sent, [])


if __name__ == "__main__":
    unittest.main()
