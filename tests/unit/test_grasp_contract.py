"""Task evidence gates, ownership and cancellation are independent of physics."""
from copy import deepcopy
import math
import unittest

from mujoco_ros2_manipulation.grasping.contract import GraspLifecycle, GraspTransitionError
from mujoco_ros2_manipulation.grasping.fixture import FixtureSpec
from mujoco_ros2_manipulation.grasping.run import invalid_attachment_receipt


SHA = "a" * 64


def ready_to_lift():
    life = GraspLifecycle(SHA)
    life.approach_complete()
    life.contacts_confirmed(left_force_n=1., right_force_n=1., observed_sec=.2)
    return life


def holding():
    life = ready_to_lift()
    life.lift_confirmed(fixture_sha256=SHA, lift_m=.1, stable_sec=.2,
        relative_position_error_m=.001, relative_angle_error_rad=.01, bilateral_fraction=1.)
    return life


def attached():
    life = holding()
    life.attach(object_id="block", owner="synthetic_gripper", link="carriage", fixture_sha256=SHA)
    return life


class GraspLifecycleTests(unittest.TestCase):
    def test_bilateral_contact_is_not_yet_a_physical_grasp(self):
        life = ready_to_lift()
        self.assertEqual(life.physical_grasp, "unconfirmed")
        self.assertEqual(life.planning_attachment, "world")
        before = life.snapshot()
        with self.assertRaises(GraspTransitionError):
            life.attach(object_id="block", owner="synthetic_gripper", link="carriage", fixture_sha256=SHA)
        self.assertEqual(life.snapshot(), before)

    def test_contact_evidence_requires_both_sides_and_observation_window(self):
        for changes in ({"left_force_n": .1}, {"right_force_n": .1}, {"observed_sec": .09},
                        {"left_force_n": True}, {"observed_sec": math.nan}):
            with self.subTest(changes=changes):
                life = GraspLifecycle(SHA)
                life.approach_complete()
                before = life.snapshot()
                with self.assertRaises(GraspTransitionError):
                    life.contacts_confirmed(**dict(dict(left_force_n=1., right_force_n=1., observed_sec=.2), **changes))
                self.assertEqual(life.snapshot(), before)

    def test_lift_requires_motion_stability_contact_and_same_identity(self):
        valid = dict(fixture_sha256=SHA, lift_m=.1, stable_sec=.2,
            relative_position_error_m=.001, relative_angle_error_rad=.01, bilateral_fraction=1.)
        for key, value in (("fixture_sha256", "b"*64), ("lift_m", .059), ("stable_sec", .19),
                           ("relative_position_error_m", .0051), ("relative_angle_error_rad", .151),
                           ("bilateral_fraction", .94), ("bilateral_fraction", 1.01)):
            with self.subTest(key=key, value=value):
                life = ready_to_lift()
                before = life.snapshot()
                with self.assertRaises(GraspTransitionError):
                    life.lift_confirmed(**dict(valid, **{key: value}))
                self.assertEqual(life.snapshot(), before)

    def test_wrong_owners_links_objects_sha_and_duplicate_attachment_are_rejected(self):
        receipt = invalid_attachment_receipt()
        self.assertTrue(receipt["passed"])
        self.assertEqual(receipt["rejected"], 6)
        self.assertTrue(receipt["state_unchanged"])
        self.assertIn("wrong_owner", [row["condition"] for row in receipt["conditions"]])

    def test_logical_attachment_does_not_change_the_physical_grasp(self):
        life = holding()
        physical = life.physical_grasp
        life.attach(object_id="block", owner="synthetic_gripper", link="carriage", fixture_sha256=SHA)
        self.assertEqual(life.physical_grasp, physical)
        self.assertEqual(life.planning_attachment, "attached")
        life.carry()
        life.fail("Measured relative movement", slipped=True)
        self.assertEqual(life.physical_grasp, "slipped")
        self.assertEqual(life.planning_attachment, "attached")
        self.assertTrue(life.events[-1]["explicit_recovery_required"])

    def test_release_requires_observation_before_detaching(self):
        life = attached()
        life.carry()
        life.release()
        for changes in ({"fixture_sha256": "b"*64}, {"bilateral_contact_absent_sec": .19},
                        {"open_gap_m": .04}):
            before = life.snapshot()
            with self.assertRaises(GraspTransitionError):
                life.release_confirmed(**dict(dict(fixture_sha256=SHA,
                    bilateral_contact_absent_sec=.2, open_gap_m=.07), **changes))
            self.assertEqual(life.snapshot(), before)
        life.release_confirmed(fixture_sha256=SHA, bilateral_contact_absent_sec=.2, open_gap_m=.07)
        self.assertEqual((life.stage, life.physical_grasp, life.planning_attachment),
                         ("settling", "released", "world"))
        before = life.snapshot()
        with self.assertRaises(GraspTransitionError):
            life.release_confirmed(fixture_sha256=SHA, bilateral_contact_absent_sec=.2, open_gap_m=.07)
        self.assertEqual(life.snapshot(), before)

    def test_placement_requires_settled_target_contact(self):
        life = attached()
        life.carry()
        life.release()
        life.release_confirmed(fixture_sha256=SHA, bilateral_contact_absent_sec=.2, open_gap_m=.07)
        for changes in ({"table_contact_sec": .1}, {"position_error_m": .011}, {"speed_m_s": .021}):
            before = life.snapshot()
            with self.assertRaises(GraspTransitionError):
                life.settled(**dict(dict(table_contact_sec=.2, position_error_m=.001, speed_m_s=.001), **changes))
            self.assertEqual(life.snapshot(), before)
        life.settled(table_contact_sec=.2, position_error_m=.001, speed_m_s=.001)
        self.assertEqual(life.stage, "succeeded")

    def test_cancel_does_not_release_resume_or_confirm_stop(self):
        life = attached()
        life.carry()
        life.cancel()
        self.assertEqual(life.planning_attachment, "attached")
        self.assertNotIn("stop_observed", life.events[-1])
        self.assertFalse(life.events[-1]["automatic_release"])
        self.assertFalse(life.events[-1]["automatic_resume"])
        before = life.snapshot()
        for action in (life.carry, life.release, life.approach_complete):
            with self.assertRaises(GraspTransitionError):
                action()
            self.assertEqual(life.snapshot(), before)
        life.cancelled_observation(stop_observed=False, grasp_still_observed=False)
        self.assertEqual(life.stage, "cancelled")
        self.assertEqual(life.physical_grasp, "unconfirmed")
        self.assertEqual(life.planning_attachment, "attached")
        self.assertFalse(life.events[-1]["stop_observed"])

    def test_cancel_cannot_create_confirmation_from_an_unconfirmed_object(self):
        life = GraspLifecycle(SHA)
        life.cancel()
        life.cancelled_observation(stop_observed=True, grasp_still_observed=True)
        self.assertEqual(life.physical_grasp, "unconfirmed")
        self.assertFalse(life.events[-1]["grasp_still_observed"])

    def test_snapshot_is_detached_from_event_mutation(self):
        life = GraspLifecycle(SHA)
        before = deepcopy(life.snapshot())
        returned = life.snapshot()
        returned["events"][0]["stage"] = "succeeded"
        self.assertEqual(life.snapshot(), before)

    def test_event_bound_rejects_before_changing_observed_state(self):
        life = attached()
        life.carry()
        life.cancel()
        while len(life.events) < 32:
            life.cancelled_observation(stop_observed=True, grasp_still_observed=True)
        before = life.snapshot()
        with self.assertRaises(GraspTransitionError):
            life.cancelled_observation(stop_observed=False, grasp_still_observed=False)
        self.assertEqual(life.snapshot(), before)


class FixtureDefinitionTests(unittest.TestCase):
    def test_identity_covers_friction_object_tcp_and_model(self):
        base = FixtureSpec().identity()
        self.assertEqual(base, FixtureSpec().identity())
        self.assertNotEqual(base["manifest_sha256"], FixtureSpec(friction=.01).identity()["manifest_sha256"])
        self.assertNotEqual(base["manifest_sha256"], FixtureSpec(block_present=False).identity()["manifest_sha256"])
        self.assertEqual(base["tcp"]["site"], "grasp_tcp")
        self.assertIn("not FR5", base["carriage"])

    def test_invalid_configuration_is_rejected_before_model_creation(self):
        for changes in ({"friction": math.nan}, {"block_mass_kg": -1.}, {"block_present": 1},
                        {"timestep_sec": 0.}, {"right_finger_offset_y_m": True},
                        {"block_xyz_m": [0, 0, math.inf]}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                FixtureSpec(**changes)


if __name__ == "__main__":
    unittest.main()
