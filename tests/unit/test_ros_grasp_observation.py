import copy
from dataclasses import FrozenInstanceError
import json
import unittest

from mujoco_ros2_manipulation.grasping.ros_observation import (
    GraspObservationStore, ObservationUnavailable,
)


ARMS = ["left_j" + str(i) for i in range(1, 7)] + [
    "right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_elbow_joint",
    "right_wrist_1_joint", "right_wrist_2_joint", "right_wrist_3_joint",
]
FINGERS = ["fr5_left_finger_slide", "fr5_right_finger_slide"]
GEOMS = ["fr5_left_finger_geom", "fr5_right_finger_geom", "fr5_table_geom", "fr5_block_geom"]
RUN, MODEL, PLUGIN = "1" * 32, "2" * 64, "3" * 64


def manifest():
    names = ARMS[:6] + FINGERS + ARMS[6:]
    layout = [dict(name=name, type="slide" if name in FINGERS else "hinge",
                   qpos_address=i, qpos_width=1, velocity_address=i)
              for i, name in enumerate(names)]
    layout.append(dict(name="fr5_block_free", type="free", qpos_address=14,
                       qpos_width=7, velocity_address=14))
    return dict(artifact_kind="grasp-execution-target-v1", arm_joint_names=ARMS[:],
                finger_joint_names=FINGERS[:], scalar_joint_units={
                    **{name: "rad" for name in ARMS}, **{name: "m" for name in FINGERS}},
                qpos_state_contract=dict(nq=21, nv=20, njnt=15, nu=14, layout=layout))


def sample(sequence=1, time_sec=0, *, armed=True, phase=1):
    qpos = [i / 100 for i in range(14)] + [.1, .2, .3, 1., 0., 0., 0.]
    names = ARMS[:6] + FINGERS + ARMS[6:]
    positions = dict(zip(names, qpos))
    return dict(schema_version=1, kind="ros-grasp-observation-v1", run_id=RUN,
                model_sha256=MODEL, plugin_sha256=PLUGIN, native_version="3.12.0",
                sequence=sequence, time_sec=time_sec, stamp_ns=str(round(time_sec * 1e9)),
                stamp_semantics="rounded_native_mjData_time", phase=phase, armed=armed,
                heartbeat_fresh=armed and phase >= 0, pose_sample_sec=time_sec,
                force_evaluation_sec=max(0., time_sec - .001), force_timing_verified=False,
                force_time_semantics="nominal last mj_step evaluation; invalid when native physics is held",
                local_overwrites_or_skips=0, qpos=qpos, qvel=[0.] * 20,
                arm_positions_rad={name: positions[name] for name in ARMS},
                finger_positions_m={name: positions[name] for name in FINGERS},
                geoms=[dict(id=i, position=[.1, .2, .3], rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.])
                       for i in range(len(GEOMS))],
                tcp=dict(position=[.1, .2, .4], quaternion_wxyz=[1., 0., 0., 0.]),
                block=dict(position=qpos[14:17], quaternion_wxyz=qpos[17:21],
                           linear_velocity_m_sec=[0.] * 3, angular_velocity_rad_sec=[0.] * 3),
                contacts=[dict(geom1=2, geom2=3, geom1_name=GEOMS[2], geom2_name=GEOMS[3], normal_force_n=.8)])


def guard(**changes):
    result = dict(latched=False, reason="", phase="ready", error="", controllers={
        "left_controller": "active", "right_controller": "active", "gripper_controller": "active",
        "joint_state_broadcaster": "active",
    })
    result.update(changes)
    return result


class RosGraspObservationTests(unittest.TestCase):
    def store(self, **changes):
        arguments = dict(run_id=RUN, model_sha256=MODEL, plugin_sha256=PLUGIN,
                         manifest=manifest(), geom_names=GEOMS[:])
        arguments.update(changes)
        return GraspObservationStore(**arguments)

    def test_full_state_addresses_units_contacts_and_copy_isolation(self):
        source = sample()
        store = self.store()
        self.assertTrue(store.ingest(source, receipt_sec=10).accepted)
        result = store.snapshot(now_sec=10.1)
        self.assertEqual(len(result.qpos), 21)
        self.assertEqual(len(result.qvel), 20)
        self.assertEqual(result.arm_positions_rad[6], source["qpos"][8])
        self.assertEqual(result.finger_positions_m, (.06, .07))
        self.assertEqual(result.payload["contacts"][0]["geom1_name"], "fr5_table_geom")
        self.assertFalse(result.force_timing_verified)
        self.assertEqual(result.stamp_ns, "0")
        source["qpos"][0] = 99
        detached = result.payload
        detached["block"]["position"][0] = 99
        self.assertEqual(result.qpos[0], 0)
        self.assertEqual(result.payload["block"]["position"][0], .1)
        with self.assertRaises(FrozenInstanceError):
            result.receipt_sec = 12

    def test_live_physical_gate_needs_clock_advance_and_three_controller_guard(self):
        store = self.store()
        store.ingest(sample(), receipt_sec=10)
        store.update_guard(guard(), receipt_sec=10)
        with self.assertRaisesRegex(ObservationUnavailable, "has not advanced"):
            store.snapshot(now_sec=10.1, require_physical=True)
        store.ingest(sample(2, .05), receipt_sec=10.1)
        result = store.snapshot(now_sec=10.2, require_physical=True)
        self.assertTrue(result.clock_has_advanced)
        self.assertEqual(result.clock_progress_receipt_sec, 10.1)

    def test_duplicate_and_out_of_order_sequence_never_refresh_receipt(self):
        store = self.store()
        store.ingest(sample(2, .05), receipt_sec=10)
        for value in (sample(2, .05), sample(1, .02), sample(2, .1)):
            self.assertFalse(store.ingest(value, receipt_sec=10.4).accepted)
        self.assertEqual(store.snapshot(now_sec=10.4).receipt_sec, 10)
        with self.assertRaisesRegex(ObservationUnavailable, "stale or held"):
            store.snapshot(now_sec=10.6)

    def test_higher_sequence_at_held_time_does_not_make_physics_fresh(self):
        store = self.store()
        store.ingest(sample(1, .05), receipt_sec=10)
        store.ingest(sample(2, .1), receipt_sec=10.1)
        self.assertTrue(store.ingest(sample(3, .1), receipt_sec=10.55).accepted)
        self.assertEqual(store.snapshot(now_sec=10.55).receipt_sec, 10.55)
        with self.assertRaisesRegex(ObservationUnavailable, "stale or held"):
            store.snapshot(now_sec=10.61)

    def test_clock_reversal_cannot_be_cleared_by_later_messages(self):
        store = self.store()
        store.ingest(sample(1, .2), receipt_sec=10)
        failure = store.ingest(sample(2, .1), receipt_sec=10.1)
        self.assertFalse(failure.accepted)
        self.assertIn("new run", failure.reason)
        self.assertFalse(store.ingest(sample(3, .3), receipt_sec=10.2).accepted)
        with self.assertRaisesRegex(ObservationUnavailable, "reversed"):
            store.snapshot(now_sec=10.2)

    def test_identity_schema_or_invalid_json_blocks_previous_sample_until_valid_update(self):
        mutations = [
            lambda d: d.update(run_id="4" * 32),
            lambda d: d.update(model_sha256="4" * 64),
            lambda d: d.update(plugin_sha256="4" * 64),
            lambda d: d.update(schema_version=True),
            lambda d: d.update(kind="other"),
            lambda d: d.update(stamp_ns="1"),
            lambda d: d.update(stamp_semantics="exact_ros_time"),
            lambda d: d.update(force_timing_verified=True),
            lambda d: d.update(force_evaluation_sec=.2),
            lambda d: d.update(pose_sample_sec=.05),
        ]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                store = self.store()
                store.ingest(sample(), receipt_sec=10)
                bad = sample(2)
                mutate(bad)
                self.assertFalse(store.ingest(bad, receipt_sec=10.1).accepted)
                with self.assertRaises(ObservationUnavailable):
                    store.snapshot(now_sec=10.1)
                self.assertTrue(store.ingest(sample(3, .1), receipt_sec=10.2).accepted)
                store.snapshot(now_sec=10.2)

    def test_dimensions_names_units_and_redundant_state_consistency(self):
        mutations = [
            lambda d: d["qpos"].pop(),
            lambda d: d["qvel"].pop(),
            lambda d: d["qpos"].__setitem__(0, float("nan")),
            lambda d: d["qvel"].__setitem__(0, float("inf")),
            lambda d: d["qpos"].__setitem__(0, True),
            lambda d: d["qpos"].__setitem__(0, 2e6),
            lambda d: d["finger_positions_m"].update(extra_joint=0),
            lambda d: d["arm_positions_rad"].pop("left_j1"),
            lambda d: d["finger_positions_m"].update(fr5_left_finger_slide=60),
            lambda d: d["arm_positions_rad"].update(left_j1=1),
            lambda d: d["block"]["position"].__setitem__(0, .11),
            lambda d: d["block"]["quaternion_wxyz"].__setitem__(0, .5),
            lambda d: d["qpos"].__setitem__(17, .5),
            lambda d: d["tcp"]["quaternion_wxyz"].__setitem__(0, .5),
            lambda d: d["block"]["linear_velocity_m_sec"].pop(),
        ]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                value = sample()
                mutate(value)
                self.assertFalse(self.store().ingest(value, receipt_sec=10).accepted)

    def test_geom_id_catalogue_proper_rotation_and_contact_force_validation(self):
        mutations = [
            lambda d: d["geoms"].pop(),
            lambda d: d["geoms"][0].update(id=1),
            lambda d: d["geoms"][0]["rotation"].__setitem__(0, -1),  # reflection
            lambda d: d["geoms"][0]["rotation"].__setitem__(0, 2),
            lambda d: d["contacts"][0].update(geom1_name="wrong"),
            lambda d: d["contacts"][0].update(geom1=5),
            lambda d: d["contacts"][0].update(geom1=True),
            lambda d: d["contacts"][0].update(geom1=3, geom1_name=GEOMS[3]),
            lambda d: d["contacts"][0].update(normal_force_n=-1),
            lambda d: d.update(contacts=d["contacts"] * 257),
        ]
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                value = sample()
                mutate(value)
                self.assertFalse(self.store().ingest(value, receipt_sec=10).accepted)

    def test_zero_or_cancel_phase_or_expired_heartbeat_cannot_confirm_grasp(self):
        for phase, armed in ((0, False), (-1, True), (8, True)):
            with self.subTest(phase=phase):
                store = self.store()
                store.ingest(sample(), receipt_sec=10)
                store.ingest(sample(2, .05, phase=phase, armed=armed), receipt_sec=10.1)
                store.update_guard(guard(), receipt_sec=10.1)
                with self.assertRaisesRegex(ObservationUnavailable, "heartbeat"):
                    store.snapshot(now_sec=10.2, require_physical=True)
        invalid = sample(2, .05)
        invalid["heartbeat_fresh"] = False
        self.assertFalse(self.store().ingest(invalid, receipt_sec=10).accepted)

    def test_latched_or_missing_inactive_stale_guard_rejects_physical_use(self):
        for change in (dict(latched=True, reason="contact"), dict(phase="stopping"),
                       dict(error="service unavailable"), dict(controllers={
                           "left_controller": "inactive", "right_controller": "active", "gripper_controller": "active"})):
            with self.subTest(change=change):
                store = self.store()
                store.ingest(sample(), receipt_sec=10)
                store.ingest(sample(2, .05), receipt_sec=10.1)
                store.update_guard(guard(**change), receipt_sec=10.1)
                with self.assertRaises(ObservationUnavailable):
                    store.snapshot(now_sec=10.2, require_physical=True)
        store = self.store()
        store.ingest(sample(), receipt_sec=10)
        store.ingest(sample(2, .05), receipt_sec=10.4)
        with self.assertRaisesRegex(ObservationUnavailable, "no contact guard"):
            store.snapshot(now_sec=10.4, require_physical=True)
        store.update_guard(guard(), receipt_sec=9.8)
        with self.assertRaisesRegex(ObservationUnavailable, "guard observation is stale"):
            store.snapshot(now_sec=10.4, require_physical=True)

    def test_guard_optional_identities_must_match_and_nested_payload_is_detached(self):
        store = self.store()
        source = guard(run_id=RUN, model_sha256=MODEL)
        self.assertTrue(store.update_guard(source, receipt_sec=10).accepted)
        source["latched"] = True
        store.ingest(sample(), receipt_sec=10)
        store.ingest(sample(2, .05), receipt_sec=10.1)
        store.snapshot(now_sec=10.1, require_physical=True)
        for key, value in (("run_id", "4" * 32), ("model_sha256", "4" * 64)):
            self.assertFalse(store.update_guard(guard(**{key: value}), receipt_sec=10.2).accepted)
            with self.assertRaisesRegex(ObservationUnavailable, "does not match"):
                store.snapshot(now_sec=10.2, require_physical=True)

    def test_json_bound_duplicate_keys_scalar_types_and_clock_arguments(self):
        store = self.store()
        bad = ["{\"sequence\":1,\"sequence\":2}", b"\xff", "[]", "x" * (256 * 1024 + 1), None]
        for value in bad:
            self.assertFalse(store.ingest(value, receipt_sec=10).accepted)
        value = sample()
        for key, invalid in (("sequence", True), ("sequence", 0), ("sequence", 2**53),
                             ("stamp_ns", "00"), ("phase", 1.5), ("armed", 1),
                             ("local_overwrites_or_skips", -1)):
            altered = copy.deepcopy(value)
            altered[key] = invalid
            self.assertFalse(store.ingest(altered, receipt_sec=10).accepted)
        self.assertTrue(store.ingest(json.dumps(value).encode(), receipt_sec=10).accepted)
        self.assertFalse(store.ingest(sample(2, .05), receipt_sec=9).accepted)
        self.assertTrue(store.ingest(sample(3, .1), receipt_sec=10.1).accepted)
        with self.assertRaises(ObservationUnavailable):
            store.snapshot(now_sec=10)
        with self.assertRaises(ValueError):
            store.snapshot(now_sec=10.1, max_age_sec=float("inf"))
        with self.assertRaises(ValueError):
            store.snapshot(now_sec=10.1, require_physical=1)

    def test_constructor_rejects_ambiguous_layout_units_and_catalogue(self):
        for mutate in (
            lambda d: d["scalar_joint_units"].update(fr5_left_finger_slide="rad"),
            lambda d: d["qpos_state_contract"]["layout"][6].update(qpos_address=8),
            lambda d: d["qpos_state_contract"].update(nq=14),
            lambda d: d["arm_joint_names"].reverse(),
        ):
            value = manifest()
            mutate(value)
            with self.assertRaises(ValueError):
                self.store(manifest=value)
        for names in (GEOMS[:-1], GEOMS + [GEOMS[0]], [], ["x" * 129] + GEOMS):
            with self.assertRaises(ValueError):
                self.store(geom_names=names)
        with self.assertRaises(ValueError):
            self.store(run_id="not-a-run")


if __name__ == "__main__":
    unittest.main()
