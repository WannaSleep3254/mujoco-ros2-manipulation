from copy import deepcopy
import math
from types import SimpleNamespace as NS
import unittest

from mujoco_ros2_manipulation.grasping.ros_observation import GraspObservationStore
from mujoco_ros2_manipulation.grasping.ros_planning_scene import RosGraspPlanningScene, GraspPlanningSceneError
from tests.unit.test_ros_grasp_observation import manifest as state_manifest, sample, RUN, MODEL, PLUGIN, GEOMS


class Pose(NS):
    def __init__(self):
        super().__init__(position=NS(x=0., y=0., z=0.), orientation=NS(w=0., x=0., y=0., z=0.))


class CollisionObject(NS):
    ADD, REMOVE, APPEND, MOVE = range(4)

    def __init__(self):
        super().__init__(id="", header=NS(frame_id="", stamp=NS(sec=0, nanosec=0)), pose=Pose(),
                         operation=0, primitives=[], primitive_poses=[], meshes=[], mesh_poses=[],
                         planes=[], plane_poses=[], subframe_names=[], subframe_poses=[])


class AttachedCollisionObject(NS):
    def __init__(self):
        super().__init__(link_name="", object=CollisionObject(), touch_links=[], weight=0.)


class SolidPrimitive(NS):
    BOX = 1

    def __init__(self):
        super().__init__(type=0, dimensions=[])


class AllowedCollisionEntry(NS):
    def __init__(self):
        super().__init__(enabled=[])


def matrix():
    return NS(entry_names=["arm_a", "arm_b"],
              entry_values=[NS(enabled=[False, True]), NS(enabled=[True, False])],
              default_entry_names=["other_sensor"], default_entry_values=[True])


def scene():
    return NS(world=NS(collision_objects=[]), robot_state=NS(attached_collision_objects=[], is_diff=False),
              allowed_collision_matrix=matrix(), link_padding=[], link_scale=[], fixed_frame_transforms=[], is_diff=False)


def convert(value):
    if isinstance(value, NS):
        return {key: convert(item) for key, item in vars(value).items()}
    if isinstance(value, list):
        return [convert(item) for item in value]
    return value


TYPES = NS(GetPlanningScene=NS(Request=lambda: NS(components=NS(components=0))),
           ApplyPlanningScene=NS(Request=lambda: NS(scene=scene())),
           PlanningSceneComponents=NS(WORLD_OBJECT_GEOMETRY=16, ROBOT_STATE_ATTACHED_OBJECTS=4,
                                     ALLOWED_COLLISION_MATRIX=128, LINK_PADDING_AND_SCALING=256, TRANSFORMS=64),
           CollisionObject=CollisionObject, AttachedCollisionObject=AttachedCollisionObject,
           SolidPrimitive=SolidPrimitive, Pose=Pose, AllowedCollisionEntry=AllowedCollisionEntry,
           message_to_ordereddict=convert)


def manifest():
    value = state_manifest()
    value.update(tool=dict(body="fr5_synthetic_gripper", tcp_frame="fr5_grasp_tcp",
                           finger_geoms=["fr5_left_finger_geom", "fr5_right_finger_geom"]),
                 block=dict(body="fr5_grasp_block", half_size_m=[.02, .015, .025], mass_kg=.08),
                 table=dict(body="fr5_grasp_table"), outputs={"cell.urdf": "a" * 64})
    return value


def observation(phase=1, *, block_position=None, tcp_position=None, tcp_quaternion=None, now=10.1):
    value = sample(2, .05, phase=phase)
    if block_position is not None:
        value["block"]["position"] = list(block_position)
        value["qpos"][14:17] = list(block_position)
    if tcp_position is not None:
        value["tcp"]["position"] = list(tcp_position)
    if tcp_quaternion is not None:
        value["tcp"]["quaternion_wxyz"] = list(tcp_quaternion)
    store = GraspObservationStore(run_id=RUN, model_sha256=MODEL, plugin_sha256=PLUGIN,
                                 manifest=manifest(), geom_names=GEOMS)
    assert store.ingest(sample(), receipt_sec=now-.1).accepted
    assert store.ingest(value, receipt_sec=now).accepted
    return store.snapshot(now_sec=now)


class SceneServer:
    def __init__(self):
        self.scene = scene()
        self.get_requests = []
        self.apply_requests = []
        self.waits = []
        self.apply_success = True
        self.corrupt_after_apply = None
        self._pending_confirmation = False
        self.get_client = NS(call_async=self.get, service_is_ready=lambda: True)
        self.apply_client = NS(call_async=self.apply, service_is_ready=lambda: True)

    def get(self, request):
        self.get_requests.append(deepcopy(request))
        result = deepcopy(self.scene)
        if self._pending_confirmation and self.corrupt_after_apply:
            self.corrupt_after_apply(result)
        self._pending_confirmation = False
        return NS(result=NS(scene=result))

    def apply(self, request):
        self.apply_requests.append(deepcopy(request))
        self._pending_confirmation = True
        if self.apply_success:
            for collection, additions, key in (
                (self.scene.world.collision_objects, request.scene.world.collision_objects, lambda obj: obj.id),
                (self.scene.robot_state.attached_collision_objects,
                 request.scene.robot_state.attached_collision_objects, lambda obj: obj.object.id),
            ):
                for obj in additions:
                    identifier = key(obj)
                    collection[:] = [current for current in collection if key(current) != identifier]
                    collision = obj.object if hasattr(obj, "object") else obj
                    if collision.operation != collision.REMOVE:
                        collection.append(deepcopy(obj))
            self.scene.allowed_collision_matrix = deepcopy(request.scene.allowed_collision_matrix)
        return NS(result=NS(success=self.apply_success))

    def wait(self, future, timeout, label):
        self.waits.append((timeout, label))
        return future.result

    def adapter(self, **overrides):
        arguments = dict(manifest=manifest(), run_id=RUN, model_sha256=MODEL, monotonic=lambda: 10.1)
        arguments.update(overrides)
        return RosGraspPlanningScene(self.get_client, self.apply_client, self.wait, TYPES, **arguments)


class RosGraspPlanningSceneTests(unittest.TestCase):
    def initialized(self):
        server = SceneServer()
        adapter = server.adapter()
        adapter.initialize(observation())
        return server, adapter

    def allowed(self):
        server, adapter = self.initialized()
        adapter.allow_finger_contact(observation(2))
        return server, adapter

    def attached(self):
        server, adapter = self.allowed()
        adapter.attach(observation(3), physical_confirmed=True)
        return server, adapter

    def test_constructor_is_read_only_and_initialize_adds_only_owned_free_block(self):
        server = SceneServer()
        adapter = server.adapter()
        self.assertEqual(server.get_requests, [])
        self.assertEqual(server.apply_requests, [])
        result = adapter.initialize(observation())
        self.assertEqual(result["last_confirmed_state"], "world")
        self.assertEqual(result["table_source"], "checked_fixed_urdf")
        self.assertEqual(result["table_tool_urdf_sha256"], "a" * 64)
        request = server.apply_requests[0]
        self.assertTrue(request.scene.is_diff)
        self.assertTrue(request.scene.robot_state.is_diff)
        self.assertEqual(len(request.scene.world.collision_objects), 1)
        block = request.scene.world.collision_objects[0]
        self.assertEqual(block.id, adapter.object_id)
        self.assertEqual(block.primitives[0].dimensions, [.04, .03, .05])
        self.assertEqual(block.header.frame_id, "world")
        self.assertEqual(request.scene.robot_state.attached_collision_objects, [])
        self.assertEqual(len(server.get_requests), 2)
        self.assertTrue(all(timeout == 5 for timeout, _ in server.waits))
        self.assertEqual(server.get_requests[0].components.components, 16 | 4 | 128 | 256 | 64)

    def test_scoped_contact_and_release_preserve_unrelated_acm_pairs_and_defaults(self):
        server, adapter = self.allowed()
        def permission(left, right):
            value = server.scene.allowed_collision_matrix
            return value.entry_values[value.entry_names.index(left)].enabled[value.entry_names.index(right)]
        self.assertTrue(permission("arm_a", "arm_b"))
        self.assertTrue(permission(adapter.object_id, "fr5_left_finger"))
        self.assertTrue(permission(adapter.object_id, "fr5_right_finger"))
        self.assertFalse(permission(adapter.object_id, adapter.table_link))
        self.assertFalse(permission(adapter.object_id, "other_sensor"))
        self.assertFalse(permission(adapter.object_id, adapter.link_name))
        # Existing defaults are not changed to achieve the scoped exception.
        defaults = dict(zip(server.scene.allowed_collision_matrix.default_entry_names,
                            server.scene.allowed_collision_matrix.default_entry_values))
        self.assertTrue(defaults["other_sensor"])
        self.assertFalse(defaults[adapter.object_id])
        adapter.attach(observation(3), physical_confirmed=True)
        adapter.release(observation(6, block_position=[.3, .4, .5]), physical_released=True)
        self.assertFalse(permission(adapter.object_id, "fr5_left_finger"))
        self.assertFalse(permission(adapter.object_id, "fr5_right_finger"))
        self.assertTrue(permission("arm_a", "arm_b"))
        self.assertEqual(len(server.scene.world.collision_objects), 1)
        self.assertEqual(server.scene.robot_state.attached_collision_objects, [])
        self.assertAlmostEqual(server.scene.world.collision_objects[0].pose.position.x, .3)
        self.assertEqual(adapter.snapshot()["planning_attachment"], "world")

    def test_attach_uses_observed_tcp_relative_transform_and_exact_finger_touch_links(self):
        server, adapter = self.allowed()
        q = [math.sqrt(.5), 0., 0., math.sqrt(.5)]
        native = observation(3, tcp_position=[1., 2., 3.], tcp_quaternion=q, block_position=[1., 2.1, 3.])
        result = adapter.attach(native, physical_confirmed=True)
        request = server.apply_requests[-1]
        self.assertEqual(request.scene.world.collision_objects[0].operation, CollisionObject.REMOVE)
        attached = request.scene.robot_state.attached_collision_objects[0]
        self.assertEqual(attached.link_name, "fr5_grasp_tcp")
        self.assertEqual(attached.object.header.frame_id, "fr5_grasp_tcp")
        self.assertEqual(attached.touch_links, ["fr5_left_finger", "fr5_right_finger"])
        self.assertNotIn("fr5_synthetic_gripper", attached.touch_links)
        self.assertAlmostEqual(attached.object.pose.position.x, .1)
        self.assertAlmostEqual(attached.object.pose.position.y, 0.)
        self.assertAlmostEqual(attached.object.pose.orientation.z, -math.sqrt(.5))
        self.assertEqual(result["planning_attachment"], "attached")
        self.assertEqual(server.scene.world.collision_objects, [])

    def test_physical_flags_and_phase_are_required_before_any_attachment_or_release_rpc(self):
        server, adapter = self.allowed()
        count = len(server.apply_requests)
        for flag in (False, 1, "confirmed"):
            with self.assertRaisesRegex(GraspPlanningSceneError, "separately confirmed"):
                adapter.attach(observation(3), physical_confirmed=flag)
        with self.assertRaisesRegex(GraspPlanningSceneError, "extraction/holding"):
            adapter.attach(observation(2), physical_confirmed=True)
        self.assertEqual(len(server.apply_requests), count)
        adapter.attach(observation(3), physical_confirmed=True)
        count = len(server.apply_requests)
        with self.assertRaisesRegex(GraspPlanningSceneError, "separately confirmed"):
            adapter.release(observation(6), physical_released=False)
        with self.assertRaisesRegex(GraspPlanningSceneError, "opening/completed"):
            adapter.release(observation(4), physical_released=True)
        self.assertEqual(len(server.apply_requests), count)

    def test_wrong_observation_identity_stale_clock_or_non_observation_sends_zero_updates(self):
        server = SceneServer()
        adapter = server.adapter(model_sha256="b" * 64)
        with self.assertRaisesRegex(GraspPlanningSceneError, "does not belong"):
            adapter.initialize(observation())
        self.assertEqual(server.apply_requests, [])
        adapter = server.adapter()
        with self.assertRaisesRegex(GraspPlanningSceneError, "stale or held"):
            adapter.initialize(observation(), now_sec=11)
        with self.assertRaisesRegex(GraspPlanningSceneError, "immutable"):
            adapter.initialize(sample())
        self.assertEqual(server.apply_requests, [])

    def test_confirmation_failure_marks_state_unknown_and_does_not_auto_recover_or_detach(self):
        server, adapter = self.allowed()
        server.corrupt_after_apply = lambda scene: setattr(
            scene.robot_state.attached_collision_objects[0], "link_name", "left_wrist3_link")
        with self.assertRaisesRegex(GraspPlanningSceneError, "unconfirmed"):
            adapter.attach(observation(3), physical_confirmed=True)
        result = adapter.snapshot()
        self.assertTrue(result["mutation_uncertain"])
        self.assertEqual(result["planning_attachment"], "unknown")
        self.assertEqual(result["last_confirmed_state"], "contact_allowed")
        count = len(server.apply_requests)
        with self.assertRaisesRegex(GraspPlanningSceneError, "explicit recovery"):
            adapter.release(observation(6), physical_released=True)
        self.assertEqual(len(server.apply_requests), count)
        self.assertEqual(len(server.scene.robot_state.attached_collision_objects), 1)

    def test_box_pose_confirmation_accepts_equivalent_shape_pose_but_rejects_changed_geometry(self):
        server = SceneServer()
        adapter = server.adapter()
        def equivalent(scene):
            obj = scene.world.collision_objects[0]
            obj.primitive_poses[0] = deepcopy(obj.pose)
            obj.pose.position.x = obj.pose.position.y = obj.pose.position.z = 0.
            obj.pose.orientation.w = 1.
            obj.pose.orientation.x = obj.pose.orientation.y = obj.pose.orientation.z = 0.
        server.corrupt_after_apply = equivalent
        adapter.initialize(observation())
        server = SceneServer()
        adapter = server.adapter()
        server.corrupt_after_apply = lambda s: s.world.collision_objects[0].primitives[0].dimensions.__setitem__(0, .4)
        with self.assertRaisesRegex(GraspPlanningSceneError, "geometry differs"):
            adapter.initialize(observation())

    def test_unrelated_world_object_preserved_and_change_during_rpc_is_refused(self):
        server = SceneServer()
        other = CollisionObject()
        other.id = "existing_fixture"
        server.scene.world.collision_objects.append(other)
        adapter = server.adapter()
        adapter.initialize(observation())
        self.assertEqual(convert(server.scene.world.collision_objects[0]), convert(other))
        server.corrupt_after_apply = lambda scene: setattr(scene.world.collision_objects[0], "id", "changed_fixture")
        with self.assertRaisesRegex(GraspPlanningSceneError, "unrelated planning object changed"):
            adapter.allow_finger_contact(observation(2))

    def test_external_owned_geometry_or_acm_bypass_changes_are_not_silently_adopted(self):
        server, adapter = self.initialized()
        acm = server.scene.allowed_collision_matrix
        i, j = acm.entry_names.index(adapter.object_id), acm.entry_names.index("arm_a")
        acm.entry_values[i].enabled[j] = acm.entry_values[j].enabled[i] = True
        count = len(server.apply_requests)
        with self.assertRaisesRegex(GraspPlanningSceneError, "unexpected collision bypass"):
            adapter.allow_finger_contact(observation(2))
        self.assertEqual(len(server.apply_requests), count)

    def test_malformed_acm_fails_before_apply_and_prior_owner_is_not_overwritten(self):
        for mutation in (lambda m: m.entry_values[0].enabled.pop(),
                         lambda m: m.entry_values[0].enabled.__setitem__(1, False),
                         lambda m: m.entry_names.__setitem__(1, "arm_a"),
                         lambda m: m.default_entry_values.__setitem__(0, 1)):
            server = SceneServer()
            adapter = server.adapter()
            mutation(server.scene.allowed_collision_matrix)
            with self.assertRaises(GraspPlanningSceneError):
                adapter.initialize(observation())
            self.assertEqual(server.apply_requests, [])
        server = SceneServer()
        adapter = server.adapter()
        old = CollisionObject()
        old.id = adapter.object_id
        server.scene.world.collision_objects.append(old)
        with self.assertRaisesRegex(GraspPlanningSceneError, "already exists"):
            adapter.initialize(observation())
        self.assertEqual(server.apply_requests, [])

    def test_transport_failure_or_false_apply_never_claims_confirmed_attachment(self):
        server, adapter = self.allowed()
        server.apply_success = False
        with self.assertRaisesRegex(GraspPlanningSceneError, "rejected"):
            adapter.attach(observation(3), physical_confirmed=True)
        self.assertEqual(adapter.snapshot()["planning_attachment"], "unknown")
        server, adapter = self.allowed()
        server.apply_client.call_async = lambda _: (_ for _ in ()).throw(TimeoutError("RPC timed out"))
        with self.assertRaisesRegex(GraspPlanningSceneError, "RPC timed out"):
            adapter.attach(observation(3), physical_confirmed=True)
        self.assertTrue(adapter.snapshot()["mutation_uncertain"])

    def test_review_identity_covers_unrelated_scene_acm_and_checked_urdf(self):
        server, adapter = self.initialized()
        before = adapter.review_identity()
        self.assertEqual(before["table_tool_urdf_sha256"], "a" * 64)
        acm = server.scene.allowed_collision_matrix
        i, j = acm.entry_names.index("arm_a"), acm.entry_names.index("arm_b")
        acm.entry_values[i].enabled[j] = acm.entry_values[j].enabled[i] = False
        self.assertNotEqual(before["planning_scene_sha256"], adapter.review_identity()["planning_scene_sha256"])
        # Header delivery stamps do not invalidate a geometrically identical scene.
        stable = adapter.review_identity()
        server.scene.world.collision_objects[0].header.stamp.sec = 99
        self.assertEqual(stable, adapter.review_identity())

    def test_drift_is_world_before_attachment_and_tcp_relative_while_carried(self):
        server, adapter = self.initialized()
        self.assertTrue(adapter.block_drift(observation())["within_tolerance"])
        self.assertFalse(adapter.block_drift(observation(block_position=[.11, .2, .3]))["within_tolerance"])
        adapter.allow_finger_contact(observation(2))
        adapter.attach(observation(3), physical_confirmed=True)
        result = adapter.block_drift(observation(4, tcp_position=[.3, .4, .6], block_position=[.3, .4, .5]))
        self.assertTrue(result["within_tolerance"])
        self.assertEqual(result["reference_frame"], "fr5_grasp_tcp")
        slipped = adapter.block_drift(observation(4, tcp_position=[.3, .4, .6], block_position=[.3, .4, .48]))
        self.assertFalse(slipped["within_tolerance"])


if __name__ == "__main__":
    unittest.main()
