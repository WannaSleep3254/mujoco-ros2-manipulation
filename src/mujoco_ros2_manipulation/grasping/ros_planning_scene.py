"""Explicit, confirmed MoveIt ownership transitions for the ROS grasp target.

Message factories, clients and a bounded future waiter are injected.  This is a
planning-scene adapter, not evidence of physical grasp.  The task owner must
first obtain fresh guarded physics evidence from GraspObservationStore and the
GraspLifecycle.  No cancellation path opens a gripper or detaches an object.

ApplyPlanningScene has no compare-and-swap operation.  The caller must serialize
scene writers while this adapter fetches, changes its own object/pairs, applies,
and confirms.  Unrelated ACM entries and defaults are copied from that fetch.
"""
from copy import deepcopy
import hashlib
import json
import math
import re
import threading
import time

from .ros_observation import GraspObservation


class GraspPlanningSceneError(ValueError):
    pass


def _require(condition, message):
    if not condition:
        raise GraspPlanningSceneError(message)


def _number(value, label, maximum=1e6, minimum=None):
    _require(type(value) in (int, float) and math.isfinite(value) and abs(value) <= maximum,
             label + " must be bounded and finite")
    _require(minimum is None or value >= minimum, label + " is below its minimum")
    return float(value)


def _vector(value, count, label):
    _require(type(value) in (list, tuple) and len(value) == count, label + " has the wrong width")
    return tuple(_number(item, label) for item in value)


def _quat(value):
    result = _vector(value, 4, "wxyz quaternion")
    _require(abs(sum(v*v for v in result) - 1) <= 1e-6, "a unit wxyz quaternion is required")
    return result


def _multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def _rotate(q, p):
    return _multiply(_multiply(q, (0., *p)), (q[0], -q[1], -q[2], -q[3]))[1:]


def _compose(a, b):
    ap, aq = a
    bp, bq = b
    rotated = _rotate(aq, bp)
    return tuple(x+y for x, y in zip(ap, rotated)), _multiply(aq, bq)


def _relative(parent, child):
    p, q = parent
    c, r = child
    inverse = (q[0], -q[1], -q[2], -q[3])
    return _rotate(inverse, tuple(x-y for x, y in zip(c, p))), _multiply(inverse, r)


def _pose_error(a, b):
    position = math.sqrt(sum((x-y)**2 for x, y in zip(a[0], b[0])))
    dot = min(1., abs(sum(x*y for x, y in zip(a[1], b[1]))))
    return position, 2*math.acos(dot)


def _message_pose(value):
    return (_vector([value.position.x, value.position.y, value.position.z], 3, "pose position"),
            _quat([value.orientation.w, value.orientation.x, value.orientation.y, value.orientation.z]))


def _observed_pose(value):
    _require(type(value) is dict, "native pose is missing")
    return _vector(value.get("position"), 3, "native pose position"), _quat(value.get("quaternion_wxyz"))


def _sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def _clean_headers(value):
    """Drop message transmission stamps while retaining geometry and frames."""
    if isinstance(value, dict):
        result = {key: _clean_headers(item) for key, item in value.items()}
        if "header" in result and isinstance(result["header"], dict):
            result["header"].pop("stamp", None)
        return result
    if isinstance(value, (list, tuple)):
        return [_clean_headers(item) for item in value]
    return value


class RosGraspPlanningScene:
    """Mutate only one run's free block and its two finger ACM pairs.

    The fixed table remains a robot collision link in the checked URDF.  It is
    never duplicated as a world object.  The empty TCP child is the attachment
    link: MoveIt's implicit attachment-link permission cannot then hide a palm
    collision.  touch_links contains exactly the two declared finger links.
    """

    def __init__(self, get_client, apply_client, wait_result, types, *, manifest,
                 run_id, model_sha256, monotonic=time.monotonic, rpc_timeout_sec=5., on_evidence=None,
                 fresh_observation=None, pre_mutation_guard=None):
        _require(type(run_id) is str and re.fullmatch(r"[0-9a-f]{32}", run_id) is not None,
                 "a complete native grasp run ID is required")
        _require(type(model_sha256) is str and re.fullmatch(r"[0-9a-f]{64}", model_sha256) is not None,
                 "a complete grasp model SHA256 is required")
        _require(type(manifest) is dict and manifest.get("artifact_kind") == "grasp-execution-target-v1",
                 "a checked grasp execution manifest is required")
        contract = deepcopy(manifest)
        tool, block, table = (contract.get(key, {}) for key in ("tool", "block", "table"))
        _require(tool.get("body") == "fr5_synthetic_gripper" and tool.get("tcp_frame") == "fr5_grasp_tcp"
                 and table.get("body") == "fr5_grasp_table" and block.get("body") == "fr5_grasp_block",
                 "unexpected grasp tool, block or fixed table identity")
        _require(tool.get("finger_geoms") == ["fr5_left_finger_geom", "fr5_right_finger_geom"],
                 "unexpected contact finger geometry")
        self._dimensions = tuple(2*item for item in _vector(block.get("half_size_m"), 3, "block half size"))
        _require(all(0 < value <= 1 for value in self._dimensions), "invalid block dimensions")
        self._mass = _number(block.get("mass_kg"), "block mass", maximum=10, minimum=0)
        self._urdf_sha = contract.get("outputs", {}).get("cell.urdf")
        _require(type(self._urdf_sha) is str and re.fullmatch(r"[0-9a-f]{64}", self._urdf_sha) is not None,
                 "the immutable table/tool URDF SHA256 is required")
        self.get_client, self.apply_client = get_client, apply_client
        self.wait_result, self.types, self.monotonic = wait_result, types, monotonic
        self.run_id, self.model_sha256 = run_id, model_sha256
        self.object_id = "grasp_" + run_id + "_block"
        self.table_link = table["body"]
        self.link_name = tool["tcp_frame"]
        self.touch_links = ("fr5_left_finger", "fr5_right_finger")
        self.rpc_timeout_sec = _number(rpc_timeout_sec, "RPC timeout", maximum=10, minimum=.01)
        _require(on_evidence is None or callable(on_evidence), "scene evidence callback must be callable")
        self._on_evidence = on_evidence
        _require(fresh_observation is None or callable(fresh_observation), "fresh observation supplier must be callable")
        _require(pre_mutation_guard is None or callable(pre_mutation_guard), "pre-mutation guard must be callable")
        self._fresh_observation, self._pre_mutation_guard = fresh_observation, pre_mutation_guard
        self._lock = threading.RLock()
        self._state = "uninitialized"
        self._fault = ""
        self._reference = None
        self._source = None
        self._last_scene_sha = None
        self._previous_finger_pairs = None

    def _pose_message(self, pose):
        value = self.types.Pose()
        value.position.x, value.position.y, value.position.z = pose[0]
        value.orientation.w, value.orientation.x, value.orientation.y, value.orientation.z = pose[1]
        return value

    def _collision(self, pose=None, *, attached=False, remove=False):
        value = self.types.CollisionObject()
        value.id = self.object_id
        value.header.frame_id = self.link_name if attached else "world"
        value.operation = value.REMOVE if remove else value.ADD
        if not remove:
            primitive = self.types.SolidPrimitive()
            primitive.type = primitive.BOX
            primitive.dimensions = list(self._dimensions)
            value.pose = self._pose_message(pose)
            value.primitives = [primitive]
            value.primitive_poses = [self._pose_message(((0., 0., 0.), (1., 0., 0., 0.)))]
        return value

    def _attached(self, pose=None, *, remove=False):
        value = self.types.AttachedCollisionObject()
        value.link_name = self.link_name
        value.object = self._collision(pose, attached=True, remove=remove)
        value.touch_links = list(self.touch_links)
        value.weight = self._mass
        return value

    def _call(self, client, request, label):
        if hasattr(client, "service_is_ready"):
            _require(client.service_is_ready(), label + " service is unavailable")
        return self.wait_result(client.call_async(request), self.rpc_timeout_sec, label)

    def _get(self):
        request = self.types.GetPlanningScene.Request()
        flags = self.types.PlanningSceneComponents
        request.components.components = (flags.WORLD_OBJECT_GEOMETRY | flags.ROBOT_STATE_ATTACHED_OBJECTS
                                         | flags.ALLOWED_COLLISION_MATRIX | flags.LINK_PADDING_AND_SCALING
                                         | flags.TRANSFORMS)
        scene = self._call(self.get_client, request, "get grasp planning scene").scene
        self._matrix(scene.allowed_collision_matrix)
        self._objects(scene)
        return scene

    def _objects(self, scene):
        world = list(scene.world.collision_objects)
        attached = list(scene.robot_state.attached_collision_objects)
        _require(len(world) <= 1024 and len(attached) <= 256, "planning object count exceeds its bound")
        world_ids = [obj.id for obj in world]
        attached_ids = [obj.object.id for obj in attached]
        _require(all(type(name) is str and 0 < len(name) <= 256 for name in world_ids + attached_ids),
                 "invalid planning object ID")
        _require(len(set(world_ids)) == len(world_ids) and len(set(attached_ids)) == len(attached_ids),
                 "ambiguous duplicate planning object ID")
        _require(not set(world_ids).intersection(attached_ids), "an object is both world and attached")
        return ({obj.id: obj for obj in world}, {obj.object.id: obj for obj in attached})

    def _matrix(self, matrix):
        names = list(matrix.entry_names)
        rows = [list(row.enabled) for row in matrix.entry_values]
        defaults, values = list(matrix.default_entry_names), list(matrix.default_entry_values)
        _require(len(names) <= 1024 and len(defaults) <= 1024 and len(set(names)) == len(names)
                 and len(set(defaults)) == len(defaults), "ambiguous or oversized allowed collision matrix")
        _require(all(type(name) is str and 0 < len(name) <= 256 for name in names + defaults),
                 "invalid allowed collision matrix name")
        _require(len(rows) == len(names) and all(len(row) == len(names) for row in rows)
                 and all(type(value) is bool for row in rows for value in row), "ACM must be a square boolean matrix")
        _require(all(rows[i][j] == rows[j][i] for i in range(len(rows)) for j in range(i)),
                 "ACM must be symmetric")
        _require(len(defaults) == len(values) and all(type(value) is bool for value in values),
                 "invalid ACM defaults")
        return names, rows, dict(zip(defaults, values))

    @staticmethod
    def _effective(matrix, left, right):
        names, rows, defaults = matrix
        if left in names and right in names:
            return rows[names.index(left)][names.index(right)]
        values = [defaults[name] for name in (left, right) if name in defaults]
        return all(values) if values else False

    def _scoped_matrix(self, current, finger_values):
        parsed = self._matrix(current)
        names, rows, defaults = parsed
        original_names = list(names)
        names = original_names + sorted(set((*defaults, self.object_id, self.table_link,
                                              self.link_name, *self.touch_links)) - set(original_names))
        new_rows = [[self._effective(parsed, left, right) for right in names] for left in names]
        block_index = names.index(self.object_id)
        # The new owned object is closed against every non-finger link. Its
        # false default also closes pairs with names not yet in the matrix.
        for index in range(len(names)):
            new_rows[block_index][index] = new_rows[index][block_index] = False
        for finger, allowed in zip(self.touch_links, finger_values):
            index = names.index(finger)
            new_rows[index][block_index] = new_rows[block_index][index] = allowed
        result = deepcopy(current)
        result.entry_names = names
        result.entry_values = []
        for row in new_rows:
            entry = self.types.AllowedCollisionEntry()
            entry.enabled = row
            result.entry_values.append(entry)
        result.default_entry_names = list(defaults)
        result.default_entry_values = [defaults[name] for name in defaults]
        if self.object_id in defaults:
            result.default_entry_values[result.default_entry_names.index(self.object_id)] = False
        else:
            result.default_entry_names.append(self.object_id)
            result.default_entry_values.append(False)
        return result

    def _matrix_identity(self, matrix):
        names, rows, defaults = self._matrix(matrix)
        # MoveIt's message constructor stores only off-diagonal exceptions to
        # pair defaults. Names with entirely default-valued rows can therefore
        # disappear on getMessage(). Compare the effective permissions, not
        # that transport representation. False exceptions to true defaults
        # remain essential and are retained here.
        parsed = (names, rows, defaults)
        ordered = sorted(set(names) | set(defaults))
        exceptions = []
        for index, left in enumerate(ordered):
            for right in ordered[index+1:]:
                values = [defaults[name] for name in (left, right) if name in defaults]
                fallback = all(values) if values else False
                allowed = self._effective(parsed, left, right)
                if allowed != fallback:
                    exceptions.append([left, right, allowed])
        return dict(pairs=exceptions, defaults=dict(sorted(defaults.items())))

    def _scene_identity(self, scene):
        world, attached = self._objects(scene)
        convert = self.types.message_to_ordereddict
        value = dict(world=[_clean_headers(convert(world[name])) for name in sorted(world)],
                     attached=[_clean_headers(convert(attached[name])) for name in sorted(attached)],
                     acm=self._matrix_identity(scene.allowed_collision_matrix),
                     link_padding=[_clean_headers(convert(item)) for item in getattr(scene, "link_padding", ())],
                     link_scale=[_clean_headers(convert(item)) for item in getattr(scene, "link_scale", ())],
                     transforms=[_clean_headers(convert(item)) for item in getattr(scene, "fixed_frame_transforms", ())])
        return _sha(value)

    def _observation(self, observation, now_sec, *, physical=False):
        _require(isinstance(observation, GraspObservation), "a checked immutable native observation is required")
        now = self.monotonic() if now_sec is None else _number(now_sec, "observation now", maximum=1e12, minimum=0)
        _require(0 <= now-observation.receipt_sec <= .5
                 and 0 <= now-observation.clock_progress_receipt_sec <= .5, "native grasp observation is stale or held")
        value = observation.payload
        _require(value.get("run_id") == self.run_id and value.get("model_sha256") == self.model_sha256,
                 "native observation does not belong to this scene owner")
        if physical:
            _require(observation.clock_has_advanced and value.get("armed") is True
                     and value.get("heartbeat_fresh") is True and value.get("phase") in range(8),
                     "fresh guarded physical evidence is required")
        return value

    def _refreshed(self, observation, now_sec, *, physical=False):
        if self._fresh_observation is not None:
            observation = self._fresh_observation()
        return self._observation(observation, now_sec, physical=physical)

    def _assert_object(self, obj, expected_pose, *, attached=False):
        _require(obj.id == self.object_id and obj.header.frame_id == (self.link_name if attached else "world"),
                 "confirmed object ID/frame differs")
        _require(obj.operation == obj.ADD and len(obj.primitives) == 1 and len(obj.primitive_poses) == 1,
                 "confirmed object must contain exactly one box")
        primitive = obj.primitives[0]
        dimensions = _vector(list(primitive.dimensions), 3, "confirmed box dimensions")
        _require(primitive.type == primitive.BOX and all(abs(a-b) <= 1e-9 for a, b in zip(dimensions, self._dimensions)),
                 "confirmed object geometry differs")
        _require(not obj.meshes and not obj.mesh_poses and not obj.planes and not obj.plane_poses
                 and not obj.subframe_names and not obj.subframe_poses, "unexpected additional collision geometry")
        actual = _compose(_message_pose(obj.pose), _message_pose(obj.primitive_poses[0]))
        position_error, angle_error = _pose_error(actual, expected_pose)
        _require(position_error <= 1e-7 and angle_error <= 1e-6, "confirmed object source pose differs")

    def _assert_owned(self, scene, state, reference):
        world, attached = self._objects(scene)
        if state in ("world", "contact_allowed", "released"):
            _require(self.object_id in world and self.object_id not in attached, "owned free block is not exclusively in the world")
            self._assert_object(world[self.object_id], reference)
        elif state == "attached":
            _require(self.object_id not in world and self.object_id in attached, "owned block attachment was not confirmed")
            item = attached[self.object_id]
            _require(item.link_name == self.link_name and set(item.touch_links) in (
                set(self.touch_links), set((*self.touch_links, self.link_name)))
                and len(item.touch_links) == len(set(item.touch_links)), "attached link or scoped touch links differ")
            self._assert_object(item.object, reference, attached=True)
        matrix = self._matrix(scene.allowed_collision_matrix)
        allowed = state in ("contact_allowed", "attached")
        for finger in self.touch_links:
            _require(self._effective(matrix, self.object_id, finger) is allowed,
                     "owned finger ACM permission differs")
        for name in set(matrix[0]) | set(matrix[2]):
            if name not in self.touch_links:
                _require(not self._effective(matrix, self.object_id, name), "owned block has an unexpected collision bypass")
        _require(matrix[2].get(self.object_id) is False, "owned block must have a closed ACM default")

    def _ready(self, *states):
        _require(not self._fault, "planning scene requires explicit recovery: " + self._fault)
        _require(self._state in states, "invalid planning scene transition from " + self._state)

    def _apply(self, before, *, world=(), attached=(), matrix, state, reference, source):
        request = self.types.ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = list(world)
        request.scene.robot_state.attached_collision_objects = list(attached)
        request.scene.allowed_collision_matrix = matrix
        submitted = False
        try:
            if hasattr(self.apply_client, "service_is_ready"):
                _require(self.apply_client.service_is_ready(), "apply grasp planning scene service is unavailable")
            if self._pre_mutation_guard is not None:
                self._pre_mutation_guard()
            submitted = True
            answer = self.wait_result(self.apply_client.call_async(request), self.rpc_timeout_sec, "apply grasp planning scene")
            _require(answer.success is True, "MoveIt rejected the grasp scene update")
            confirmed = self._get()
            self._assert_owned(confirmed, state, reference)
            # Confirm the whole copied ACM, not just the newly allowed pairs.
            if self._matrix_identity(confirmed.allowed_collision_matrix) != self._matrix_identity(matrix):
                if self._on_evidence is not None:
                    convert = self.types.message_to_ordereddict
                    self._on_evidence("acm_confirmation_mismatch", dict(
                        schema_version=1, run_id=self.run_id, model_sha256=self.model_sha256,
                        before=convert(before.allowed_collision_matrix), requested=convert(matrix),
                        confirmed=convert(confirmed.allowed_collision_matrix)))
                raise GraspPlanningSceneError("confirmed ACM differs from the scoped update")
            old_world, old_attached = self._objects(before)
            new_world, new_attached = self._objects(confirmed)
            convert = self.types.message_to_ordereddict
            for old, new in ((old_world, new_world), (old_attached, new_attached)):
                a = {name: _clean_headers(convert(value)) for name, value in old.items() if name != self.object_id}
                b = {name: _clean_headers(convert(value)) for name, value in new.items() if name != self.object_id}
                _require(a == b, "an unrelated planning object changed during the grasp update")
            self._last_scene_sha = self._scene_identity(confirmed)
            self._state, self._reference, self._source = state, reference, source
            return self.snapshot()
        except Exception as exc:
            if not submitted:
                raise  # No mutation was sent; keep the last confirmed state.
            # The request may have been applied despite a transport/confirmation
            # failure. Never report the last known attachment as current fact.
            self._fault = str(exc) or type(exc).__name__
            raise GraspPlanningSceneError("planning scene mutation is unconfirmed: " + self._fault) from exc

    def initialize(self, observation, *, now_sec=None):
        with self._lock:
            self._ready("uninitialized")
            value = self._observation(observation, now_sec)
            before = self._get()
            world, attached = self._objects(before)
            _require(self.object_id not in world and self.object_id not in attached,
                     "owned object already exists; explicit recovery is required")
            value = self._refreshed(observation, now_sec)
            reference = _observed_pose(value["block"])
            matrix = self._scoped_matrix(before.allowed_collision_matrix, (False, False))
            return self._apply(before, world=(self._collision(reference),), matrix=matrix,
                               state="world", reference=reference, source=self._source_identity(value))

    def allow_finger_contact(self, observation, *, now_sec=None):
        with self._lock:
            self._ready("world")
            value = self._observation(observation, now_sec, physical=True)
            _require(value["phase"] in (2, 3), "finger contact is allowed only during closing/extraction")
            before = self._get()
            self._assert_owned(before, self._state, self._reference)
            value = self._refreshed(observation, now_sec, physical=True)
            _require(value["phase"] in (2, 3), "finger contact phase changed before mutation")
            parsed = self._matrix(before.allowed_collision_matrix)
            previous = tuple(self._effective(parsed, self.object_id, finger) for finger in self.touch_links)
            matrix = self._scoped_matrix(before.allowed_collision_matrix, (True, True))
            result = self._apply(before, matrix=matrix, state="contact_allowed",
                                 reference=self._reference, source=self._source)
            self._previous_finger_pairs = previous
            return result

    def attach(self, observation, *, physical_confirmed, now_sec=None):
        with self._lock:
            self._ready("contact_allowed")
            _require(physical_confirmed is True, "logical attachment requires separately confirmed physical grasp")
            value = self._observation(observation, now_sec, physical=True)
            _require(value["phase"] in (3, 4), "attachment requires the extraction/holding phase")
            before = self._get()
            self._assert_owned(before, self._state, self._reference)
            value = self._refreshed(observation, now_sec, physical=True)
            _require(value["phase"] in (3, 4), "physical attachment phase changed before mutation")
            reference = _relative(_observed_pose(value["tcp"]), _observed_pose(value["block"]))
            # MoveIt processes robot_state before world updates. Attached ADD
            # atomically removes the same world ID itself; an additional world
            # REMOVE would then fail after the attachment had already happened.
            # Full fresh geometry prevents reuse of the old world source pose.
            return self._apply(before, attached=(self._attached(reference),),
                               matrix=self._scoped_matrix(before.allowed_collision_matrix, (True, True)),
                               state="attached", reference=reference, source=self._source_identity(value))

    def release(self, observation, *, physical_released, now_sec=None):
        with self._lock:
            self._ready("attached")
            _require(physical_released is True, "detachment requires separately confirmed physical release")
            value = self._observation(observation, now_sec, physical=True)
            _require(value["phase"] in (6, 7), "detachment requires the opening/completed phase")
            before = self._get()
            self._assert_owned(before, self._state, self._reference)
            value = self._refreshed(observation, now_sec, physical=True)
            _require(value["phase"] in (6, 7), "physical release phase changed before mutation")
            _require(self._previous_finger_pairs == (False, False), "scoped finger permissions cannot be restored")
            reference = _observed_pose(value["block"])
            return self._apply(before, world=(self._collision(reference),), attached=(self._attached(remove=True),),
                               matrix=self._scoped_matrix(before.allowed_collision_matrix, self._previous_finger_pairs),
                               state="released", reference=reference, source=self._source_identity(value))

    @staticmethod
    def _source_identity(value):
        return dict(sequence=value["sequence"], stamp_ns=value["stamp_ns"],
                    stamp_semantics=value["stamp_semantics"], time_sec=value["time_sec"])

    def snapshot(self):
        with self._lock:
            return dict(schema_version=1, run_id=self.run_id, model_sha256=self.model_sha256,
                        owned_object_id=self.object_id, table_link=self.table_link, table_source="checked_fixed_urdf",
                        table_tool_urdf_sha256=self._urdf_sha, attachment_link=self.link_name,
                        touch_links=list(self.touch_links), last_confirmed_state=self._state,
                        planning_attachment=("unknown" if self._fault else
                            "uninitialized" if self._state == "uninitialized" else
                            "attached" if self._state == "attached" else "world"),
                        mutation_uncertain=bool(self._fault), error=self._fault,
                        source=deepcopy(self._source), reference_pose=deepcopy(self._reference),
                        planning_scene_sha256=self._last_scene_sha,
                        physical_authority="caller-confirmed physics; scene mutation does not establish grasp")

    def review_identity(self):
        with self._lock:
            self._ready("world", "contact_allowed", "attached", "released")
            scene = self._get()
            self._assert_owned(scene, self._state, self._reference)
            return dict(run_id=self.run_id, model_sha256=self.model_sha256,
                        table_tool_urdf_sha256=self._urdf_sha, planning_scene_sha256=self._scene_identity(scene),
                        owned_object_id=self.object_id, planning_attachment=self._state)

    def block_drift(self, observation, *, now_sec=None, position_tolerance_m=.005, angle_tolerance_rad=.15):
        with self._lock:
            self._ready("world", "contact_allowed", "attached", "released")
            value = self._observation(observation, now_sec)
            position_tolerance = _number(position_tolerance_m, "position tolerance", maximum=.02, minimum=0)
            angle_tolerance = _number(angle_tolerance_rad, "angle tolerance", maximum=.5, minimum=0)
            pose = _observed_pose(value["block"])
            if self._state == "attached":
                pose = _relative(_observed_pose(value["tcp"]), pose)
            position, angle = _pose_error(pose, self._reference)
            return dict(position_error_m=position, angle_error_rad=angle,
                        within_tolerance=position <= position_tolerance and angle <= angle_tolerance,
                        reference_frame=self.link_name if self._state == "attached" else "world")
