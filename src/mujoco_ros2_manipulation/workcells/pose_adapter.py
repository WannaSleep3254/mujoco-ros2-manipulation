"""Bounded MoveIt IK/FK request adapters with injected ROS message factories.

This module imports no ROS backend and sends no trajectory. IK is a candidate
joint goal; the caller must still plan, inspect the full path, and approve it.
Only selected-arm joints may change; response values for the other arm are
validated but never replace its supplied start state.
"""
from collections.abc import Mapping, Sequence
import math
import time

from .pose_goal import _finite, validate_frame_name, validate_world_pose


MAX_KINEMATICS_TIMEOUT_SEC = 5.0


class PoseAdapterError(RuntimeError):
    def __init__(self, kind, message, *, error_code=None):
        super().__init__(message)
        self.kind = kind
        self.error_code = error_code


def _names(value):
    if (not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or not value
            or any(type(name) is not str or not name.strip() for name in value)
            or len(value) != len(set(value))):
        raise ValueError("Expected unique ordered joint names")
    return tuple(value)


def _timeout(value):
    if not _finite(value) or not 0 < value <= MAX_KINEMATICS_TIMEOUT_SEC or round(value*1e9) < 1:
        raise ValueError("Kinematics timeout must be positive, at least 1 ns, and at most 5 wall seconds")
    return float(value)


class MoveItPoseAdapter:
    def __init__(self, ik_client, fk_client, wait_result, ik_request_factory,
                 fk_request_factory, joint_names, limits, monotonic=time.monotonic,
                 *, passthrough_exceptions=()):
        if (type(passthrough_exceptions) is not tuple
                or any(not isinstance(kind, type) or not issubclass(kind, Exception)
                       for kind in passthrough_exceptions)):
            raise ValueError("Expected a tuple of caller-owned control exception classes")
        # The injected waiter may observe cancellation or the shared planning
        # budget while it spins ROS. Preserve those control-flow exceptions;
        # they are not an IK/FK service failure or a transport disconnection.
        self.passthrough_exceptions = passthrough_exceptions
        self.joint_names = _names(joint_names)
        if not isinstance(limits, Mapping) or set(limits) != set(self.joint_names):
            raise ValueError("Joint limits must cover every configured joint")
        self.limits = {}
        for name in self.joint_names:
            row = limits[name]
            if (not isinstance(row, Mapping) or not {"lower", "upper"}.issubset(row)
                    or not _finite(row["lower"]) or not _finite(row["upper"])
                    or row["lower"] > row["upper"]):
                raise ValueError("Expected finite ordered position limits: " + name)
            self.limits[name] = (float(row["lower"]), float(row["upper"]))
        self.ik_client, self.fk_client = ik_client, fk_client
        self.wait_result = wait_result
        self.ik_request_factory, self.fk_request_factory = ik_request_factory, fk_request_factory
        self.monotonic = monotonic

    def _state(self, state):
        if not isinstance(state, Mapping) or set(state) != set(self.joint_names):
            raise ValueError("Kinematics state must cover every configured joint")
        result = {}
        for name in self.joint_names:
            value = state[name]
            low, high = self.limits[name]
            if not _finite(value) or not low <= value <= high:
                raise ValueError("Nonfinite or out-of-limit kinematics state: " + name)
            result[name] = float(value)
        return result

    def _rpc(self, client, request, timeout_sec, operation):
        future = None
        try:
            future = client.call_async(request)
            return self.wait_result(future, timeout_sec, "pose " + operation.upper())
        except BaseException as error:
            # A timed-out/cancelled waiter must not retain an unbounded pending
            # ROS request. These hooks are best effort and never replace the
            # primary owner, timeout, transport or process-interruption error.
            if future is not None:
                for owner, name, args in ((client, "remove_pending_request", (future,)),
                                          (future, "cancel", ())):
                    try:
                        callback = getattr(owner, name, None)
                        if callable(callback):
                            callback(*args)
                    except Exception:
                        pass
            if isinstance(error, self.passthrough_exceptions) or not isinstance(error, Exception):
                raise
            if isinstance(error, TimeoutError):
                kind = operation + "_timeout"
            elif isinstance(error, RuntimeError) and str(error).startswith("Timeout:"):
                # The repository's bounded ROS waiter reports this prefix.
                kind = operation + "_timeout"
            else:
                kind = operation + "_transport"
            raise PoseAdapterError(kind, str(error)) from error

    @staticmethod
    def _code(answer, operation):
        try:
            code = answer.error_code.val
            if type(code) is not int:
                raise ValueError("Non-integer MoveIt error code")
        except (AttributeError, TypeError, ValueError) as error:
            raise PoseAdapterError(operation + "_response_invalid", "Missing or malformed MoveIt error code") from error
        if code != 1:
            kind = operation + "_failed"
            if code == -6:
                kind = operation + "_timeout"
            elif operation == "ik" and code == -31:
                kind = "ik_no_solution"
            raise PoseAdapterError(kind, "MoveIt " + operation.upper() + " failed: " + str(code), error_code=code)
        return code

    def _seed(self, robot_state, state):
        robot_state.is_diff = False
        robot_state.joint_state.name = list(self.joint_names)
        robot_state.joint_state.position = [state[name] for name in self.joint_names]

    def solve_ik(self, group, tip_frame, tip_pose, start_rad, selected_joint_names, *,
                 timeout_sec=1.0, rpc_timeout_sec=None):
        started = self.monotonic()
        if type(group) is not str or not group.strip():
            raise ValueError("Expected an explicit IK planning group")
        validate_frame_name(tip_frame)
        pose = validate_world_pose(tip_pose)
        start = self._state(start_rad)
        selected = _names(selected_joint_names)
        if not set(selected).issubset(self.joint_names):
            raise ValueError("IK selected joints must be configured joints")
        timeout_sec = _timeout(timeout_sec)
        rpc_timeout_sec = timeout_sec if rpc_timeout_sec is None else _timeout(rpc_timeout_sec)
        request = self.ik_request_factory()
        ik = request.ik_request
        ik.group_name = group
        ik.ik_link_name = tip_frame
        ik.avoid_collisions = True
        self._seed(ik.robot_state, start)
        ik.pose_stamped.header.frame_id = "world"
        for axis, value in zip(("x", "y", "z"), pose["position_m"]):
            setattr(ik.pose_stamped.pose.position, axis, value)
        for axis, value in zip(("x", "y", "z", "w"), pose["quaternion_xyzw"]):
            setattr(ik.pose_stamped.pose.orientation, axis, value)
        ns = round(timeout_sec*1e9)
        ik.timeout.sec, ik.timeout.nanosec = divmod(ns, 1_000_000_000)
        answer = self._rpc(self.ik_client, request, rpc_timeout_sec, "ik")
        code = self._code(answer, "ik")
        try:
            solution = answer.solution.joint_state
            names = _names(solution.name)
            values = solution.position
            if (not isinstance(values, Sequence) or isinstance(values, (str, bytes))
                    or len(names) != len(values) or not set(selected).issubset(names)
                    or not set(names).issubset(self.joint_names)):
                raise ValueError("IK response joint coverage differs")
            for name, value in zip(names, values):
                low, high = self.limits[name]
                if not _finite(value) or not low <= value <= high:
                    raise ValueError("Nonfinite or out-of-limit IK solution: " + name)
            response_state = dict(zip(names, values))
            target = dict(start)
            target.update({name: float(response_state[name]) for name in selected})
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            raise PoseAdapterError("ik_response_invalid", str(error), error_code=code) from error
        return target, dict(group=group, tip_frame=tip_frame, frame_id="world", error_code=code,
                            selected_joint_names=list(selected), avoid_collisions=True,
                            timeout_sec=timeout_sec, rpc_timeout_sec=rpc_timeout_sec,
                            wall_sec=self.monotonic()-started)

    def forward_kinematics(self, tip_frame, state_rad, *, timeout_sec=1.0):
        started = self.monotonic()
        validate_frame_name(tip_frame)
        state = self._state(state_rad)
        timeout_sec = _timeout(timeout_sec)
        request = self.fk_request_factory()
        request.header.frame_id = "world"
        request.fk_link_names = [tip_frame]
        self._seed(request.robot_state, state)
        answer = self._rpc(self.fk_client, request, timeout_sec, "fk")
        code = self._code(answer, "fk")
        try:
            if list(answer.fk_link_names) != [tip_frame] or len(answer.pose_stamped) != 1:
                raise ValueError("FK response must contain exactly the requested tip")
            stamped = answer.pose_stamped[0]
            position, q = stamped.pose.position, stamped.pose.orientation
            pose = validate_world_pose(dict(frame_id=stamped.header.frame_id,
                position_m=[position.x, position.y, position.z], quaternion_xyzw=[q.x, q.y, q.z, q.w]))
        except (AttributeError, TypeError, ValueError) as error:
            raise PoseAdapterError("fk_response_invalid", str(error), error_code=code) from error
        return pose, dict(tip_frame=tip_frame, frame_id="world", error_code=code,
                          timeout_sec=timeout_sec, wall_sec=self.monotonic()-started)
