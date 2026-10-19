"""Shared-world arm isolation, collision planning, coordinated timing and cancellation."""

import argparse
from copy import deepcopy
import json
import math
import random
from types import SimpleNamespace
import rclpy
from rclpy.action import ActionClient
from rclpy.utilities import get_rmw_implementation_identifier
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.ros_checks import Verification
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.workcells.config import identifier, load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.trajectory import sampled_states, partitions


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class WorkcellVerification(Verification):
    def __init__(self, cell, progress=None):
        settings = cell.instances[0].profile.data["verification"]
        proxy = SimpleNamespace(
            id=cell.id,
            joints=cell.joints,
            data={"verification": settings},
            ros=cell.ros,
        )
        self.cell = cell
        self.watch = None
        self.dispatched = 0
        self.validity_samples = 0
        self.latest_velocities = {}
        self.commanded = dict(zip(cell.joints, cell.home))
        super().__init__(proxy, progress=progress)
        self.arm_clients = {
            i.id: ActionClient(
                self, FollowJointTrajectory, f"/{i.controller}/follow_joint_trajectory"
            )
            for i in cell.instances
        }
        self.handles = []
        self.acceptance_margins = []
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")

    def joints(self, message):
        super().joints(message)
        self.latest_velocities = dict(zip(message.name, message.velocity))
        if self.watch and self.positions:
            values = dict(zip(self.joint_names, self.positions))
            for inst in self.cell.instances:
                if (
                    inst.id not in self.watch["observed"]
                    and max(
                        abs(values[name] - self.watch["initial"][name])
                        for name in inst.joints
                    )
                    > 0.001
                ):
                    self.watch["observed"][inst.id] = seconds(message.header.stamp)

    def state(self):
        return dict(zip(self.joint_names, self.positions))

    def validity_of(self, state):
        request = GetStateValidity.Request()
        request.group_name = "both_arms"
        request.robot_state.joint_state.name = list(self.joint_names)
        request.robot_state.joint_state.position = [
            float(state[name]) for name in self.joint_names
        ]
        self.validity_samples += 1
        return self.result(
            self.validity.call_async(request), 5, "shared collision check"
        )

    def plan(self, group, target):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = group
        motion.pipeline_id = "ompl"
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.1
        motion.max_acceleration_scaling_factor = 0.1
        motion.start_state.is_diff = True
        motion.start_state.joint_state.name = list(self.joint_names)
        motion.start_state.joint_state.position = list(self.positions)
        constraint = Constraints()
        for name, value in target.items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = joint.tolerance_below = 0.003
            joint.weight = 1.0
            constraint.joint_constraints.append(joint)
        motion.goal_constraints = [constraint]
        return self.result(
            self.planner.call_async(request), 25, "shared motion plan"
        ).motion_plan_response

    def anchored(self, trajectory):
        trajectory = deepcopy(trajectory)
        first = trajectory.points[0]
        first.positions = [self.commanded[name] for name in trajectory.joint_names]
        first.velocities = [0.0] * len(trajectory.joint_names)
        first.accelerations = [0.0] * len(trajectory.joint_names)
        return trajectory

    def validate(self, trajectory):
        samples = sampled_states(
            trajectory.joint_names,
            [(seconds(p.time_from_start), p.positions) for p in trajectory.points],
            self.state(),
        )
        for sample in samples:
            result = self.validity_of(sample)
            require(result.valid, "Refusing to dispatch a colliding shared trajectory")
        return len(samples)

    def cancel_peers(self):
        for handle, result in self.handles:
            if not result.done():
                self.result(handle.cancel_goal_async(), 5, "peer cancellation")
                self.result(result, 5, "peer cancellation result")

    def dispatch(self, trajectories, *, inject_cancel_at=None, scheduled_start=None):
        self.handles = []
        pending = []
        try:
            for inst, trajectory in trajectories:
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = trajectory
                pending.append((inst, self.arm_clients[inst.id].send_goal_async(goal)))
                self.dispatched += 1
            rejected = []
            for inst, future in pending:
                handle = self.result(future, 5, "arm goal acceptance")
                if handle.accepted:
                    self.handles.append((handle, handle.get_result_async()))
                else:
                    rejected.append(inst.id)
            require(not rejected, f"Arm goal rejected: {rejected}")
            if scheduled_start is not None:
                margin = scheduled_start - self.clock_last
                require(
                    margin > 0.1,
                    "Both goals must be accepted at least 0.1s before their start",
                )
                self.acceptance_margins.append(margin)
            injected = [False]

            def finished():
                if (
                    inject_cancel_at is not None
                    and not injected[0]
                    and self.clock_last >= inject_cancel_at
                ):
                    self.handles[0][0].cancel_goal_async()
                    injected[0] = True
                for _, future in self.handles:
                    if future.done():
                        answer = future.result()
                        require(
                            answer.status == GoalStatus.STATUS_SUCCEEDED
                            and answer.result.error_code
                            == FollowJointTrajectory.Result.SUCCESSFUL,
                            "An arm failed; cancelling peers",
                        )
                return all(f.done() for _, f in self.handles)

            self.until(finished, 45, "all arm results")
            for _, trajectory in trajectories:
                self.commanded.update(
                    zip(trajectory.joint_names, trajectory.points[-1].positions)
                )
        except BaseException as primary:
            cleanup = []
            try:
                self.cancel_peers()
            except Exception as error:
                cleanup.append(str(error))
            for _, future in pending:
                try:
                    handle = self.result(future, 5, "pending peer acceptance")
                    if handle.accepted and all(
                        existing is not handle for existing, _ in self.handles
                    ):
                        self.handles.append((handle, handle.get_result_async()))
                except Exception as error:
                    cleanup.append(str(error))
            try:
                self.cancel_peers()
            except Exception as error:
                cleanup.append(str(error))
            if cleanup:
                raise RuntimeError(
                    f"{primary}; peer cleanup errors: {cleanup}"
                ) from primary
            raise

    def trajectory(self, inst, target):
        trajectory = JointTrajectory()
        trajectory.joint_names = inst.joints
        initial = JointTrajectoryPoint()
        initial.positions = [self.commanded[name] for name in inst.joints]
        final = JointTrajectoryPoint()
        final.positions = list(map(float, target))
        final.time_from_start.sec = 3
        trajectory.points = [initial, final]
        return trajectory

    def run(self):
        self.progress("shared_state_discovery")
        self.until(
            lambda: self.positions is not None and self.clock_count > 10,
            30,
            "all workcell states and clock",
        )
        manager = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.until(manager.service_is_ready, 20, "shared manager")
        expected = ["joint_state_broadcaster"] + [
            i.controller for i in self.cell.instances
        ]

        def ready():
            result = self.result(
                manager.call_async(ListControllers.Request()), 5, "workcell controllers"
            )
            return all(
                {c.name: c.state for c in result.controller}.get(name) == "active"
                for name in expected
            )

        self.until(ready, 30, "both arm controllers")
        for client in self.arm_clients.values():
            self.until(client.server_is_ready, 10, "arm action server")
        for client in (self.validity, self.planner):
            self.until(client.service_is_ready, 15, "MoveIt shared services")
        require(
            len(self.get_publishers_info_by_topic("/clock")) == 1,
            "Expected a single simulation clock publisher",
        )
        require(
            len(self.get_publishers_info_by_topic("/joint_states")) == 1,
            "Expected a single combined state publisher",
        )
        self.settle(self.cell.home)
        isolated = []
        planned = []
        for inst in self.cell.instances:
            self.progress(inst.id + "_isolated_trajectory")
            initial = self.state()
            target = inst.profile.data["verification"]["direct_target_rad"]
            trajectory = self.trajectory(inst, target)
            self.validate(trajectory)
            self.dispatch([(inst, trajectory)])
            desired = dict(initial)
            desired.update(zip(inst.joints, target))
            error = self.settle([desired[n] for n in self.joint_names])
            other = [name for name in self.joint_names if name not in inst.joints]
            drift = max(abs(self.state()[name] - initial[name]) for name in other)
            require(drift < 0.01, "Other arm moved during isolated command")
            isolated.append(
                {
                    "instance": inst.id,
                    "other_arm_drift_rad": drift,
                    "tracking_error_rad": error,
                }
            )
        for inst in self.cell.instances:
            self.progress(inst.id + "_planning")
            target = dict(zip(inst.joints, inst.plan_target))
            answer = self.plan(inst.group, target)
            require(
                answer.error_code.val == MoveItErrorCodes.SUCCESS, "Arm plan failed"
            )
            trajectory = self.anchored(answer.trajectory.joint_trajectory)
            count = self.validate(trajectory)
            initial = self.state()
            self.dispatch([(inst, trajectory)])
            initial.update(zip(trajectory.joint_names, trajectory.points[-1].positions))
            self.settle([initial[n] for n in self.joint_names])
            planned.append(
                {
                    "instance": inst.id,
                    "trajectory_points": len(trajectory.points),
                    "collision_samples": count,
                }
            )
        self.progress("joint_planning")
        target = {
            name: value
            for inst in self.cell.instances
            for name, value in zip(
                inst.joints, inst.profile.data["verification"]["direct_target_rad"]
            )
        }
        plan = self.plan("both_arms", target)
        require(plan.error_code.val == MoveItErrorCodes.SUCCESS, "Both-arm plan failed")
        trajectory = self.anchored(plan.trajectory.joint_trajectory)
        count = self.validate(trajectory)
        joint_point_count = len(trajectory.points)
        indices = partitions(trajectory.joint_names, self.cell.instances)
        quiet = [0, self.joint_count]

        def stationary():
            if quiet[1] == self.joint_count:
                return False
            quiet[1] = self.joint_count
            quiet[0] = (
                quiet[0] + 1
                if self.latest_velocities
                and max(
                    abs(self.latest_velocities.get(n, float("inf")))
                    for n in self.joint_names
                )
                < 0.0001
                else 0
            )
            return quiet[0] >= 10

        self.until(stationary, 10, "stationary arms before timing observation")
        start = self.clock_last + 0.75
        stamp = Time(sec=int(start), nanosec=int((start - int(start)) * 1e9))
        commands = []
        for inst in self.cell.instances:
            split = JointTrajectory()
            split.header.stamp = stamp
            split.joint_names = inst.joints
            for point in trajectory.points:
                cloned = deepcopy(point)
                for field in ("positions", "velocities", "accelerations", "effort"):
                    values = getattr(point, field)
                    if values:
                        setattr(
                            cloned, field, [values[index] for index in indices[inst.id]]
                        )
                split.points.append(cloned)
            commands.append((inst, split))
        self.watch = {"initial": self.state(), "observed": {}}
        self.dispatch(commands, scheduled_start=start)
        endpoint = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        self.settle([endpoint[n] for n in self.joint_names])
        observed = dict(self.watch["observed"])
        self.watch = None
        require(len(observed) == 2, "Both arm movement starts were not observed")
        skew = max(observed.values()) - min(observed.values())
        require(skew <= 0.10, "Observed arm starts differ by more than 0.10s")
        require(
            min(observed.values()) >= start - 0.03,
            f"An arm moved before scheduled start {start}: {observed}",
        )
        self.progress("collision_goal_rejection")
        metadata = check_workcell(self.cell)
        rng = random.Random(42)
        probe = None
        contacts = []
        for attempt in range(400):
            candidate = {
                name: min(
                    metadata["joint_limits"][name]["upper"] - 0.02,
                    max(
                        metadata["joint_limits"][name]["lower"] + 0.02,
                        value + rng.uniform(-0.9, 0.9),
                    ),
                )
                for name, value in zip(self.joint_names, self.cell.home)
            }
            if attempt == 0:
                candidate = {
                    name: value
                    for inst in self.cell.instances
                    for name, value in zip(
                        inst.joints,
                        inst.profile.data["verification"]["moveit_target_rad"],
                    )
                }
            answer = self.validity_of(candidate)
            contacts = [(c.contact_body_1, c.contact_body_2) for c in answer.contacts]
            prefixes = [i.prefix for i in self.cell.instances]
            cross = any(
                (a.startswith(prefixes[0]) and b.startswith(prefixes[1]))
                or (a.startswith(prefixes[1]) and b.startswith(prefixes[0]))
                for a, b in contacts
            )
            if not answer.valid and cross:
                probe = candidate
                break
        require(
            probe is not None,
            "Could not find the deterministic cross-arm collision probe",
        )
        before = self.dispatched
        rejected = self.plan("both_arms", probe)
        require(
            rejected.error_code.val != MoveItErrorCodes.SUCCESS,
            "MoveIt accepted the colliding target",
        )
        require(self.dispatched == before, "A rejected collision plan was dispatched")
        self.progress("peer_rejection_cancellation")
        initial = self.state()
        commands = []
        cancel_start = self.clock_last + 1.0
        for index, inst in enumerate(self.cell.instances):
            trajectory = self.trajectory(inst, [initial[n] + 0.05 for n in inst.joints])
            trajectory.header.stamp = Time(
                sec=int(cancel_start),
                nanosec=int((cancel_start - int(cancel_start)) * 1e9),
            )
            if index == 1:
                trajectory.joint_names[0] = "invalid_joint"
            commands.append((inst, trajectory))
        try:
            self.dispatch(commands, scheduled_start=cancel_start)
        except RuntimeError as error:
            require("Arm goal rejected" in str(error), str(error))
        else:
            raise RuntimeError("Injected malformed peer goal was accepted")
        require(
            all(
                f.done() and f.result().status == GoalStatus.STATUS_CANCELED
                for _, f in self.handles
            ),
            "Accepted peer was not cancelled",
        )
        self.until(
            lambda: self.clock_last > cancel_start + 0.2,
            5,
            "peer cancellation observation",
        )
        self.settle([initial[n] for n in self.joint_names])
        self.progress("active_peer_cancellation")
        initial = self.state()
        start_active = self.clock_last + 0.5
        commands = []
        combined = JointTrajectory()
        combined.joint_names = list(self.joint_names)
        first = JointTrajectoryPoint()
        first.positions = [self.commanded[n] for n in self.joint_names]
        end = JointTrajectoryPoint()
        end.positions = [
            self.commanded[n]
            + (0.05 if n in {i.joints[0] for i in self.cell.instances} else 0.0)
            for n in self.joint_names
        ]
        end.time_from_start.sec = 3
        combined.points = [first, end]
        self.validate(combined)
        active_indices = partitions(combined.joint_names, self.cell.instances)
        for inst in self.cell.instances:
            command = self.trajectory(
                inst, [end.positions[k] for k in active_indices[inst.id]]
            )
            command.header.stamp = Time(
                sec=int(start_active),
                nanosec=int((start_active - int(start_active)) * 1e9),
            )
            commands.append((inst, command))
        try:
            self.dispatch(
                commands,
                inject_cancel_at=start_active + 0.5,
                scheduled_start=start_active,
            )
        except RuntimeError as error:
            require("An arm failed" in str(error), str(error))
        else:
            raise RuntimeError("Active cancellation injection did not stop both goals")
        require(
            all(
                f.done() and f.result().status == GoalStatus.STATUS_CANCELED
                for _, f in self.handles
            ),
            "Active peer was not cancelled",
        )
        moved = {
            inst.id: max(abs(self.state()[n] - initial[n]) for n in inst.joints)
            for inst in self.cell.instances
        }
        require(
            all(value > 0.001 for value in moved.values()),
            "Cancellation was not exercised while both arms were moving",
        )
        self.until(
            lambda: self.latest_velocities
            and max(
                abs(self.latest_velocities.get(n, float("inf")))
                for n in self.joint_names
            )
            < 0.001,
            5,
            "arms stopped after cancellation",
        )
        missing = {
            i.prefix + i.profile.ros["tip_frame"] for i in self.cell.instances
        } - self.transforms
        require(not missing, f"Missing instance TF: {missing}")
        require(
            not self.clock_backwards and self.clock_last > self.clock_first,
            "Invalid shared simulation clock",
        )
        print(
            "PASS: shared 12-joint state, isolated arms, joint planning, collision rejection and peer cancellation",
            flush=True,
        )
        return {
            "workcell_id": self.cell.id,
            "rmw_implementation": get_rmw_implementation_identifier(),
            "joints": self.joint_names,
            "controllers": expected,
            "clock_publishers": 1,
            "joint_state_publishers": 1,
            "isolated_arms": isolated,
            "arm_plans": planned,
            "joint_plan_points": joint_point_count,
            "joint_plan_collision_samples": count,
            "collision_samples_total": self.validity_samples,
            "interpolation_max_step_rad": 0.025,
            "interpolation_max_dt_sec": 0.05,
            "common_start_sec": start,
            "future_goal_acceptance_margins_sec": self.acceptance_margins,
            "observed_start_sec": observed,
            "observed_start_skew_sec": skew,
            "observed_start_tolerance_sec": 0.10,
            "collision_probe_rad": probe,
            "cross_arm_contacts": contacts,
            "collision_goal_rejected": True,
            "peer_rejection_cancels_other_arm": True,
            "active_cancellation_stops_both_arms": True,
            "movement_before_active_cancel_rad": moved,
            "tf_child_frames": sorted(self.transforms),
            "final_joint_positions_rad": self.positions,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell", default="fr5_ur5e")
    parser.add_argument("--domain", type=int, default=93)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    try:
        identifier(args.workcell)
    except ValueError as error:
        parser.error(str(error))
    args.robot = args.workcell
    args.scene = "default"
    args.stop_mode = "interrupt"
    args.screenshot_dir = None
    return run_verification(
        args,
        ROOT,
        rclpy,
        WorkcellVerification,
        Trigger,
        profile_loader=lambda name, **kw: load_workcell(name, root=kw["root"]),
        validator=check_workcell,
        command_builder=lambda cell, args: [
            "bash",
            str(ROOT / "run_scene.sh"),
            f"workcell:={cell.id}",
            f"gui:={str(args.gui).lower()}",
            f"rviz:={str(args.gui).lower()}",
        ],
        report_prefix="workcell_",
    )
