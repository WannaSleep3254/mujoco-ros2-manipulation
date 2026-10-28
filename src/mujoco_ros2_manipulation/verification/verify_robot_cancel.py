"""Owned single-robot cancellation and fresh physical-stop observations.

The existing Verification.run() is reused first. Cancellation response, Action
CANCELED and the subsequent observed joint-stop window are separate evidence.
This is simulation validation, not certification of a physical safety stop.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
from numbers import Integral
from pathlib import Path
import re
import time
import uuid

from ..paths import ROOT
from .reports import atomic_json


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def cancel_diagnostic_environment(report, args):
    """Enable the existing native contact guard for this owned verifier only."""
    from .diagnostics import diagnostic_environment
    environment = diagnostic_environment(report, args)
    environment["MANIPULATION_CONTACT_STOP"] = "1"
    return environment


def normalized_goal_uuid(raw):
    """Validate all 16 transport bytes and detach them as native JSON integers."""
    try:
        values = iter(raw)
    except TypeError as error:
        raise ValueError("Goal UUID must contain exactly 16 integer bytes") from error
    result = []
    for value in values:
        _require(len(result) < 16, "Goal UUID must contain exactly 16 integer bytes")
        _require(isinstance(value, Integral) and not isinstance(value, bool)
            and 0 <= value <= 255, "Goal UUID bytes must be integers within 0..255")
        result.append(int(value))
    _require(len(result) == 16, "Goal UUID must contain exactly 16 integer bytes")
    return result


def normalized_state(expected_names, names, positions, velocities):
    """Reject duplicate, partial or mismatched mappings rather than zip-truncate."""
    expected = tuple(expected_names)
    _require(type(names) in (list, tuple) and type(positions) in (list, tuple)
        and type(velocities) in (list, tuple) and len(names) == len(positions) == len(velocities) == len(expected)
        and all(type(name) is str and name for name in names)
        and len(set(names)) == len(names) and set(names) == set(expected),
        "Joint observation must map every expected joint exactly once, including velocities")
    _require(all(_finite(value) and abs(value) <= 1e6 for value in (*positions, *velocities)),
             "Joint observation contains nonfinite or unbounded position/velocity")
    mapping = {name: (position, velocity) for name, position, velocity in zip(names, positions, velocities)}
    return dict(positions_rad=[mapping[name][0] for name in expected],
                velocities_rad_sec=[mapping[name][1] for name in expected])


class StopEvidence:
    """A bounded post-cancellation stream, never success from a stale snapshot."""
    def __init__(self, joint_names, *, minimum_window_sec=.4, minimum_samples=5,
                 position_drift_rad=.002, velocity_rad_sec=.02, max_age_sec=.5, max_gap_sec=.2):
        _require(type(joint_names) in (list, tuple) and 1 <= len(joint_names) <= 64
            and all(type(name) is str and name for name in joint_names)
            and len(set(joint_names)) == len(joint_names), "Expected unique bounded joint names")
        _require(type(minimum_samples) is int and 3 <= minimum_samples <= 100,
                 "Expected 3..100 samples for a physical-stop window")
        _require(all(_finite(value) and value > 0 for value in (minimum_window_sec,
            position_drift_rad, velocity_rad_sec, max_age_sec, max_gap_sec))
            and .1 <= minimum_window_sec <= 2. and max_gap_sec < minimum_window_sec,
            "Invalid bounded physical-stop timing or tolerances")
        self.joint_names = tuple(joint_names)
        self.minimum_window_sec, self.minimum_samples = minimum_window_sec, minimum_samples
        self.position_drift_rad, self.velocity_rad_sec = position_drift_rad, velocity_rad_sec
        self.max_age_sec, self.max_gap_sec = max_age_sec, max_gap_sec
        self.started_ns = self.minimum_source = self.minimum_clock = None
        self.window, self.samples, self.rejections = [], [], []
        self.previous_source = self.previous_clock = self.previous_received = None
        self.observation_context = "canceled_terminal"

    def begin(self, *, terminal_received_ns, minimum_source_sec, minimum_clock_sec,
              goal_accepted, movement_observed, cancel_response_accepted, action_status):
        _require(self.started_ns is None, "A stop-evidence context cannot be reused")
        _require(goal_accepted is True and movement_observed is True and cancel_response_accepted is True
            and type(action_status) is int and action_status == 5,
            "Physical-stop evidence requires an accepted moving goal, acknowledged cancellation and Action CANCELED")
        _require(type(terminal_received_ns) is int and terminal_received_ns > 0
            and _finite(minimum_source_sec) and minimum_source_sec >= 0
            and _finite(minimum_clock_sec) and minimum_clock_sec >= 0,
            "Expected actual cancellation source/clock watermarks")
        self.started_ns, self.minimum_source, self.minimum_clock = terminal_received_ns, minimum_source_sec, minimum_clock_sec

    def _reject(self, reason):
        self.window.clear()
        self.rejections.append(reason)
        _require(len(self.rejections) <= 2000, "Stop observation rejection bound exceeded")
        return False

    def observe(self, *, names, positions, velocities, source_sec, clock_sec,
                received_ns, clock_received_ns, now_ns):
        _require(self.started_ns is not None, "No actual active cancellation context exists")
        state = normalized_state(self.joint_names, names, positions, velocities)
        _require(all(type(value) is int and value > 0 for value in (received_ns, clock_received_ns, now_ns))
            and _finite(source_sec) and _finite(clock_sec) and source_sec >= 0 and clock_sec >= 0,
            "Expected finite source/clock and local monotonic receive times")
        _require(now_ns >= received_ns and now_ns >= clock_received_ns,
                 "Observation receive times cannot be in the future")
        if received_ns <= self.started_ns or source_sec <= self.minimum_source or clock_sec <= self.minimum_clock:
            return self._reject("not_after_" + self.observation_context)
        if max((now_ns-received_ns)/1e9, (now_ns-clock_received_ns)/1e9) > self.max_age_sec:
            return self._reject("stale_joint_or_clock_delivery")
        if abs(source_sec-clock_sec) > .1:
            return self._reject("source_clock_disagreement")
        if ((self.previous_source is not None and source_sec <= self.previous_source)
                or (self.previous_clock is not None and clock_sec <= self.previous_clock)
                or (self.previous_received is not None and received_ns <= self.previous_received)):
            return self._reject("repeated_or_backwards_source_clock_receive")
        self.previous_source, self.previous_clock, self.previous_received = source_sec, clock_sec, received_ns
        sample = dict(source_sec=source_sec, clock_sec=clock_sec, received_ns=received_ns,
            clock_received_ns=clock_received_ns, **state)
        self.samples.append(sample)
        _require(len(self.samples) <= 2000, "Physical-stop sample bound exceeded")
        if max(abs(value) for value in state["velocities_rad_sec"]) > self.velocity_rad_sec:
            return self._reject("joint_velocity_above_stop_threshold")
        if self.window and max((received_ns-self.window[-1]["received_ns"])/1e9,
            source_sec-self.window[-1]["source_sec"], clock_sec-self.window[-1]["clock_sec"]) > self.max_gap_sec:
            self._reject("observation_gap")
        self.window.append(sample)
        if self._position_drift() > self.position_drift_rad:
            self._reject("position_drift_above_stop_threshold")
            self.window.append(sample)
        return self.snapshot(now_ns=now_ns)["physical_stop_observed"]

    def _position_drift(self):
        return max((max(row["positions_rad"][index] for row in self.window)
                    - min(row["positions_rad"][index] for row in self.window)
                    for index in range(len(self.joint_names))), default=0.) if self.window else 0.

    def snapshot(self, *, now_ns):
        _require(type(now_ns) is int and now_ns > 0, "Expected local monotonic observation time")
        first, last = (self.window[0], self.window[-1]) if self.window else (None, None)
        wall = (last["received_ns"]-first["received_ns"])/1e9 if last else 0.
        source = last["source_sec"]-first["source_sec"] if last else 0.
        clock = last["clock_sec"]-first["clock_sec"] if last else 0.
        fresh = bool(last and now_ns >= last["received_ns"] and now_ns >= last["clock_received_ns"]
            and max(now_ns-last["received_ns"], now_ns-last["clock_received_ns"]) <= self.max_age_sec*1e9)
        confirmed = bool(fresh and len(self.window) >= self.minimum_samples
            and min(wall, source, clock) >= self.minimum_window_sec-1e-9
            and self._position_drift() <= self.position_drift_rad)
        return dict(schema_version=1, physical_stop_observed=confirmed, fresh=fresh,
            joint_names=list(self.joint_names), window_samples=len(self.window),
            wall_window_sec=wall, source_window_sec=source, clock_window_sec=clock,
            maximum_position_drift_rad=self._position_drift(),
            maximum_velocity_rad_sec=max((abs(value) for row in self.window for value in row["velocities_rad_sec"]), default=None),
            requirements=dict(minimum_window_sec=self.minimum_window_sec, minimum_samples=self.minimum_samples,
                position_drift_rad=self.position_drift_rad, velocity_rad_sec=self.velocity_rad_sec,
                max_age_sec=self.max_age_sec, max_gap_sec=self.max_gap_sec),
            rejected_observations=len(self.rejections), rejection_reasons=list(self.rejections),
            total_samples=len(self.samples), samples=deepcopy(self.samples),
            interpretation="Fresh advancing joint states and clock after Action CANCELED; process exit is not physical-stop evidence")


class HomeRecoveryEvidence(StopEvidence):
    """Fresh settled target observations after this new successful home Action."""
    def __init__(self, joint_names, target_rad, *, tracking_tolerance_rad, minimum_samples=15):
        super().__init__(joint_names, minimum_samples=minimum_samples)
        _require(type(target_rad) in (list, tuple) and len(target_rad) == len(self.joint_names)
            and all(_finite(value) for value in target_rad) and _finite(tracking_tolerance_rad)
            and 0 < tracking_tolerance_rad <= .1, "Home recovery requires a complete finite target and tolerance")
        self.target_rad = list(target_rad)
        self.tracking_tolerance_rad = tracking_tolerance_rad
        self.action_context = None
        self.observation_context = "home_terminal"

    def begin(self, *, sent_ns, terminal_received_ns, source_before_send, clock_before_send,
              minimum_source_sec, minimum_clock_sec, goal_accepted, action_status, error_code):
        _require(self.started_ns is None and goal_accepted is True and type(action_status) is int
            and action_status == 4 and type(error_code) is int and error_code == 0,
            "Home observations require this accepted Action's successful terminal result")
        _require(type(sent_ns) is int and type(terminal_received_ns) is int and 0 < sent_ns < terminal_received_ns
            and all(_finite(value) and value >= 0 for value in (source_before_send, clock_before_send,
                minimum_source_sec, minimum_clock_sec))
            and minimum_source_sec >= source_before_send and minimum_clock_sec >= clock_before_send,
            "Home Action or source/clock watermarks are stale or backwards")
        self.started_ns = terminal_received_ns
        self.minimum_source, self.minimum_clock = minimum_source_sec, minimum_clock_sec
        self.action_context = dict(sent_wall_ns=sent_ns, terminal_received_wall_ns=terminal_received_ns,
            source_before_send_sec=source_before_send, clock_before_send_sec=clock_before_send,
            source_at_terminal_sec=minimum_source_sec, clock_at_terminal_sec=minimum_clock_sec,
            goal_accepted=True, action_status=action_status, error_code=error_code)

    def observe(self, **sample):
        super().observe(**sample)
        state = normalized_state(self.joint_names, sample["names"], sample["positions"], sample["velocities"])
        if max(abs(actual - target) for actual, target in zip(state["positions_rad"], self.target_rad)) > self.tracking_tolerance_rad:
            return self._reject("home_target_outside_tracking_tolerance")
        return self.snapshot(now_ns=sample["now_ns"])["home_recovery_observed"]

    def snapshot(self, *, now_ns):
        value = super().snapshot(now_ns=now_ns)
        maximum = max((abs(row["positions_rad"][index] - target) for row in self.window
                       for index, target in enumerate(self.target_rad)), default=None)
        value.update(home_recovery_observed=bool(value["physical_stop_observed"] and maximum is not None
            and maximum <= self.tracking_tolerance_rad), target_rad=list(self.target_rad),
            maximum_target_error_rad=maximum, tracking_tolerance_rad=self.tracking_tolerance_rad,
            action_context=deepcopy(self.action_context),
            interpretation="Fresh advancing mapped states and clock after this successful home Action; target/velocity/drift window required")
        return value


def capture_confirmation(evidence, *, now_ns, field="physical_stop_observed"):
    """Freeze exactly the snapshot accepted by a predicate, at the same instant."""
    _require(field in ("physical_stop_observed", "home_recovery_observed"), "Unknown confirmation evidence field")
    value = evidence.snapshot(now_ns=now_ns)
    if value.get(field) is not True:
        return None
    return dict(deepcopy(value), confirmed_at_wall_ns=now_ns)


def completion_flags(record):
    """Report proof-derived flags rather than unconditional successful literals."""
    cancel = record.get("cancellation_response") or {}
    terminal = record.get("action_terminal") or {}
    stop = record.get("physical_stop") or {}
    recovery = record.get("recovery") or {}
    recovered = recovery.get("fresh_state") or {}
    return dict(cancel_response_accepted=cancel.get("own_goal_acknowledged") is True,
        action_canceled=type(terminal.get("status")) is int and terminal["status"] == 5 and terminal.get("canceled") is True,
        physical_stop_observed=stop.get("physical_stop_observed") is True and stop.get("fresh") is True
            and type(stop.get("confirmed_at_wall_ns")) is int and stop["confirmed_at_wall_ns"] > 0,
        home_recovery_succeeded=recovery.get("action_succeeded") is True
            and recovery.get("goal_accepted") is True
            and type(recovery.get("action_status")) is int and recovery["action_status"] == 4
            and type(recovery.get("error_code")) is int and recovery["error_code"] == 0
            and type(recovery.get("sent_wall_ns")) is int and type(recovery.get("terminal_received_wall_ns")) is int
            and 0 < recovery["sent_wall_ns"] < recovery["terminal_received_wall_ns"]
            and recovered.get("home_recovery_observed") is True and recovered.get("fresh") is True
            and type(recovered.get("confirmed_at_wall_ns")) is int and recovered["confirmed_at_wall_ns"] > 0)


def cancel_target(joint_names, positions, limits, *, offset_rad=.15):
    _require(type(offset_rad) in (int, float) and .05 <= offset_rad <= .2, "Expected a small bounded cancel-test offset")
    _require(type(joint_names) in (list, tuple) and type(positions) in (list, tuple)
        and 1 <= len(joint_names) == len(positions) <= 64
        and all(type(name) is str and name for name in joint_names)
        and len(set(joint_names)) == len(joint_names) and type(limits) is dict and set(limits) == set(joint_names)
        and all(_finite(q) for q in positions), "Cancel target requires complete actual joints and generated limits")
    for name, q in zip(joint_names, positions):
        _require(type(limits[name]) is dict and all(_finite(limits[name].get(key)) for key in ("lower", "upper", "velocity"))
            and limits[name]["lower"] < limits[name]["upper"] and limits[name]["velocity"] > 0,
            "Cancel target requires finite generated position/velocity limits")
        _require(limits[name]["lower"] <= q <= limits[name]["upper"], "Observed initial joint lies outside generated limits")
    target = list(positions)
    name, q = joint_names[0], positions[0]
    for direction in (1., -1.):
        value = q+direction*offset_rad
        if limits[name]["lower"]+.01 <= value <= limits[name]["upper"]-.01:
            target[0] = value
            return target
    raise ValueError("No bounded first-joint cancellation motion fits the generated limits")


def moving_observation(initial, sample, *, sent_ns, source_before, now_ns):
    """Fresh measured displacement and velocity, while the owner checks active Action."""
    _require(type(sample) is dict and type(initial) in (list, tuple)
        and len(initial) == len(sample.get("positions", [])) == len(sample.get("velocities", []))
        and len(initial) > 0 and all(_finite(value) for value in (
            *initial, *sample["positions"], *sample["velocities"], sample.get("source_sec"), source_before))
        and all(type(value) is int and value > 0 for value in (sent_ns, sample.get("received_ns"), now_ns)),
        "Actual movement requires a finite complete observed state and receive times")
    return bool(sent_ns < sample["received_ns"] <= now_ns and sample["source_sec"] > source_before
        and (now_ns-sample["received_ns"])/1e9 <= .5
        and max(abs(a-b) for a, b in zip(sample["positions"], initial)) >= .005
        and max(abs(value) for value in sample["velocities"]) > .002)


def node_class():
    """Keep StopEvidence usable without importing or starting ROS."""
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from .ros_checks import Verification

    class RobotCancelVerification(Verification):
        def __init__(self, profile, progress=None, *, output):
            self.clock_received_ns = None
            self.latest_sample = None
            self.stop_evidence = None
            self.recovery_evidence = None
            self.directory = Path(output)
            self.directory.mkdir(parents=True, exist_ok=False)
            self.record = dict(schema_version=1, status="running", phase="baseline", robot_id=profile.id,
                baseline=None, cancellation_response=None, action_terminal=None, physical_stop=None,
                recovery=None, error=None, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                scope="Owned simulation only; direct action cancellation and observed stop are distinct from physical robot safety")
            self.save()
            super().__init__(profile, progress)

        def save(self):
            atomic_json(self.directory/"cancel-verification.json", self.record)

        def clock(self, message):
            _require(_finite(message.clock.sec+message.clock.nanosec*1e-9)
                and message.clock.sec >= 0, "Invalid simulation clock during robot cancellation verification")
            super().clock(message)
            self.clock_received_ns = time.monotonic_ns()

        def joints(self, message):
            super().joints(message)
            state = normalized_state(self.joint_names, list(message.name), list(message.position), list(message.velocity))
            source = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            _require(_finite(source) and source >= 0, "Invalid joint source timestamp")
            self.latest_sample = dict(names=list(self.joint_names), positions=state["positions_rad"],
                velocities=state["velocities_rad_sec"], source_sec=source, received_ns=time.monotonic_ns())
            if self.stop_evidence is not None:
                self.stop_evidence.observe(**self.latest_sample, clock_sec=self.clock_last,
                    clock_received_ns=self.clock_received_ns, now_ns=time.monotonic_ns())
            if self.recovery_evidence is not None:
                self.recovery_evidence.observe(**self.latest_sample, clock_sec=self.clock_last,
                    clock_received_ns=self.clock_received_ns, now_ns=time.monotonic_ns())

        def goal(self, target, duration):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(self.joint_names)
            first, last = JointTrajectoryPoint(), JointTrajectoryPoint()
            first.positions, first.velocities = list(self.positions), [0.]*len(self.joint_names)
            first.time_from_start.nanosec = 100_000_000
            last.positions, last.velocities = list(target), [0.]*len(self.joint_names)
            last.time_from_start.sec, last.time_from_start.nanosec = divmod(round(duration*1e9), 1_000_000_000)
            goal.trajectory.points = [first, last]
            return goal

        def run(self):
            try:
                self.record["baseline"] = super().run()
                self.record["phase"] = "active_cancel_goal"
                self.progress("active_cancel_goal")
                self.save()
                metadata = json.loads((self.profile.output/"source.json").read_text())
                limits = metadata["joint_limits"]
                initial = list(self.positions)
                target = cancel_target(self.joint_names, initial, limits)
                self.record["cancel_goal"] = dict(initial_rad=initial, target_rad=target, duration_sec=8.,
                    model_manifest_sha256=hashlib.sha256((self.profile.output/"source.json").read_bytes()).hexdigest())
                client = ActionClient(self, FollowJointTrajectory,
                    "/"+self.profile.ros["trajectory_controller"]+"/follow_joint_trajectory")
                self.until(client.server_is_ready, 10, "owned cancel-test action server")
                sent_ns = time.monotonic_ns()
                source_before = self.latest_sample["source_sec"]
                handle = self.result(client.send_goal_async(self.goal(target, 8.)), 5, "cancel-test acceptance")
                _require(handle.accepted is True, "New cancellation goal was rejected")
                future = handle.get_result_async()
                def moving():
                    _require(not future.done(), "Cancellation goal completed before active movement was observed")
                    sample = self.latest_sample
                    return bool(sample and moving_observation(initial, sample, sent_ns=sent_ns,
                        source_before=source_before, now_ns=time.monotonic_ns()))
                self.until(moving, 5, "fresh actual movement before cancellation")
                self.record["movement_before_cancel"] = deepcopy(self.latest_sample)
                _require(not future.done(), "Cannot test active cancellation of an already terminal goal")
                self.record["phase"] = "cancel_response"
                self.progress("cancel_response")
                cancel_sent = time.monotonic_ns()
                response = self.result(handle.cancel_goal_async(), 5, "owned cancellation response")
                own_uuid = normalized_goal_uuid(handle.goal_id.uuid)
                acknowledged = response.return_code == 0 and any(normalized_goal_uuid(item.goal_id.uuid) == own_uuid
                    for item in response.goals_canceling)
                self.record["cancellation_response"] = dict(return_code=response.return_code,
                    own_goal_acknowledged=acknowledged, goals_canceling=len(response.goals_canceling),
                    own_goal_uuid=own_uuid,
                    request_wall_ns=cancel_sent, response_wall_ns=time.monotonic_ns())
                self.save()
                _require(acknowledged, "Cancellation response did not acknowledge this owned active goal")
                self.record["phase"] = "action_terminal"
                self.progress("action_terminal")
                terminal = self.result(future, 10, "owned canceled Action terminal")
                terminal_ns = time.monotonic_ns()
                self.record["action_terminal"] = dict(status=terminal.status,
                    canceled=terminal.status == GoalStatus.STATUS_CANCELED,
                    error_code=terminal.result.error_code, error_string=terminal.result.error_string,
                    received_wall_ns=terminal_ns)
                self.save()
                _require(terminal.status == GoalStatus.STATUS_CANCELED, "Action terminal did not report CANCELED")
                self.record["phase"] = "physical_stop_window"
                self.progress("physical_stop_window")
                self.stop_evidence = StopEvidence(self.joint_names)
                self.stop_evidence.begin(terminal_received_ns=terminal_ns,
                    minimum_source_sec=self.latest_sample["source_sec"], minimum_clock_sec=self.clock_last,
                    goal_accepted=True, movement_observed=True, cancel_response_accepted=acknowledged,
                    action_status=int(terminal.status))
                def stopped():
                    value = capture_confirmation(self.stop_evidence, now_ns=time.monotonic_ns())
                    if value is None:
                        return False
                    self.record["physical_stop"] = value
                    return True
                self.until(stopped, 8, "fresh advancing joint position/velocity stop window")
                _require(completion_flags(self.record)["physical_stop_observed"], "No authoritative stop observation was captured")
                _require(not self.clock_backwards, "Simulation clock moved backwards during cancellation")
                self.stop_evidence = None
                self.record["phase"] = "home_recovery"
                self.progress("home_recovery")
                self.save()
                home = list(self.profile.home)
                _require(len(home) == len(self.joint_names) and all(limits[name]["lower"] <= value <= limits[name]["upper"]
                    for name, value in zip(self.joint_names, home)), "Home recovery exceeds generated joint limits")
                home_sent_ns = time.monotonic_ns()
                home_source, home_clock = self.latest_sample["source_sec"], self.clock_last
                self.record["recovery"] = dict(action_succeeded=False, target_rad=home, fresh_state=None,
                    sent_wall_ns=home_sent_ns, source_before_send_sec=home_source, clock_before_send_sec=home_clock)
                self.save()
                home_handle = self.result(client.send_goal_async(self.goal(home, max(3., self.settings["trajectory_duration_sec"]))),
                                          5, "new owned home goal acceptance")
                home_uuid = normalized_goal_uuid(home_handle.goal_id.uuid)
                _require(home_handle.accepted is True and home_uuid != own_uuid, "Home recovery requires a new accepted owned goal")
                self.record["recovery"].update(goal_accepted=True, goal_uuid=home_uuid)
                recovered = self.result(home_handle.get_result_async(), 20, "new owned home Action terminal")
                home_terminal_ns = time.monotonic_ns()
                self.record["recovery"].update(action_status=recovered.status, error_code=recovered.result.error_code,
                    terminal_received_wall_ns=home_terminal_ns)
                _require(recovered.status == GoalStatus.STATUS_SUCCEEDED
                    and recovered.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL, "Home recovery controller failed")
                self.record["recovery"]["action_succeeded"] = True
                self.recovery_evidence = HomeRecoveryEvidence(self.joint_names, home,
                    tracking_tolerance_rad=self.settings["tracking_tolerance_rad"], minimum_samples=self.settings["settle_messages"])
                self.recovery_evidence.begin(sent_ns=home_sent_ns, terminal_received_ns=home_terminal_ns,
                    source_before_send=home_source, clock_before_send=home_clock,
                    minimum_source_sec=self.latest_sample["source_sec"], minimum_clock_sec=self.clock_last,
                    goal_accepted=True, action_status=int(recovered.status), error_code=int(recovered.result.error_code))
                def recovered_fresh():
                    value = capture_confirmation(self.recovery_evidence, now_ns=time.monotonic_ns(), field="home_recovery_observed")
                    if value is None:
                        return False
                    self.record["recovery"]["fresh_state"] = value
                    return True
                self.until(recovered_fresh, 10, "fresh advancing home target/velocity/clock window")
                observed = self.record["recovery"]["fresh_state"]["samples"][-1]["positions_rad"]
                error = [actual - desired for actual, desired in zip(observed, home)]
                _require(not self.clock_backwards, "Simulation clock moved backwards during home recovery")
                self.record["recovery"].update(tracking_error_rad=error, actual_rad=list(observed),
                    clock_sec=self.record["recovery"]["fresh_state"]["samples"][-1]["clock_sec"])
                self.recovery_evidence = None
                _require(all(completion_flags(self.record).values()), "Completed run lacks authoritative cancellation/recovery evidence")
                self.record.update(status="completed", phase="completed")
            except BaseException as error:
                self.record.update(status="failed", error=dict(type=type(error).__name__, message=str(error)))
                raise
            finally:
                if self.stop_evidence is not None and self.record["physical_stop"] is None:
                    self.record["physical_stop"] = self.stop_evidence.snapshot(now_ns=time.monotonic_ns())
                if self.recovery_evidence is not None and self.record["recovery"].get("fresh_state") is None:
                    self.record["recovery"]["fresh_state"] = self.recovery_evidence.snapshot(now_ns=time.monotonic_ns())
                self.save()
            payload = deepcopy(self.record["baseline"])
            payload.update(robot_cancel_verification_path=str(self.directory/"cancel-verification.json"),
                robot_cancel_verification_sha256=hashlib.sha256((self.directory/"cancel-verification.json").read_bytes()).hexdigest(),
                **completion_flags(self.record),
                final_joint_positions_rad=list(self.positions), joint_state_messages=self.joint_count,
                clock_messages=self.clock_count, clock_end_sec=self.clock_last)
            return payload
    return RobotCancelVerification


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="fr3")
    parser.add_argument("--scene", default="default")
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232 or any(not re.fullmatch(r"[a-z][a-z0-9_]*", value) for value in (args.robot, args.scene)):
        parser.error("Expected declared robot/scene IDs and an explicit ROS domain within 0..232")
    output = (args.output or ROOT/"runtime/validation/robot-cancel"/uuid.uuid4().hex).resolve()
    if output.exists() or not output.is_relative_to((ROOT/"runtime").resolve()):
        parser.error("Output must be a new directory under this workspace runtime/")
    import rclpy
    from std_srvs.srv import Trigger
    from ..profiles import load_profile, check_generated_model
    from .diagnostics import finalize_physics
    from .runner import run_verification
    from ..workcells.verify_motion_reproduction import require_unused_domain
    profile = load_profile(args.robot, root=ROOT, scene=args.scene)
    check_generated_model(profile)
    require_unused_domain(args.domain)
    args.gui, args.stop_mode, args.screenshot_dir, args.require_gpu = False, "interrupt", None, False
    verifier = node_class()
    return run_verification(args, ROOT, rclpy,
        lambda profile, progress: verifier(profile, progress, output=output), Trigger,
        report_prefix="robot_cancel_", launch_environment=cancel_diagnostic_environment, finalize=finalize_physics)


if __name__ == "__main__":
    raise SystemExit(main())
