"""Owned ROS2-control/MoveIt FR5 grasp task over the full native world.

The free block is moved only by native MuJoCo contact forces.  IK supplies a
candidate, MoveIt supplies a path, native full-state clearance checks the same
controller spline, and only the unchanged checked trajectory is submitted.
Physical grasp, logical attachment, Action terminal and cleanup are independent
reported facts.  This module does not start an HTTP server or command hardware.
"""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
from types import SimpleNamespace
import uuid

from ..model_build.integrity import file_sha256
from ..paths import ROOT
from ..verification.process_faults import process_failures
from ..verification.reports import atomic_json, utc_now
from ..verification.runner import group_gone, stop_process
from ..workcells.config import load_workcell
from ..workcells.interference import Spline
from ..workcells.planning_adapter import MoveItPlanningAdapter
from ..workcells.pose_adapter import MoveItPoseAdapter, PoseAdapterError
from .contract import GraspLifecycle
from .ros_distance import GraspDistanceEvaluator
from .ros_model import FINGER_JOINTS
from .ros_observation import GraspObservationStore, ObservationUnavailable
from .ros_planning_scene import RosGraspPlanningScene, _observed_pose, _relative, _pose_error, _rotate
from .ros_runtime import check_grasp_runtime
from .ros_target import load_ros_grasp_target
from .ros_admission import AcceptedObservationRecorder, ReadOnlyAdmissionWorker, file_stats


CONTROLLERS = ("left_controller", "right_controller", "gripper_controller")
MAX_TASK_WALL_SEC = 240
MAX_FRAME_BYTES = 64 * 1024 * 1024
MAX_RECEIPT_BYTES = 2 * 1024 * 1024


class RosGraspTaskError(RuntimeError):
    pass


class RosGraspCancelled(RosGraspTaskError):
    pass


def _require(condition, message):
    if not condition:
        raise RosGraspTaskError(message)


def identity_inventory(target, pinned_paths):
    """The already checked target's files, assets and generated input manifests."""
    paths = {Path(path).absolute() for path in pinned_paths}
    manifest = target.manifest
    paths.update(target.output/name for name in ("grasp.json", *manifest["outputs"]))
    paths.update(Path(name) for name in manifest["dependencies"])
    paths.update(Path(__file__).with_name(name) for name in manifest["inputs"]["producer_sha256"])
    base = target.base
    report = json.loads((base.output/"source.json").read_text())
    paths.update(base.output/name for name in ("source.json", *report["outputs"]))
    paths.update(base.root/name for name in report["inputs"])
    paths.update((target.output, base.output))
    for instance in base.instances:
        profile = instance.profile
        report = json.loads((profile.output/"source.json").read_text())
        paths.add(profile.output/"source.json")
        paths.add(profile.output)
        paths.update(profile.root/name for name in report["integrity"]["inputs"]["files"])
        paths.update(profile.output/name for name in report["integrity"]["outputs"])
        git = profile.source_path("repository_path")/".git"
        if git.is_dir():
            paths.add(git)
            paths.update(p for p in (git/"HEAD", git/"index", git/"packed-refs") if p.exists())
            if (git/"refs").exists():
                paths.add(git/"refs")
            for index,p in enumerate((git/"refs").rglob("*")):
                _require(index < 2048, "manufacturer Git reference inventory exceeds its bound")
                paths.add(p)
        elif git.exists():
            raise ValueError("manufacturer Git indirection needs an explicit checked stat inventory")
    _require(0 < len(paths) <= 2048, "immutable file inventory exceeds its admission bound")
    return tuple(sorted(map(str, paths)))


def verify_identity_proof(proof, *, manifest_sha256, expected_hashes, inventory, current_stats=file_stats):
    """Owner verifies a read-only result and fast file identity after ROS work."""
    _require(proof.get("manifest_sha256") == manifest_sha256 and proof.get("observed_sha256") == expected_hashes,
             "task target/source/native binary changed before submission")
    _require(set(proof.get("before", {})) == set(inventory) and proof.get("before") == proof.get("after"),
             "task identity files changed during read-only validation")
    _require(current_stats(inventory) == proof["after"], "task identity files changed after validation")


def contact_evidence(value, reference=None):
    """Summarize a checked native observation, without creating evidence."""
    forces, table = [0., 0.], 0.
    for contact in value["contacts"]:
        pair = frozenset((contact["geom1_name"], contact["geom2_name"]))
        for index, name in enumerate(("fr5_left_finger_geom", "fr5_right_finger_geom")):
            if pair == frozenset(("fr5_block_geom", name)):
                forces[index] += contact["normal_force_n"]
        if pair == frozenset(("fr5_block_geom", "fr5_table_geom")):
            table += contact["normal_force_n"]
    relative = _relative(_observed_pose(value["tcp"]), _observed_pose(value["block"]))
    position, angle = _pose_error(relative, reference) if reference is not None else (0., 0.)
    return dict(sequence=value["sequence"], time_sec=value["time_sec"],
                stamp_ns=value["stamp_ns"], stamp_semantics=value["stamp_semantics"],
                finger_force_n=forces, bilateral_contact=min(forces) >= .2,
                table_force_n=table, relative_pose=[*relative[0], *relative[1]],
                relative_position_error_m=position, relative_angle_error_rad=angle,
                block_position_m=list(value["block"]["position"]),
                block_speed_m_s=math.sqrt(sum(x*x for x in value["block"]["linear_velocity_m_sec"])),
                open_gap_m=.07-sum(value["finger_positions_m"].values()),
                force_timing_verified=False,
                force_timing_scope="nominal last native step, eligible only with a fresh unlatched guard")


def _evidence_time_ns(sample):
    """Retain the checked observer's rounded native clock, never exact ROS time."""
    stamp, seconds = sample.get("stamp_ns"), sample.get("time_sec")
    _require(type(stamp) is str and re.fullmatch(r"0|[1-9][0-9]{0,18}", stamp) is not None
             and sample.get("stamp_semantics") == "rounded_native_mjData_time",
             "native evidence requires its original rounded-time stamp")
    _require(type(seconds) in (int, float) and math.isfinite(seconds) and 0 <= seconds <= 2147483647.,
             "native evidence time is invalid")
    rounded = seconds*1e9
    _require(abs(int(stamp)-rounded) <= max(.501, math.ulp(rounded)),
             "native evidence stamp differs from its rounded native time")
    return int(stamp)


def _window_minimum_ns(minimum_sec):
    _require(type(minimum_sec) in (int, float) and math.isfinite(minimum_sec) and 0 < minimum_sec <= 60,
             "native evidence duration is invalid")
    return math.ceil(minimum_sec*1e9)


def checked_window(samples, minimum_sec):
    """Compare rounded native integer stamps; held frames add no evidence time.

    A 150-ms maximum gap remains exactly 150,000,000 native nanoseconds. Native
    floating timestep accumulation must not turn that boundary into a larger
    gap. This clock is not an exact ROS timestamp or a force-timing certificate.
    """
    _require(len(samples) >= 3, "insufficient distinct native observation samples")
    times = [_evidence_time_ns(sample) for sample in samples]
    sequences = [sample["sequence"] for sample in samples]
    _require(all(type(value) is int and value > 0 for value in sequences)
             and all(b > a for a, b in zip(times, times[1:]))
             and all(b["time_sec"] > a["time_sec"] for a,b in zip(samples,samples[1:]))
             and all(b > a for a, b in zip(sequences, sequences[1:])), "native evidence did not advance")
    _require(times[-1]-times[0] >= _window_minimum_ns(minimum_sec), "native evidence duration is insufficient")
    _require(max(b-a for a, b in zip(times, times[1:])) <= 150_000_000, "native evidence has an observation gap")
    return (times[-1]-times[0])/1e9


def collect_evidence_window(duration, *, spin, current_evidence, predicate=None, timeout=8., clock=time.monotonic):
    """Consume each ordered callback once; missing evidence is never filled in.

    The owner uses a SingleThreadedExecutor: one spin executes one handler,
    then this loop inspects that original observation before the next handler.
    An ordered burst therefore retains all native samples, without replaying
    actions or resetting a gap to manufacture a successful interval.
    """
    samples, last_sequence = [], None
    minimum_ns = _window_minimum_ns(duration)
    deadline = clock()+timeout
    while clock() < deadline:
        spin()
        item = current_evidence()
        if item["sequence"] == last_sequence:
            continue
        last_sequence = item["sequence"]
        if predicate is not None and not predicate(item):
            samples = []
            continue
        if not samples or _evidence_time_ns(item) > _evidence_time_ns(samples[-1]):
            samples.append(item)
        if len(samples) >= 3 and _evidence_time_ns(samples[-1])-_evidence_time_ns(samples[0]) >= minimum_ns:
            checked_window(samples, duration)
            return samples
    raise TimeoutError("physical observation window did not complete")


def trajectory_samples(trajectory, start, selected_names, limits):
    """Sample the actual JTC interpolation, preserving all derivatives."""
    curve = Spline.from_ros(trajectory)
    _require(set(curve.names) == set(selected_names) and len(curve.names) == len(selected_names),
             "only the selected FR5 six joints may be commanded")
    _require(max(abs(curve.points[0]["p"][j]-start[name]) for j, name in enumerate(curve.names)) <= .01,
             "planned trajectory start differs from native state")
    count = max(1, math.ceil(curve.duration/.01))
    times = sorted({curve.duration*k/count for k in range(count+1)} | set(curve.times))
    _require(len(times) <= 6000, "native trajectory validation sample limit exceeded")
    result, previous = [], None
    for stamp in times:
        state = dict(start)
        state.update(zip(curve.names, curve.position(stamp)))
        _require(all(limits[name]["lower"] <= value <= limits[name]["upper"] for name, value in state.items()),
                 "controller spline crosses a joint limit")
        _require(previous is None or max(abs(state[name]-previous[name]) for name in state) <= .025,
                 "controller spline exceeds the validation angular step")
        result.append((stamp, state))
        previous = state
    return curve, result


def kinematics_state_limits(manifest):
    """Allow one micrometre of observed slide limit roundoff, without clamping.

    These limits apply only to IK/FK input/response validation. Finger command
    ranges and trajectories remain 0..0.034 m; IK may command only six arm axes.
    """
    limits = deepcopy(dict(manifest["arm_joint_limits"], **manifest["finger_joint_limits"]))
    for name in FINGER_JOINTS:
        limits[name]["lower"] -= 1e-6
        limits[name]["upper"] += 1e-6
    return limits


def latest_action_terminal(actions):
    """The newest submitted action owns the displayed terminal, even pending."""
    if not actions:
        return None
    latest = actions[-1]
    return dict(stage=latest["stage"], status=latest.get("terminal_status"),
                error_code=latest.get("terminal_error_code"),
                terminal_observed=bool(latest.get("terminal_observed")), physical_grasp_claim=False)


def cancelled_hold_observation(lifecycle, *, stop_observed):
    """A held native clock cannot establish a new physical grasp after cancel."""
    _require(stop_observed is True, "native stop must be observed before cancelled hold is finalized")
    prior = lifecycle.physical_grasp
    lifecycle.cancelled_observation(stop_observed=True, grasp_still_observed=False)
    return dict(physical_grasp_before_cancel=prior,
                current_hold_verification="not revalidated after the native physics clock was held")


def failed_stop_hold_observation(lifecycle, *, stop_observed):
    """Keep the historical lift fact separate from current holding after stop."""
    _require(stop_observed is True, "native stop must be observed before failed hold is finalized")
    prior = lifecycle.physical_grasp
    lifecycle.failed_stop_observation(stop_observed=True)
    return dict(physical_grasp_before_stop=prior,
                current_hold_verification=("not revalidated after the native physics clock was held"
                    if prior == "confirmed" else "prior physical state retained; native physics is held"))


def admitted_action_submission(check_abort, prepare, check_identity, submit):
    """Publish preparation before a final cancel/identity gate and one send.

    The final abort check is immediately adjacent to submit. No save callback,
    filesystem read or RPC may be inserted between them by this owner.
    """
    check_abort()
    prepare()
    check_abort()
    check_identity()
    check_abort()
    return submit()


def validate_dispatch_geometry(value, *, anchor_arms, anchor_fingers,
                               world_block_pose=None, held_pose=None):
    """Reject a path whose full-state validation anchor no longer applies."""
    _require(max(abs(value["arm_positions_rad"][name]-position) for name, position in anchor_arms.items()) <= .01,
             "native arm anchor changed before dispatch")
    _require(max(abs(value["finger_positions_m"][name]-position) for name, position in anchor_fingers.items()) <= .003,
             "native finger anchor changed before dispatch")
    if world_block_pose is not None:
        position, angle = _pose_error(_observed_pose(value["block"]), world_block_pose)
        _require(position <= .005 and angle <= .15, "free block moved after path validation")
    if held_pose is not None:
        relative = (tuple(held_pose[:3]), tuple(held_pose[3:]))
        evidence = contact_evidence(value, relative)
        _require(evidence["bilateral_contact"] and evidence["relative_position_error_m"] <= .005
                 and evidence["relative_angle_error_rad"] <= .15,
                 "held block/contact changed after path validation")


def diagnostic_hold_duration(value):
    _require(type(value) in (int, float) and math.isfinite(value) and 0 <= value <= 60,
             "diagnostic hold must be explicitly selected within 0..60 simulation seconds")
    return float(value)


def _native_exit_proof(log):
    values = re.findall(r"\[INFO\] \[(ros2_control_node-\d+)\]: process started with pid \[(\d+)\]", log)
    if len(values) != 1:
        return dict(observed=False, reason="native child start is missing or ambiguous")
    name, pid = values[0]
    marker = f"[{name}]: process has finished cleanly [pid {pid}]"
    return dict(observed=marker in log, node=name, pid=int(pid), clean_exit_marker=marker)


def expected_startup_spawner_interruptions(log, *, startup_cancel, launch_was_alive, native_stop_observed):
    """Recognize only owned startup helpers interrupted by our orderly SIGINT.

    Python controller-manager spawners can report KeyboardInterrupt/-SIGINT
    after launch sends their stop signal. This is not a native control-node
    crash. Raw failure rows remain in the receipt; all other exits still fail.
    """
    if not (startup_cancel and launch_was_alive and native_stop_observed):
        return []
    expected = []
    commands = {"spawner-3": "joint_state_broadcaster", "spawner-4": "left_controller",
                "spawner-5": "right_controller", "spawner-6": "gripper_controller"}
    for row in process_failures(log):
        name, pid = row["node"], row["pid"]
        if row["exit_code"] != -signal.SIGINT or name not in commands:
            continue
        start = f"[INFO] [{name}]: process started with pid [{pid}]"
        stopped = f"[INFO] [{name}]: sending signal 'SIGINT' to process[{name}]"
        command = ("/opt/ros/humble/lib/controller_manager/spawner "+commands[name]+
                   " -c /controller_manager --controller-manager-timeout 30 --ros-args")
        failure = f"[ERROR] [{name}]: process has died [pid {pid}, exit code {-signal.SIGINT}, cmd '{command}']."
        starts = re.findall(r"^\[INFO\] \["+re.escape(name)+r"\]: process started with pid \[(\d+)\]$", log, re.MULTILINE)
        # An ambiguous PID/name, changed command, duplicate failure or an exit
        # before our signal is ineligible, even during an intended cancellation.
        if (starts == [str(pid)] and log.splitlines().count(stopped) == 1
                and log.splitlines().count(failure) == 1
                and sum(item["node"] == name for item in process_failures(log)) == 1
                and log.index(start) < log.index(stopped) < log.index(failure)):
            expected.append(dict(row, controller=commands[name], reason="owned startup spawner interrupted by launch SIGINT"))
    return expected


def run_ros_grasp(target_directory, manifest_sha256, output, *, domain, cancel_event=None,
                  on_update=None, run_id=None, diagnostic_hold_sec=0):
    """Run one fresh fixed task and retain originals, including any failure.

    ``on_update`` receives detached dictionaries with ``kind`` (task or
    observation), run/model identity and a monotonic event sequence.  It runs on
    the ROS spin thread and must return promptly.  ``cancel_event`` requests a
    native three-controller stop and never opens/detaches/replays a task.
    """
    _require(type(domain) is int and 0 <= domain <= 232, "select an independent ROS domain 0..232")
    diagnostic_hold_sec = diagnostic_hold_duration(diagnostic_hold_sec)
    _require(type(manifest_sha256) is str and re.fullmatch(r"[0-9a-f]{64}", manifest_sha256),
             "select an explicit SHA-paired grasp target")
    identifier = uuid.uuid4().hex if run_id is None else run_id
    _require(type(identifier) is str and re.fullmatch(r"[0-9a-f]{32}", identifier), "invalid grasp run ID")
    output = Path(output).resolve()
    _require(output.is_relative_to((ROOT/"runtime").resolve()), "task output must stay under this worktree runtime/")
    _require(not output.exists(), "select a new task output directory; prior evidence is preserved")
    output.mkdir(parents=True, mode=0o700)
    result_path, log_path = output/"result.json", output/"launch.log"
    log_path.touch()
    cancel_event = cancel_event if cancel_event is not None else threading.Event()
    record = dict(schema_version=1, kind="ros-grasp-task-v1", run_id=identifier, domain=domain,
                  status="running", stage="preflight", started_utc=utc_now(), event_sequence=0,
                  truth_source="native MuJoCo 3.12 observation; ROS2-control actions; confirmed MoveIt planning scene",
                  automatic_retry=False, physical_grasp="unconfirmed", planning_attachment="uninitialized",
                  action_terminal=None, cleanup=dict(status="not_started", clean=False),
                  checks=[], actions=[], cleanup_errors=[], simulation_only=True)
    record["diagnostic_hold"] = dict(requested_simulation_sec=diagnostic_hold_sec,
        enabled=diagnostic_hold_sec > 0, production_default_sec=0., gui_option=False,
        reference_policy="original closed-grasp reference is retained; no tolerance or contact gate change")
    process = node = context = executor = target = evaluator = scene = life = frames = receipts = log_stream = timer = publisher = None
    identity_worker = None
    identity_paths = ()
    clients, pending, handles = {}, [], []
    phase = [0]
    store = None
    latest = dict(observation=None, observed_monotonic_ns=None, clock_progress_observed_monotonic_ns=None,
                  guard=None, guard_observed_monotonic_ns=None, accepted_context=None, error=None)
    ready = False
    baseline_right = None
    held_reference = None
    monitor_held = False
    start_wall = time.monotonic()
    ros = None
    source_hashes = {}
    input_hashes = {}

    def snapshot():
        record["action_terminal"] = latest_action_terminal(record["actions"])
        if life is not None:
            values = life.snapshot()
            values["attachment_authority"] = "confirmed MoveIt scene; physical grasp remains separately observed"
            record["lifecycle"] = values
            record["physical_grasp"] = life.physical_grasp
        if scene is not None:
            values = scene.snapshot()
            record["planning_scene"] = values
            record["planning_attachment"] = values["planning_attachment"]
        return deepcopy(record)

    def emit(kind="task", observation=None):
        record["event_sequence"] += 1
        saved = snapshot()
        public = {key: saved[key] for key in ("status", "stage", "physical_grasp", "planning_attachment",
            "action_terminal", "cleanup", "cleanup_errors", "simulation_only")}
        for key in ("error", "failure_stage", "lifecycle", "planning_scene", "physical_grasp_before_cancel", "physical_grasp_before_stop",
                    "current_hold_verification"):
            if key in saved:
                public[key] = saved[key]
        for key in ("owned_process_group", "owned_validation_process_group"):
            if key in saved:
                public[key] = saved[key]
        public.update(phase=record["stage"], native_phase=phase[0],
                      actions=deepcopy(record["actions"][-8:]), check_count=len(record["checks"]))
        native = observation
        if native is None and latest["observation"] is not None:
            native = latest["observation"].payload
        value = dict(schema_version=1, kind=kind, run_id=identifier,
                     model_sha256=record.get("model_sha256"), plugin_sha256=record.get("plugin_sha256"),
                     sequence=record["event_sequence"], event_sequence=record["event_sequence"],
                     observed_monotonic_ns=latest["observed_monotonic_ns"], native_observation=deepcopy(native),
                     clock_progress_observed_monotonic_ns=latest["clock_progress_observed_monotonic_ns"],
                     native_clock_has_advanced=(latest["observation"].clock_has_advanced
                                                if latest["observation"] is not None else False),
                     native_guard=deepcopy(latest["guard"]),
                     guard_observed_monotonic_ns=latest["guard_observed_monotonic_ns"],
                     truth_source=record["truth_source"], task=public)
        if observation is not None:
            value["observation"] = deepcopy(observation)
        if on_update is not None:
            on_update(value)

    def save():
        atomic_json(result_path, snapshot())
        emit()

    def admission_context():
        now = time.monotonic_ns()
        observed = latest["accepted_context"]
        if observed is None:
            return dict(now_monotonic_ns=now, native_sequence=None)
        return dict(now_monotonic_ns=now, **observed,
            capture_age_sec=(now-observed["capture_monotonic_ns"])/1e9,
            receipt_age_sec=(now-observed["received_monotonic_ns"])/1e9)

    def timed_admission(admission, label, work):
        row = dict(stage=label, before=admission_context())
        admission["stages"].append(row)
        started = time.monotonic()
        try:
            return work()
        except BaseException as error:
            row["error"] = dict(type=type(error).__name__, message=str(error)[:512])
            raise
        finally:
            row.update(elapsed_sec=time.monotonic()-started, after=admission_context())

    def stage(name):
        record["stage"] = name
        save()

    def publish_phase():
        if publisher is not None:
            publisher.publish(types.String(data=identifier+":"+str(phase[0])))

    def change_phase(value):
        phase[0] = value
        publish_phase()

    def current(*, physical=True):
        _require(store is not None, "native observation store is not ready")
        return store.snapshot(now_sec=time.monotonic(), require_physical=physical)

    def abort():
        if cancel_event.is_set():
            change_phase(8)
            raise RosGraspCancelled("operator requested cancellation; no automatic release or resume")
        _require(time.monotonic()-start_wall <= MAX_TASK_WALL_SEC, "grasp task wall-time budget exceeded")
        _require(process is not None and process.poll() is None, "owned ROS launch exited unexpectedly")
        _require(latest["error"] is None, "invalid native ROS input: "+str(latest["error"]))
        if ready:
            value = current().payload
            if baseline_right is not None:
                _require(max(abs(value["arm_positions_rad"][name]-position) for name, position in baseline_right.items()) <= .015,
                         "non-selected UR5e arm moved beyond its hold tolerance")
            if monitor_held:
                evidence = contact_evidence(value, held_reference)
                _require(evidence["bilateral_contact"] and evidence["relative_position_error_m"] <= .005
                         and evidence["relative_angle_error_rad"] <= .15, "physical grasp slipped or lost bilateral contact")

    def spin(*, health=True):
        if health:
            abort()
        elif record["cleanup"]["status"] == "not_started":
            # Startup waits have no ready guard yet, but cancellation and a
            # dead owned launcher must still be handled promptly.
            if cancel_event.is_set():
                change_phase(8)
                raise RosGraspCancelled("operator cancelled during native startup")
            if process is not None:
                _require(process.poll() is None, "owned ROS launch exited during startup")
        executor.spin_once(timeout_sec=.01)
        if health:
            abort()

    def until(predicate, seconds, label, *, health=True):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            spin(health=health)
            value = predicate()
            if value:
                return value
        raise TimeoutError(label)

    def wait_result(future, seconds, label):
        until(future.done, seconds, label)
        value = future.result()
        _require(value is not None, "empty RPC result: "+label)
        return value

    def check_file_identity():
        _require(identity_worker is not None, "read-only admission worker is not ready")
        proof = identity_worker.check(pump=spin, abort=abort)
        verify_identity_proof(proof, manifest_sha256=manifest_sha256, expected_hashes=input_hashes, inventory=identity_paths)
        return proof

    def scene_mutation_guard():
        abort()
        proof = check_file_identity()
        if phase[0] == 6:
            evidence = contact_evidence(current().payload)
            _require(not evidence["bilateral_contact"] and .06 <= evidence["open_gap_m"] <= .08,
                     "physical release changed before MoveIt detachment")
        verify_identity_proof(proof, manifest_sha256=manifest_sha256, expected_hashes=input_hashes, inventory=identity_paths)
        abort()

    def scene_observation():
        abort()
        return current()

    def window(duration, *, predicate=None, timeout=8.):
        return collect_evidence_window(duration, spin=spin,
            current_evidence=lambda: contact_evidence(current().payload, held_reference), predicate=predicate, timeout=timeout)

    def send_action(name, trajectory, *, gripper=False, close=False, expected_scene=None, geometry_guard=None):
        client = clients["gripper" if gripper else "left"]
        goal = types.FollowJointTrajectory.Goal()
        goal.trajectory = deepcopy(trajectory)
        tolerance = .02 if gripper and close else .0015 if gripper else .005
        goal.goal_tolerance = [types.JointTolerance(name=joint, position=tolerance) for joint in trajectory.joint_names]
        goal.path_tolerance = [types.JointTolerance(name=joint, position=.04 if gripper else .03) for joint in trajectory.joint_names]
        goal.goal_time_tolerance = types.Duration(sec=3)
        action = dict(stage=name, controller="gripper_controller" if gripper else "left_controller",
                      sent_monotonic_sec=None, submission_started=False, accepted=None, terminal_observed=False,
                      physical_grasp_claim=False, goal_tolerance=tolerance, units="m" if gripper else "rad")
        admission = dict(schema_version=1, stages=[], goal_submission_count=0, automatic_retry=False)
        action["admission"] = admission
        def prepare():
            record["actions"].append(action)
            timed_admission(admission, "prepare_checkpoint", save)
        def identity():
            proof = timed_admission(admission, "target_file_binary_hash", check_file_identity)
            if expected_scene is not None:
                def confirm_scene():
                    _require(scene.review_identity() == expected_scene, "MoveIt scene changed before action submission")
                timed_admission(admission, "scene_identity", confirm_scene)
            if geometry_guard is not None:
                timed_admission(admission, "current_geometry", geometry_guard)
            timed_admission(admission, "final_file_stat", lambda: verify_identity_proof(proof,
                manifest_sha256=manifest_sha256, expected_hashes=input_hashes, inventory=identity_paths))
        def submit():
            _require(admission["goal_submission_count"] == 0, "an admission can submit only one goal")
            admission["goal_submission_count"] += 1
            action.update(submission_started=True, sent_monotonic_sec=time.monotonic())
            try:
                future = client.send_goal_async(goal)
            except BaseException:
                action["submission_uncertain"] = True
                raise
            pending.append((future, action))
            return future
        future = admitted_action_submission(lambda: timed_admission(admission, "current_guard_cancel", abort),
            prepare, identity, lambda: timed_admission(admission, "one_goal_submit", submit))
        handle = wait_result(future, 5., "grasp action acceptance")
        action["accepted"] = bool(handle.accepted)
        _require(handle.accepted, "trajectory action was rejected")
        action["goal_id"] = bytes(handle.goal_id.uuid).hex()
        result_future = handle.get_result_async()
        handles.append((handle, result_future, action))
        answer = wait_result(result_future, 42., "grasp trajectory terminal")
        terminal_receipt = time.monotonic()
        action.update(terminal_observed=True, terminal_status=answer.status,
                      terminal_error_code=answer.result.error_code,
                      terminal_received_monotonic_sec=terminal_receipt)
        record["action_terminal"] = dict(stage=name, status=answer.status, error_code=answer.result.error_code,
                                          physical_grasp_claim=False)
        save()
        _require(answer.status == 4 and answer.result.error_code == 0, "trajectory action did not succeed")
        until(lambda: latest["observation"] and latest["observation"].receipt_sec > terminal_receipt,
              3., "post-terminal native observation")
        return current()

    def gripper(name, target_m, *, close=False):
        _require(type(target_m) in (int, float) and math.isfinite(target_m) and 0 <= target_m <= .034,
                 "gripper commands must remain within the declared 0..0.034 m range")
        trajectory = types.JointTrajectory()
        trajectory.joint_names = list(FINGER_JOINTS)
        trajectory.points = [types.JointTrajectoryPoint(positions=[target_m, target_m],
                                                        time_from_start=types.Duration(sec=1))]
        anchor = current().payload
        world = _observed_pose(anchor["block"]) if close else None
        held = [*held_reference[0], *held_reference[1]] if not close and held_reference is not None else None
        def geometry():
            observed = current()
            validate_dispatch_geometry(observed.payload, anchor_arms=anchor["arm_positions_rad"],
                anchor_fingers=anchor["finger_positions_m"], world_block_pose=world, held_pose=held)
            if close:
                _require(scene.block_drift(observed)["within_tolerance"], "free block differs from the planning scene before close")
        return send_action(name, trajectory, gripper=True, close=close,
                           expected_scene=scene.review_identity(), geometry_guard=geometry)

    def motion(name, position, quaternion, *, held_pose=None):
        stage(name)
        observed = current()
        value = observed.payload
        initial = dict(value["arm_positions_rad"], **value["finger_positions_m"])
        initial_arms = dict(value["arm_positions_rad"])
        goal_pose = dict(frame_id="world", position_m=list(position),
                         quaternion_xyzw=[*quaternion[1:], quaternion[0]])
        scene_before = scene.review_identity()
        fk_request = types.GetPositionFK.Request()
        fk_request.header.frame_id = "world"
        fk_request.fk_link_names = [target.manifest["tool"]["tcp_frame"], target.manifest["tool"]["parent_tip"]]
        fk_request.robot_state.is_diff = False
        fk_request.robot_state.joint_state.name = list(target.allowed_scalar_joints)
        fk_request.robot_state.joint_state.position = [initial[name] for name in target.allowed_scalar_joints]
        fk_started = time.monotonic()
        observed_fk = wait_result(fk_client.call_async(fk_request), 2., "raw observed native/FK diagnostic")
        _require(observed_fk.error_code.val == 1 and list(observed_fk.fk_link_names) == fk_request.fk_link_names
                 and len(observed_fk.pose_stamped) == 2, "raw observed FK diagnostic did not return the declared TCP/tip")
        stamped = observed_fk.pose_stamped[0]
        _require(stamped.header.frame_id == "world", "raw observed FK diagnostic frame differs")
        fp, fq = stamped.pose.position, stamped.pose.orientation
        fk_native_pose = ((fp.x, fp.y, fp.z), (fq.w, fq.x, fq.y, fq.z))
        native_fk_error = _pose_error(fk_native_pose, _observed_pose(value["tcp"]))
        atomic_json(output/(name+"-native-moveit-fk.json"), dict(
            native_tcp=value["tcp"], moveit_fk=types.message_to_ordereddict(observed_fk),
            raw_scalar_seed=initial, no_seed_clamping=True, fk_wall_sec=time.monotonic()-fk_started,
            configured_tcp_frame=target.manifest["tool"]["tcp_frame"],
            position_error_m=native_fk_error[0], angle_error_rad=native_fk_error[1],
            observed_scalar_joint_units=target.description()["scalar_joint_units"]))
        _require(native_fk_error[0] <= .001 and native_fk_error[1] <= .005,
                 "native MuJoCo and MoveIt TCP frames disagree")
        if phase[0] == 1:
            _require(scene.block_drift(observed)["within_tolerance"], "free block moved since its scene anchor")
        try:
            target_state, ik = pose_adapter.solve_ik("left_arm", target.manifest["tool"]["tcp_frame"],
                goal_pose, initial, target.joints[:6], timeout_sec=2., rpc_timeout_sec=3.)
        except PoseAdapterError:
            # Diagnostic only: a collision-unchecked IK result is never adopted,
            # planned or dispatched. Keep the original checked-IK failure.
            diagnosis = dict(diagnostic_only=True, dispatch_authorized=False, target_pose=goal_pose)
            try:
                request = types.GetPositionIK.Request()
                ik_request = request.ik_request
                ik_request.group_name = "left_arm"
                ik_request.ik_link_name = target.manifest["tool"]["tcp_frame"]
                ik_request.avoid_collisions = False
                ik_request.robot_state.is_diff = False
                ik_request.robot_state.joint_state.name = list(target.allowed_scalar_joints)
                ik_request.robot_state.joint_state.position = [initial[joint] for joint in target.allowed_scalar_joints]
                ik_request.pose_stamped.header.frame_id = "world"
                ik_request.pose_stamped.pose.position.x, ik_request.pose_stamped.pose.position.y, ik_request.pose_stamped.pose.position.z = position
                (ik_request.pose_stamped.pose.orientation.w, ik_request.pose_stamped.pose.orientation.x,
                 ik_request.pose_stamped.pose.orientation.y, ik_request.pose_stamped.pose.orientation.z) = quaternion
                ik_request.timeout.sec = 2
                answer = wait_result(ik_client.call_async(request), 3., "diagnostic unchecked IK")
                diagnosis["ik_error_code"] = answer.error_code.val
                diagnosis["solution"] = types.message_to_ordereddict(answer.solution)
                if answer.error_code.val == 1:
                    diagnosis["validity"] = []
                    for group in ("left_arm", ""):
                        check = types.GetStateValidity.Request()
                        check.robot_state = answer.solution
                        check.group_name = group
                        checked = wait_result(validity_client.call_async(check), 3., "diagnostic state validity")
                        diagnosis["validity"].append(dict(group=group,
                            response=types.message_to_ordereddict(checked)))
            except Exception as error:
                diagnosis["diagnostic_error"] = str(error)
            atomic_json(output/(name+"-ik-failure-diagnostic.json"), diagnosis)
            raise
        fk_goal, fk_timing = pose_adapter.forward_kinematics(target.manifest["tool"]["tcp_frame"], target_state, timeout_sec=2.)
        fk_pose = (tuple(fk_goal["position_m"]), tuple([fk_goal["quaternion_xyzw"][3], *fk_goal["quaternion_xyzw"][:3]]))
        errors = _pose_error(fk_pose, (tuple(position), tuple(quaternion)))
        _require(errors[0] <= .001 and errors[1] <= .005, "IK/FK candidate does not reach the requested TCP")
        response, planning = planning_adapter.plan_joint_target("left_arm",
            {joint: target_state[joint] for joint in target.joints[:6]}, initial,
            allowed_planning_sec=4., goal_tolerance_rad=.0005)
        _require(response.error_code.val == 1, "MoveIt rejected the grasp motion plan: "+str(response.error_code.val))
        trajectory = deepcopy(response.trajectory.joint_trajectory)
        curve, samples = trajectory_samples(trajectory, initial_arms, target.joints[:6], target.manifest["arm_joint_limits"])
        minimum = None
        validation_started = time.monotonic()
        for index, (_, state) in enumerate(samples):
            if index % 8 == 0:
                spin()
            clearance = evaluator.at(observed.qpos, phase=phase[0], arm_state=state, held_relative_pose=held_pose)
            _require(clearance["distance_m"] >= .0005, "native spline clearance is insufficient: "+str(clearance))
            if minimum is None or clearance["distance_m"] < minimum["distance_m"]:
                minimum = clearance
        now = current()
        anchor = now.payload
        _require(max(abs(anchor["arm_positions_rad"][joint]-initial_arms[joint]) for joint in target.joints) <= .01,
                 "native arm anchor drifted while planning")
        _require(max(abs(anchor["finger_positions_m"][joint]-value["finger_positions_m"][joint]) for joint in FINGER_JOINTS) <= .003,
                 "native gripper anchor drifted while planning")
        _require(scene.review_identity() == scene_before, "MoveIt scene changed between planning and dispatch")
        record["checks"].append(dict(name=name+"_plan", ik=ik, fk=fk_timing, planning=planning,
            sample_count=len(samples), duration_sec=curve.duration, minimum_clearance=minimum,
            validation_wall_sec=time.monotonic()-validation_started,
            spline=curve.record(), scene_identity=scene_before,
            selected_arm="left", other_arm="native observed hold; no right-controller goal",
            dispatch_policy="unchanged MoveIt trajectory, same JTC spline derivatives"))
        atomic_json(output/(name+"-plan.json"), record["checks"][-1])
        save()
        def geometry():
            observed_now = current()
            validate_dispatch_geometry(observed_now.payload, anchor_arms=initial_arms,
                anchor_fingers=value["finger_positions_m"],
                world_block_pose=_observed_pose(value["block"]) if phase[0] == 1 else None, held_pose=held_pose)
            if phase[0] == 1:
                _require(scene.block_drift(observed_now)["within_tolerance"], "free block differs from the planning scene before motion")
        completed = send_action(name, trajectory, expected_scene=scene_before, geometry_guard=geometry)
        actual = _observed_pose(completed.payload["tcp"])
        errors = _pose_error(actual, (tuple(position), tuple(quaternion)))
        _require(errors[0] <= .006 and errors[1] <= .04, "Action terminal did not establish TCP completion")
        return completed

    save()
    try:
        base = load_workcell("fr5_ur5e")
        target = load_ros_grasp_target(base, target_directory, manifest_sha256)
        _require(target is not None, "an explicit native grasp target is required")
        check_grasp_runtime(ROOT)
        evaluator = GraspDistanceEvaluator(target)
        record.update(model_sha256=evaluator.model_sha256, execution_manifest_sha256=manifest_sha256,
                      target_fingerprint=target.fingerprint, clearance=evaluator.metadata(),
                      plugin_sha256=file_sha256(ROOT/"native/grasp_ros2_plugin/src/observation.cpp"))
        source_paths = [Path(__file__), *[Path(__file__).with_name(name) for name in
            ("ros_observation.py", "ros_planning_scene.py", "ros_distance.py", "ros_admission.py", "contract.py")],
            *[ROOT/"src/mujoco_ros2_manipulation/workcells"/name for name in
              ("pose_adapter.py", "planning_adapter.py", "interference.py")],
            ROOT/"native/grasp_ros2_plugin/src/observation.cpp", ROOT/"native/grasp_policy.hpp",
            ROOT/"native/grasp_ros2_plugin/src/ordered_fifo.hpp",
            ROOT/"native/grasp_distance.cpp", ROOT/"native/step_monitor.hpp", ROOT/"native/control_loop_shutdown.hpp"]
        source_hashes = {str(path): file_sha256(path) for path in source_paths}
        record["source_sha256"] = source_hashes
        binaries = [ROOT/"runtime/native/grasp_distance.so",
            ROOT/"ros_ws/install/manipulation_grasp_plugin/lib/libmujoco_ros2_control_grasp_observer.so",
            ROOT/"ros_ws/install/mujoco_ros2_control/lib/libmujoco_ros2_control.so",
            ROOT/"ros_ws/install/mujoco_ros2_control/lib/mujoco_ros2_control/ros2_control_node"]
        record["runtime_binary_sha256"] = {str(path): file_sha256(path) for path in binaries}
        input_hashes = dict(source_hashes, **record["runtime_binary_sha256"])
        identity_paths = identity_inventory(target, input_hashes)
        identity_worker = ReadOnlyAdmissionWorker(dict(target_directory=str(target.output),
            manifest_sha256=manifest_sha256, input_hashes=input_hashes, inventory=list(identity_paths)),
            output/"identity-worker")
        record["owned_validation_process_group"] = identity_worker.process.pid
        save()  # publish ownership before any request can start target/Git work
        store = GraspObservationStore(run_id=identifier, model_sha256=evaluator.model_sha256,
            plugin_sha256=file_sha256(ROOT/"native/grasp_ros2_plugin/src/observation.cpp"),
            fifo_source_sha256=file_sha256(ROOT/"native/grasp_ros2_plugin/src/ordered_fifo.hpp"),
            manifest=target.manifest, geom_names=evaluator.geom_names())

        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTolerance
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject, CollisionObject,
                                     Constraints, JointConstraint, PlanningSceneComponents)
        from moveit_msgs.srv import (ApplyPlanningScene, GetMotionPlan, GetPlanningScene,
                                    GetPositionFK, GetPositionIK, GetStateValidity)
        from rosidl_runtime_py.convert import message_to_ordereddict
        from shape_msgs.msg import SolidPrimitive
        from std_msgs.msg import String
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from ..workcells.verify_motion_reproduction import require_unused_domain

        types = SimpleNamespace(**{key: value for key, value in locals().items() if key in (
            "String", "Duration", "FollowJointTrajectory", "JointTolerance", "Pose", "AllowedCollisionEntry",
            "AttachedCollisionObject", "CollisionObject", "Constraints", "JointConstraint", "PlanningSceneComponents",
            "ApplyPlanningScene", "GetMotionPlan", "GetPlanningScene", "GetPositionFK", "GetPositionIK", "GetStateValidity",
            "message_to_ordereddict", "SolidPrimitive", "JointTrajectory", "JointTrajectoryPoint")})
        ros = rclpy
        stage("unused_domain")
        require_unused_domain(domain)
        environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), MANIPULATION_ROS_DOMAIN_ID=str(domain),
                           MANIPULATION_DIAGNOSTICS_PATH=str(output/"physics.json"),
                           MANIPULATION_DIAGNOSTICS_RUN_ID=identifier)
        context = Context()
        ros.init(context=context, domain_id=domain)
        node = Node("native_ros_grasp_task_"+identifier[:8], context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        guard_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        frames = (output/"native-observations.jsonl").open("wb")
        receipts = (output/"native-receipts.jsonl").open("wb")
        recorder = AcceptedObservationRecorder(frames, receipts, max_frame_bytes=MAX_FRAME_BYTES, max_receipt_bytes=MAX_RECEIPT_BYTES)

        def observation_callback(message):
            received_ns = time.monotonic_ns()
            accepted = store.ingest(message.data, receipt_sec=received_ns*1e-9)
            if not accepted.accepted:
                if "duplicate or reordered" not in accepted.reason:
                    latest["error"] = accepted.reason
                return
            value = json.loads(message.data)  # already schema/identity checked
            try:
                observation = recorder.record(value, received_ns=received_ns,
                    eligible_snapshot=lambda: store.snapshot(now_sec=time.monotonic()))
            finally:
                latest["accepted_context"] = recorder.last_context
            if observation is None:
                # Raw accepted frame and its original receipt remain recorded;
                # ineligible or stopped data never becomes physical authority.
                return
            previous = latest["observation"]
            if previous is None or observation.time_sec > previous.time_sec:
                latest["clock_progress_observed_monotonic_ns"] = received_ns
            latest["observation"] = observation
            latest["observed_monotonic_ns"] = received_ns
            emit("observation", value)

        def guard_callback(message):
            received_ns = time.monotonic_ns()
            accepted = store.update_guard(message.data, receipt_sec=received_ns*1e-9)
            if not accepted.accepted:
                # During controller spawning the guard intentionally publishes
                # partial inventories. They remain ineligible for physical use.
                if ready:
                    latest["error"] = accepted.reason
            else:
                latest["guard"] = json.loads(message.data)
                latest["guard_observed_monotonic_ns"] = received_ns

        observation_qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(String, "/workcell/grasp_observation", observation_callback, observation_qos)
        node.create_subscription(String, "/workcell/contact_stop", guard_callback, guard_qos)
        publisher = node.create_publisher(String, "/workcell/grasp_phase", qos)
        timer = node.create_timer(.05, publish_phase)
        clients = {name: ActionClient(node, FollowJointTrajectory, "/"+controller+"/follow_joint_trajectory")
                   for name, controller in (("left", "left_controller"), ("gripper", "gripper_controller"))}
        ik_client = node.create_client(GetPositionIK, "/compute_ik")
        fk_client = node.create_client(GetPositionFK, "/compute_fk")
        plan_client = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        get_scene = node.create_client(GetPlanningScene, "/get_planning_scene")
        apply_scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        validity_client = node.create_client(GetStateValidity, "/check_state_validity")
        limits = kinematics_state_limits(target.manifest)
        record["kinematics_seed_policy"] = dict(finger_state_numeric_margin_m=1e-6,
            raw_observation_preserved=True, seed_clamping=False, commanded_finger_range_m=[0., .034],
            ik_selected_joint_names=list(target.joints[:6]), unselected_ik_values_never_commanded=True)
        pose_adapter = MoveItPoseAdapter(ik_client, fk_client, wait_result, GetPositionIK.Request,
            GetPositionFK.Request, target.allowed_scalar_joints, limits, passthrough_exceptions=(RosGraspTaskError,))
        planning_adapter = MoveItPlanningAdapter(plan_client, wait_result, GetMotionPlan.Request,
            Constraints, JointConstraint, target.allowed_scalar_joints)
        scene = RosGraspPlanningScene(get_scene, apply_scene, wait_result, types, manifest=target.manifest,
            run_id=identifier, model_sha256=evaluator.model_sha256,
            on_evidence=lambda name, value: atomic_json(output/("planning-scene-"+name+".json"), value),
            fresh_observation=scene_observation, pre_mutation_guard=scene_mutation_guard)
        life = GraspLifecycle(evaluator.model_sha256, object_id=scene.object_id,
                              owner=target.manifest["tool"]["id"], link=scene.link_name)
        command = ["bash", str(ROOT/"run_scene.sh"), "workcell:=fr5_ur5e", "gui:=false", "rviz:=false",
                   "execution_environment:="+str(target.output), "execution_manifest_sha256:="+manifest_sha256,
                   "grasp_run_id:="+identifier]
        if cancel_event.is_set():
            raise RosGraspCancelled("operator cancelled before native launch")
        stage("launch")
        log_stream = log_path.open("wb")
        process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log_stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        record["owned_process_group"] = process.pid
        save()
        until(lambda: latest["observation"] is not None, 60., "native observation", health=False)
        until(lambda: latest["guard"] and latest["guard"].get("phase") == "ready"
              and not latest["guard"]["latched"]
              and all(latest["guard"]["controllers"].get(name) == "active" for name in CONTROLLERS),
              60., "three ready native controllers", health=False)
        change_phase(1)
        until(lambda: latest["observation"].payload["armed"]
              and latest["observation"].clock_has_advanced, 3., "armed progressing native state", health=False)
        ready = True
        abort()
        until(lambda: all(client.server_is_ready() for client in clients.values()) and all(
            client.service_is_ready() for client in (ik_client, fk_client, plan_client, get_scene, apply_scene)),
            30., "MoveIt and Action servers")
        stage("initial_settle")
        window(.25)
        first = current()
        baseline_right = {name: first.payload["arm_positions_rad"][name] for name in target.joints[6:]}
        original_block = list(first.payload["block"]["position"])
        orientation = tuple(first.payload["tcp"]["quaternion_wxyz"])
        scene.initialize(first)
        save()
        motion("approach", original_block, orientation)
        life.approach_complete()
        change_phase(2)
        until(lambda: current().payload["phase"] == 2, 2., "closing phase observation")
        scene.allow_finger_contact(current())
        stage("close")
        gripper("close", target.manifest["tool"]["close_target_m"], close=True)
        closed = window(.12, predicate=lambda sample: sample["bilateral_contact"])
        life.contacts_confirmed(left_force_n=min(item["finger_force_n"][0] for item in closed),
            right_force_n=min(item["finger_force_n"][1] for item in closed),
            observed_sec=checked_window(closed, .1))
        current_value = current().payload
        held_reference = _relative(_observed_pose(current_value["tcp"]), _observed_pose(current_value["block"]))
        held_pose = [*held_reference[0], *held_reference[1]]
        change_phase(3)
        lift_target = list(current_value["tcp"]["position"])
        lift_target[2] += .08
        motion("verification_lift", lift_target, orientation, held_pose=held_pose)
        stable = window(.25)
        evidence = dict(fixture_sha256=evaluator.model_sha256,
            lift_m=min(item["block_position_m"][2]-original_block[2] for item in stable),
            stable_sec=checked_window(stable, .2),
            relative_position_error_m=max(item["relative_position_error_m"] for item in stable),
            relative_angle_error_rad=max(item["relative_angle_error_rad"] for item in stable),
            bilateral_fraction=sum(item["bilateral_contact"] for item in stable)/len(stable))
        life.lift_confirmed(**evidence)
        record["checks"].append(dict(name="physical_grasp_confirmed", evidence=evidence, samples=stable))
        monitor_held = True
        scene.attach(current(), physical_confirmed=True)
        life.attach(object_id=scene.object_id, owner=target.manifest["tool"]["id"], link=scene.link_name,
                    fixture_sha256=evaluator.model_sha256)
        life.carry()
        change_phase(4)
        if diagnostic_hold_sec > 0:
            stage("diagnostic_hold")
            held = window(diagnostic_hold_sec, timeout=diagnostic_hold_sec+10.)
            record["checks"].append(dict(name="diagnostic_held_stability", diagnostic_only=True,
                requested_simulation_sec=diagnostic_hold_sec,
                observed_simulation_sec=checked_window(held, diagnostic_hold_sec), sample_count=len(held),
                relative_position_error_m_max=max(item["relative_position_error_m"] for item in held),
                relative_angle_error_rad_max=max(item["relative_angle_error_rad"] for item in held),
                bilateral_fraction=sum(item["bilateral_contact"] for item in held)/len(held),
                original_closed_reference_retained=True, samples=held))
            save()
        carry_target = list(current().payload["tcp"]["position"])
        carry_target[0] += .06
        motion("transport", carry_target, orientation, held_pose=held_pose)
        change_phase(5)
        block_target = [original_block[0]+.06, original_block[1], original_block[2]+.004]
        relative_offset = _rotate(orientation, held_reference[0])
        place_tcp = [position-offset for position, offset in zip(block_target, relative_offset)]
        motion("place", place_tcp, orientation, held_pose=held_pose)
        life.release()
        monitor_held = False
        change_phase(6)
        stage("open")
        gripper("open", 0.)
        released = window(.25, predicate=lambda item: not item["bilateral_contact"] and .06 <= item["open_gap_m"] <= .08)
        scene.release(current(), physical_released=True)
        life.release_confirmed(fixture_sha256=evaluator.model_sha256,
            bilateral_contact_absent_sec=checked_window(released, .2), open_gap_m=min(item["open_gap_m"] for item in released))
        change_phase(7)
        stage("settle")
        settled = window(.25, predicate=lambda item: item["table_force_n"] >= .2 and item["block_speed_m_s"] <= .02)
        target_position = [original_block[0]+.06, original_block[1], original_block[2]]
        position_error = max(math.sqrt(sum((a-b)**2 for a, b in zip(item["block_position_m"], target_position))) for item in settled)
        life.settled(table_contact_sec=checked_window(settled, .2), position_error_m=position_error,
                     speed_m_s=max(item["block_speed_m_s"] for item in settled))
        record["checks"].append(dict(name="physical_release_and_placement", release_samples=released,
                                     settle_samples=settled, position_error_m=position_error))
        record["status"] = "succeeded"
        save()
    except BaseException as error:
        record.update(status="cancelled" if isinstance(error, RosGraspCancelled) else "failed",
                      failure_stage=record["stage"], error=dict(type=type(error).__name__, message=str(error)))
        record["failure_observation_context"] = admission_context()
        if identity_worker is not None:
            record["validation_worker_at_failure"] = identity_worker.snapshot()
        if life is not None and life.stage not in ("succeeded", "cancelled", "failed"):
            if isinstance(error, RosGraspCancelled):
                life.cancel()
            else:
                life.fail(str(error)[:256] or type(error).__name__, slipped="slip" in str(error))
    finally:
        startup_cancel = bool(not ready and not record["actions"] and cancel_event.is_set()
            and record["status"] == "cancelled" and record.get("error", {}).get("type") == "RosGraspCancelled")
        ready = False
        monitor_held = False
        record["cleanup"] = dict(status="running", clean=False, automatic_release=False, automatic_detach=False)
        try:
            save()
        except Exception as error:
            record["cleanup_errors"].append("checkpoint: "+str(error))
        if process is not None and process.poll() is None and publisher is not None:
            try:
                change_phase(8)
                stopped = until(lambda: latest["guard"] and latest["guard"].get("phase") == "stopped"
                    and latest["guard"]["latched"] and all(latest["guard"]["controllers"].get(name) == "inactive"
                                                           for name in CONTROLLERS),
                    8., "native three-controller cleanup stop", health=False)
                record["cleanup"]["native_stop_observed"] = bool(stopped)
                record["cleanup"]["guard"] = deepcopy(latest["guard"])
                if life is not None and life.stage == "cancelled":
                    record.update(cancelled_hold_observation(life, stop_observed=True))
                elif life is not None and life.stage == "failed":
                    record.update(failed_stop_hold_observation(life, stop_observed=True))
            except Exception as error:
                record["cleanup_errors"].append("native stop: "+str(error))
        # Own any late accepted goal before draining. Native stop came first.
        if node is not None:
            for future, action in pending:
                if any(item[2] is action for item in handles):
                    continue
                try:
                    handle = until(lambda: future.result() if future.done() else None, 2., "late owned acceptance", health=False)
                    action["accepted"] = bool(handle.accepted)
                    if handle.accepted:
                        handles.append((handle, handle.get_result_async(), action))
                except Exception as error:
                    action["submission_uncertain"] = True
                    record["cleanup_errors"].append("late acceptance: "+str(error))
            for handle, future, action in handles:
                if action.get("terminal_observed"):
                    continue
                try:
                    cancellation = handle.cancel_goal_async()
                    until(cancellation.done, 2., "owned action cancel acknowledgement", health=False)
                    action["cancel_acknowledged"] = True
                    until(future.done, 4., "owned action terminal", health=False)
                    answer = future.result()
                    action.update(terminal_observed=True, terminal_status=answer.status,
                                  terminal_error_code=answer.result.error_code)
                except Exception as error:
                    record["cleanup_errors"].append("owned action terminal: "+str(error))
        if timer is not None:
            timer.cancel()
        forced = False
        launch_was_alive_at_stop = process is not None and process.poll() is None
        if process is not None:
            try:
                forced = stop_process(process)
            except BaseException as error:
                record["cleanup_errors"].append("owned launch: "+str(error))
        if log_stream is not None:
            log_stream.close()
        for client in clients.values():
            try:
                client.destroy()
            except Exception as error:
                record["cleanup_errors"].append("action client: "+str(error))
        if executor is not None:
            try:
                executor.shutdown(timeout_sec=1.)
            except Exception as error:
                record["cleanup_errors"].append("ROS executor: "+str(error))
        if node is not None:
            try:
                node.destroy_node()
            except Exception as error:
                record["cleanup_errors"].append("ROS node: "+str(error))
        if context is not None and context.ok():
            try:
                context.shutdown()
            except Exception as error:
                record["cleanup_errors"].append("ROS context: "+str(error))
        if evaluator is not None:
            evaluator.close()
        if frames is not None:
            frames.close()
        if receipts is not None:
            receipts.close()
        log = log_path.read_text(errors="replace")
        failures, native = process_failures(log), _native_exit_proof(log)
        interruptions = expected_startup_spawner_interruptions(log, startup_cancel=startup_cancel,
            launch_was_alive=launch_was_alive_at_stop, native_stop_observed=record["cleanup"].get("native_stop_observed", False))
        expected_keys = {(item["node"], item["pid"], item["exit_code"]) for item in interruptions}
        unexpected = [item for item in failures if (item["node"], item["pid"], item["exit_code"]) not in expected_keys]
        gone = process is None or group_gone(process.pid)
        clean = bool((process is None or (process.returncode == 0 and native["observed"] and gone))
                     and not forced and not unexpected and not record["cleanup_errors"])
        record["cleanup"].update(status="complete", clean=clean, launch_started=process is not None,
            forced=forced, process_group_gone=gone,
            native_exit=native, launcher_return_code=process.returncode if process is not None else None,
            child_process_failures=failures, expected_startup_spawner_interruptions=interruptions,
            unexpected_child_process_failures=unexpected, launch_was_alive_at_stop=launch_was_alive_at_stop)
        if process is not None and not clean:
            record["status"] = "failed"
            record.setdefault("error", dict(type="NativeCleanupFailure", message="clean native shutdown was not observed"))
        try:
            if identity_worker is not None:
                if identity_worker.snapshot()["abandoned"]:
                    record["final_source_validation"] = dict(status="unavailable",
                        reason="validation generation was abandoned; late result cannot authorize a new check")
                else:
                    proof = identity_worker.check(pump=lambda: time.sleep(.005), abort=lambda: None)
                    verify_identity_proof(proof, manifest_sha256=manifest_sha256,
                        expected_hashes=input_hashes, inventory=identity_paths)
                    record["final_source_validation"] = dict(status="passed", generation=proof["generation"])
            else:
                record["final_source_validation"] = dict(status="not_started", reason="native admission was not initialized")
        except Exception as error:
            if record["status"] == "succeeded":
                record["status"] = "failed"
            record["final_identity_error"] = str(error)
            record["final_source_validation"] = dict(status="failed", error=str(error)[:512])
        finally:
            if identity_worker is not None:
                stopped_worker = identity_worker.close()
                record["cleanup"].update(native_process_clean=clean, readonly_validation_worker=stopped_worker)
                if not stopped_worker["quiescent"]:
                    record["cleanup"]["clean"] = False
                    record["status"] = "failed"
                    record["cleanup_errors"].append("read-only validation worker did not become quiescent")
        record.update(stage="complete", completed_utc=utc_now(), wall_sec=time.monotonic()-start_wall,
                      launch_log_sha256=file_sha256(log_path))
        if (output/"native-observations.jsonl").exists():
            record["native_observations_sha256"] = file_sha256(output/"native-observations.jsonl")
        if (output/"native-receipts.jsonl").exists():
            record["native_receipts_sha256"] = file_sha256(output/"native-receipts.jsonl")
        if (output/"physics.json").exists():
            record["physics_sha256"] = file_sha256(output/"physics.json")
        save()
    return snapshot()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--sha", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", help="startup owner's fresh 32-character lowercase run ID")
    parser.add_argument("--updates", type=Path, help="new output/latest.json atomic IPC; never an arbitrary destination")
    parser.add_argument("--diagnostic-hold-sec", type=float, default=0.,
        help="Explicit diagnostic only: hold the confirmed physical grasp for 0..60 simulation seconds; GUI never sets this")
    args = parser.parse_args(argv)
    cancel = threading.Event()
    previous_sigint = signal.getsignal(signal.SIGINT)
    updates = None
    try:
        if args.updates is not None:
            updates = args.updates.resolve()
            _require(updates == args.output.resolve()/"latest.json" and not updates.exists(),
                     "updates must be the new task output/latest.json")
        def update(value):
            raw = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
            _require(len(raw) <= 512*1024, "task IPC update exceeds 512 KiB")
            atomic_json(updates, value)
        signal.signal(signal.SIGINT, lambda *_: cancel.set())
        result = run_ros_grasp(args.target, args.sha, args.output, domain=args.domain, run_id=args.run_id,
            cancel_event=cancel, on_update=update if updates is not None else None,
            diagnostic_hold_sec=args.diagnostic_hold_sec)
        print(result["status"].upper()+": "+str(args.output/"result.json"), flush=True)
        return 0 if result["status"] == "succeeded" else 1
    except Exception as error:
        print(type(error).__name__+": "+str(error), flush=True)
        return 1
    finally:
        signal.signal(signal.SIGINT, previous_sigint)


if __name__ == "__main__":
    raise SystemExit(main())
