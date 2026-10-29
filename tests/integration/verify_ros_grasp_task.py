"""One fresh native ROS grasp task or explicitly owned SIGINT negative case.

The child runs the production CLI, including original observation IPC, checked
MoveIt trajectories and physical lifecycle gates. A transport cancellation is
injected only after its Action is accepted and native TCP/block motion exceeds
5 mm from the lifted pose. No test moves qpos, welds an object, releases after
cancel, retries a failed case, or changes collision permissions. normal_hold
explicitly adds a 30-simulation-second stability diagnostic before transport;
the ordinary GUI task retains its zero-second diagnostic default.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.reports import atomic_json, utc_now
from mujoco_ros2_manipulation.verification.runner import group_gone


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def process_identity(pid):
    try:
        value = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return dict(pid=pid, ppid=int(value[1]), pgrp=int(value[2]), session=int(value[3]), starttime=int(value[19]))
    except (FileNotFoundError, ProcessLookupError):
        return None


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


def verify(args):
    output = args.output.resolve()
    require(output.is_relative_to((ROOT/"runtime").resolve()) and not output.exists(),
            "select a new output under this worktree runtime/")
    require(type(args.domain) is int and 0 <= args.domain <= 232, "select an independent domain 0..232")
    require(re.fullmatch(r"[0-9a-f]{64}", args.sha) is not None, "select an explicit grasp manifest SHA256")
    output.mkdir(parents=True, mode=0o700)
    worker_output = output/"worker"
    run_id = uuid.uuid4().hex
    record = dict(schema_version=1, case=args.case, status="running", started_utc=utc_now(),
                  domain=args.domain, run_id=run_id, automatic_retry=False, signal_sent=False,
                  diagnostic_only_fault_injection="SIGINT to owned worker" if args.case.startswith("cancel_") else None,
                  source_sha256=file_sha256(Path(__file__)), cleanup_errors=[])
    process = None
    inner = None
    stream = None
    previous_sequence = 0
    lift_anchor = None
    latest = None
    receipt = output/"result.json"

    def save():
        atomic_json(receipt, record)

    save()
    try:
        command = [sys.executable, "-m", "mujoco_ros2_manipulation.grasping.ros_task", "--domain", str(args.domain),
                   "--target", str(args.target.resolve()), "--sha", args.sha, "--output", str(worker_output),
                   "--run-id", run_id, "--updates", str(worker_output/"latest.json")]
        if args.case == "normal_hold":
            command += ["--diagnostic-hold-sec", "30"]
        stream = (output/"worker.log").open("wb")
        process = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
                                   env=dict(os.environ, ROS_DOMAIN_ID=str(args.domain), MANIPULATION_ROS_DOMAIN_ID=str(args.domain)))
        record["owned_worker_pid"] = process.pid
        record["command"] = command
        save()
        deadline = time.monotonic()+300.
        while process.poll() is None and time.monotonic() < deadline:
            path = worker_output/"latest.json"
            if path.exists():
                require(path.stat().st_size <= 512*1024, "IPC update exceeds its byte limit")
                value = json.loads(path.read_text())
                require(value.get("schema_version") == 1 and value.get("run_id") == run_id,
                        "worker IPC identity differs")
                sequence = value.get("sequence")
                require(type(sequence) is int and sequence >= previous_sequence, "worker IPC event sequence reversed")
                previous_sequence = sequence
                latest = value
                task = value["task"]
                pgid = task.get("owned_process_group")
                if inner is None and type(pgid) is int:
                    candidate = process_identity(pgid)
                    if candidate and candidate["ppid"] == process.pid and candidate["pgrp"] == candidate["session"] == pgid:
                        inner = candidate
                        record["owned_inner_group_identity"] = inner
                        save()
                native = value.get("native_observation")
                if args.case == "cancel_startup" and inner is not None and not record["signal_sent"]:
                    record.update(signal_sent=True, signal_monotonic_ns=time.monotonic_ns(), signal_boundary="owned native launch created",
                                  signal_before=value)
                    save()
                    process.send_signal(signal.SIGINT)
                elif args.case == "cancel_transport" and not record["signal_sent"] and task.get("phase") == "transport" and native:
                    if lift_anchor is None:
                        lift_anchor = dict(tcp=native["tcp"]["position"], block=native["block"]["position"],
                                           native_sequence=native["sequence"], stamp_ns=native["stamp_ns"])
                        record["lift_anchor"] = lift_anchor
                    actions = task.get("actions", [])
                    moved = max(distance(native["tcp"]["position"], lift_anchor["tcp"]),
                                distance(native["block"]["position"], lift_anchor["block"]))
                    if (actions and actions[-1]["stage"] == "transport" and actions[-1].get("accepted") is True
                            and not actions[-1].get("terminal_observed") and moved >= .005):
                        require(task["physical_grasp"] == "confirmed" and task["planning_attachment"] == "attached",
                                "transport cancellation lacks prior physical/MoveIt confirmation")
                        record.update(signal_sent=True, signal_monotonic_ns=time.monotonic_ns(),
                                      observed_motion_m=moved, signal_boundary="accepted moving transport", signal_before=value)
                        save()
                        process.send_signal(signal.SIGINT)
            time.sleep(.03)
        require(process.poll() is not None, "owned worker exceeded its wall-time budget")
        record["worker_return_code"] = process.returncode
        result_path = worker_output/"result.json"
        require(result_path.is_file(), "worker final report is missing")
        result = json.loads(result_path.read_text())
        require(result.get("kind") == "ros-grasp-task-v1" and result.get("run_id") == run_id,
                "worker result identity differs")
        record["worker_result_sha256"] = file_sha256(result_path)
        record["worker_result"] = result
        require(result["cleanup"]["clean"] and result["cleanup"]["native_exit"]["observed"]
                and result["cleanup"]["process_group_gone"] and not result["cleanup"]["forced"],
                "native child/launcher/group cleanup was not confirmed")
        require(not result["cleanup"]["automatic_release"] and not result["cleanup"]["automatic_detach"],
                "cleanup must never automatically release or detach")
        actions = result["actions"]
        expected_interruptions = result["cleanup"].get("expected_startup_spawner_interruptions", [])
        if expected_interruptions:
            require(args.case == "cancel_startup" and not actions
                    and result["cleanup"].get("launch_was_alive_at_stop") is True
                    and result["cleanup"].get("unexpected_child_process_failures") == []
                    and all(item["exit_code"] == -signal.SIGINT
                            and item["controller"] in ("joint_state_broadcaster", "left_controller", "right_controller", "gripper_controller")
                            for item in expected_interruptions)
                    and result["cleanup"]["child_process_failures"] == [
                        {key: item[key] for key in ("node", "pid", "exit_code")} for item in expected_interruptions],
                    "startup helper interruptions were not narrowly classified and retained")
        stages = [action["stage"] for action in actions]
        checks = {check["name"]: check for check in result["checks"]}
        if args.case in ("normal", "normal_hold"):
            require(process.returncode == 0 and result["status"] == "succeeded", "normal grasp task failed")
            require(stages == ["approach", "close", "verification_lift", "transport", "place", "open"]
                    and all(action.get("terminal_observed") and action.get("terminal_status") == 4
                            and action.get("terminal_error_code") == 0 for action in actions),
                    "normal task did not complete the six actual actions")
            require(result["physical_grasp"] == "released" and result["planning_attachment"] == "world"
                    and result["lifecycle"]["stage"] == "succeeded", "normal final physical/MoveIt state differs")
            require("physical_grasp_confirmed" in checks and "physical_release_and_placement" in checks,
                    "physical lift/release/settle evidence is missing")
            for name in ("approach_plan", "verification_lift_plan", "transport_plan", "place_plan"):
                require(checks[name]["sample_count"] >= 2 and checks[name]["minimum_clearance"]["distance_m"] >= .0005,
                        "native sampled path clearance is missing")
            require(any(event["event"] == "logical_attachment_recorded" for event in result["lifecycle"]["events"]),
                    "confirmed MoveIt attachment transition is missing")
            if args.case == "normal_hold":
                hold = checks.get("diagnostic_held_stability", {})
                require(result.get("diagnostic_hold", {}).get("requested_simulation_sec") == 30
                        and hold.get("observed_simulation_sec", 0) >= 30 and hold.get("sample_count", 0) >= 3
                        and hold.get("relative_position_error_m_max", 1) <= .005
                        and hold.get("relative_angle_error_rad_max", 1) <= .15
                        and hold.get("bilateral_fraction") == 1.
                        and hold.get("original_closed_reference_retained") is True,
                        "explicit 30-second held stability with unchanged physical gates was not verified")
        else:
            require(record["signal_sent"] and process.returncode == 1 and result["status"] == "cancelled",
                    "owned SIGINT did not produce a cancelled task")
            require("open" not in stages and "place" not in stages, "cancellation automatically continued the task")
            require(result["cleanup"].get("native_stop_observed"), "cancelled native three-controller stop is missing")
            if args.case == "cancel_transport":
                require(record["observed_motion_m"] >= .005 and stages[-1] == "transport",
                        "transport cancellation was not injected during observed movement")
                require(actions[-1].get("terminal_observed") and actions[-1].get("terminal_status") in (5, 6),
                        "moving transport cancellation/abort terminal was not observed")
                require(result["planning_attachment"] == "attached" and result["physical_grasp"] == "unconfirmed"
                        and result.get("physical_grasp_before_cancel") == "confirmed", "cancelled physical/attachment truth differs")
                require(result["action_terminal"]["stage"] == "transport"
                        and result["action_terminal"]["status"] in (5, 6), "displayed terminal belongs to an older action")
            else:
                require(not actions and result["planning_attachment"] == "uninitialized",
                        "startup cancellation dispatched an action or initialized a task scene")
        require(group_gone(process.pid), "owned worker process group remains")
        record["status"] = "passed"
    except BaseException as error:
        record.update(status="failed", error=dict(type=type(error).__name__, message=str(error)))
    finally:
        if process is not None and process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=45.)
            except Exception as error:
                record["cleanup_errors"].append("owned worker graceful stop: "+str(error))
                # The inner group was bound while its leader was a direct child
                # and PID/PGID/session/start-time matched. Recheck before signal.
                if inner is not None:
                    current = process_identity(inner["pid"])
                    if current and all(current[key] == inner[key] for key in ("pid", "pgrp", "session", "starttime")):
                        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                            try:
                                os.killpg(inner["pgrp"], sig)
                            except ProcessLookupError:
                                break
                            time.sleep(.2)
                try:
                    process.terminate()
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5.)
        if stream is not None:
            stream.close()
        record.update(completed_utc=utc_now(), latest_ipc_sequence=previous_sequence,
                      worker_process_group_gone=process is None or group_gone(process.pid))
        if (output/"worker.log").exists():
            record["worker_log_sha256"] = file_sha256(output/"worker.log")
        save()
    print(record["status"].upper()+": "+str(receipt), flush=True)
    return 0 if record["status"] == "passed" else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--sha", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", choices=("normal", "normal_hold", "cancel_transport", "cancel_startup"), required=True)
    args = parser.parse_args(argv)
    try:
        return verify(args)
    except Exception as error:
        print(type(error).__name__+": "+str(error), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
