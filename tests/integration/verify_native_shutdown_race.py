"""Own a simulation-only JTC update gate and inspect actual shutdown ordering.

This deliberately stalls one installed JTC virtual update. It sends no robot
goals and never targets a pre-existing process. Do not use it with real robots.
An old unsafe overlap is preserved as expected failure evidence, not a safe run.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import group_gone, stop_process

JOINED = "CONTROL_LOOP_SHUTDOWN phase=UPDATE_LOOP_JOINED"
CONTEXT = "CONTROL_LOOP_SHUTDOWN phase=CONTEXT_SHUTDOWN_BEGIN"
TEARDOWN = "Shutdown request received"
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def require(value, message):
    if not value:
        raise AssertionError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_log(path):
    if not path.exists():
        return ""
    require(path.stat().st_size <= 32 * 1024 * 1024, "Bounded probe log exceeded 32 MiB")
    return ANSI.sub("", path.read_text(errors="replace"))


def gate(path, run_id):
    require(re.fullmatch(r"[a-f0-9]{32}", run_id), "Gate requires a fresh UUID hex run ID")
    temporary = path.with_name(".gate-" + uuid.uuid4().hex)
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
    try:
        with os.fdopen(descriptor, "w") as stream:
            stream.write(run_id + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        # A hard link publishes the complete file atomically and refuses an
        # existing name. The RT gate never sees an empty or partial arm/release.
        os.link(temporary, path, follow_symlinks=False)
    finally:
        temporary.unlink()


def wait_for(predicate, timeout, description, process=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        if process is not None and process.poll() is not None:
            raise RuntimeError("Owned launch exited before " + description)
        time.sleep(.01)
    raise TimeoutError("Bounded wait expired: " + description)


def entered_record(path):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError:
        # A single regular-file write can become visible before fsync completes.
        return None


def validate_owner(entry, *, run_id, launch_pid, native, expected_sha, shim, jtc, simulation=None):
    require(entry.get("event") == "entered" and entry.get("run_id") == run_id,
            "Probe gate belongs to another run")
    pid = entry.get("pid")
    require(type(pid) is int and pid > 1 and type(entry.get("tid")) is int and entry["tid"] > 1
            and type(entry.get("monotonic_ns")) is int and entry["monotonic_ns"] > 0,
            "Actual native PID/TID are absent")
    process = Path("/proc") / str(pid)
    require(process.stat().st_uid == os.getuid() and os.getpgid(pid) == launch_pid,
            "Refuse to signal a process outside this owned launch group")
    require((process / "exe").resolve() == native.resolve() and sha(native) == expected_sha,
            "Overlay executable or native SHA differs from the declared fixture")
    mappings = (process / "maps").read_text()
    require(str(shim) in mappings and str(jtc) in mappings,
            "Actual native process did not map both the probe and installed JTC")
    if simulation is not None:
        require(str(simulation.resolve()) in mappings, "Native loaded the wrong old/fixed simulation library")
    return pid


def validate_bound_events(record):
    """Bind the event stream to the validated owned entry and actual SIGINT."""
    entry, events = record["entered"], record["probe_events"]
    run_id, pid = record["run_id"], record["native_pid"]
    require(type(run_id) is str and re.fullmatch(r"[a-f0-9]{32}", run_id)
        and type(pid) is int and pid > 1, "Invalid validated run/PID identity")
    require(type(entry) is dict and entry.get("event") == "entered" and entry.get("run_id") == run_id
        and type(entry.get("pid")) is int and entry["pid"] == pid
        and type(entry.get("tid")) is int and entry["tid"] > 1
        and type(entry.get("monotonic_ns")) is int and entry["monotonic_ns"] > 0,
        "Validated entry identity/timestamp differs from this native run")
    require(type(events) is list and 2 <= len(events) <= 3
        and [event.get("event") for event in events if type(event) is dict]
        in (["entered", "released"], ["entered", "released", "returned"]),
        "Actual probe stream requires one entered/released prefix and optional one return")
    for event in events:
        require(type(event) is dict and type(event.get("pid")) is int and event["pid"] == pid
            and type(event.get("tid")) is int and event["tid"] == entry["tid"]
            and event.get("run_id") == run_id
            and type(event.get("monotonic_ns")) is int and event["monotonic_ns"] > 0,
            "Probe event is missing identity or belongs to a foreign run/PID/TID")
    require(all(a["monotonic_ns"] < b["monotonic_ns"] for a, b in zip(events, events[1:])),
            "Probe event timestamps must be strictly advancing and unique")
    sent = record["signal"]
    release = record["release_created_monotonic_ns"]
    require(type(sent) is dict and sent.get("signal") == "SIGINT"
        and type(sent.get("pid")) is int and sent["pid"] == pid
        and all(type(value) is int and value > 0 for value in (
            sent.get("before_monotonic_ns"), sent.get("after_monotonic_ns"), release)),
        "Signal/release timestamps or owned signal identity are missing")
    require(entry["monotonic_ns"] == events[0]["monotonic_ns"] < sent["before_monotonic_ns"]
        <= sent["after_monotonic_ns"] < release <= events[1]["monotonic_ns"],
        "Validated entry, bound entered event, actual SIGINT and release watermarks disagree")
    return dict(bound_to_validated_entry=True, pid=pid, tid=entry["tid"], run_id=run_id,
                event_count=len(events), strictly_advancing_watermarks=True)


def ordering(held_log, final_log, events, expected):
    """Pure acceptance contract; ordering evidence remains distinct from exit."""
    names = [event.get("event") for event in events]
    require(names[:2] == ["entered", "released"] and "hold_timeout" not in names,
            "No complete actual held-update gate, or the watchdog expired")
    require(all(type(event.get("monotonic_ns")) is int for event in events)
            and all(a["monotonic_ns"] <= b["monotonic_ns"] for a, b in zip(events, events[1:])),
            "Probe event monotonic times are invalid")
    require(len({(event.get("pid"), event.get("tid"), event.get("run_id")) for event in events}) == 1,
            "Held update events do not belong to one native thread/run")
    require("SHUTDOWN_PROBE event=released" not in held_log and JOINED not in held_log,
            "Pre-release snapshot was taken after the held gate had ended")
    overlap = TEARDOWN in held_log
    if expected == "legacy":
        require(overlap, "Old runtime did not expose CM pre-shutdown while actual JTC update was held")
        return dict(expected_unsafe_overlap_observed=True, safe_ordering_observed=False,
                    ordering_assertions_passed=True, native_root_cause_closed=False)
    require(expected == "fixed" and not overlap and CONTEXT not in held_log,
            "Fixed runtime began context/controller teardown before update release")
    require(names == ["entered", "released", "returned"], "Actual held JTC update did not return")
    markers = ["SHUTDOWN_PROBE event=released", "SHUTDOWN_PROBE event=returned", JOINED, CONTEXT, TEARDOWN]
    positions = [final_log.find(marker) for marker in markers]
    require(all(position >= 0 for position in positions) and positions == sorted(positions)
            and len(set(positions)) == len(positions),
            "Expected release -> actual JTC return -> owner join -> context shutdown -> CM teardown")
    return dict(expected_unsafe_overlap_observed=False, safe_ordering_observed=True,
                ordering_assertions_passed=True, native_root_cause_closed=False)


def verify(args):
    from mujoco_ros2_manipulation.workcells.verify_motion_reproduction import require_unused_domain
    from mujoco_ros2_manipulation.workcells.config import load_workcell
    from mujoco_ros2_manipulation.verification.diagnostics import check_physics
    from mujoco_ros2_manipulation.verification.process_faults import process_failures

    output = args.output.resolve()
    require(not output.exists() and output.is_relative_to((ROOT / "runtime").resolve()),
            "Select a new evidence directory under this workspace runtime/")
    require_unused_domain(args.domain)
    shim, jtc = args.probe.resolve(strict=True), Path("/opt/ros/humble/lib/libjoint_trajectory_controller.so")
    build = json.loads(shim.with_name("build.json").read_text())
    require(build["sha256"].get(str(shim)) == sha(shim)
            and build["sha256"].get(str(jtc)) == sha(jtc), "Probe/JTC differs from its build receipt")
    prefix = args.overlay.resolve(strict=True) if args.overlay else ROOT / "ros_ws/install"
    native = prefix / "mujoco_ros2_control/lib/mujoco_ros2_control/ros2_control_node"
    simulation = prefix / "mujoco_ros2_control/lib/libmujoco_ros2_control.so"
    require(native.is_file() and sha(native) == args.expected_native_sha256,
            "Declared old/fixed native executable SHA does not match")
    if args.overlay:
        require((prefix / "local_setup.bash").is_file(), "Overlay must be an isolated colcon install prefix")
    cell = load_workcell(args.workcell)
    output.mkdir(parents=True, mode=0o700)
    gates = output / "private-gates"
    gates.mkdir(mode=0o700)
    run_id = uuid.uuid4().hex
    record = dict(schema_version=1, status="running", expected=args.expect, run_id=run_id, domain=args.domain,
        scope="Test-only installed JTC update/SIGINT ordering on owned MuJoCo simulation; no goal dispatch",
        automatic_retry=False, native_root_cause_closed=False, old_w01_stall_resolved=False,
        native_path=str(native), native_sha256=sha(native), simulation_library_path=str(simulation),
        simulation_library_sha256=sha(simulation), probe_sha256=sha(shim), jtc_sha256=sha(jtc),
        workcell=args.workcell, model_manifest_sha256=sha(cell.output / "source.json"),
        overlay=str(prefix), native_pid=None, signal=None, ordering=None, error=None, physics_checks_passed=False)
    receipt, log_path = output / "result.json", output / "launch.log"
    atomic_json(receipt, record)
    process = None
    try:
        environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), MANIPULATION_ROS_DOMAIN_ID=str(args.domain),
            MANIPULATION_DIAGNOSTICS_PATH=str(output / "physics.json"), MANIPULATION_DIAGNOSTICS_RUN_ID=run_id,
            MANIPULATION_SHUTDOWN_PROBE_DIR=str(gates), MANIPULATION_SHUTDOWN_PROBE_RUN_ID=run_id,
            MANIPULATION_SHUTDOWN_PROBE_MAX_HOLD_MS="10000", MANIPULATION_SHUTDOWN_PROBE_LIBRARY=str(shim),
            MANIPULATION_SHUTDOWN_PROBE_JTC_LIBRARY=str(jtc))
        # run_scene.sh sources the canonical overlay again. Source once here,
        # then the explicitly selected overlay LAST and invoke ros2 directly.
        shell = 'set -e\nsource "$1/manipulation_env.sh"\nif [[ -n "$2" ]]; then source "$2/local_setup.bash"; fi\nexec ros2 launch "$3" "workcell:=$4" gui:=false rviz:=false'
        command = ["bash", "--noprofile", "--norc", "-c", shell, "owned-shutdown-probe", str(ROOT),
            str(prefix) if args.overlay else "", str(ROOT / "tests/native/shutdown_probe/probe.launch.py"), args.workcell]
        record["command"] = command
        with log_path.open("x") as log:
            process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True)
        record["launch_pid"] = process.pid
        atomic_json(receipt, record)
        controllers = [item.controller for item in cell.instances]
        wait_for(lambda: all("Configured and activated " + name in read_log(log_path) for name in controllers),
                 60., "both original arm controllers active", process)
        gate(gates / "arm", run_id)
        entry = wait_for(lambda: entered_record(gates / "entered.json"), 5., "actual held JTC update", process)
        native_pid = validate_owner(entry, run_id=run_id, launch_pid=process.pid, native=native,
                                   expected_sha=args.expected_native_sha256, shim=shim, jtc=jtc,
                                   simulation=simulation)
        (output / "native-maps.txt").write_text((Path("/proc") / str(native_pid) / "maps").read_text())
        record.update(native_pid=native_pid, entered=entry)
        record["signal"] = dict(pid=native_pid, signal="SIGINT", before_monotonic_ns=time.monotonic_ns())
        atomic_json(receipt, record)
        os.kill(native_pid, signal.SIGINT)
        record["signal"]["after_monotonic_ns"] = time.monotonic_ns()
        # Use a bounded observation window in both variants; old callbacks get
        # time to expose teardown, while fixed must keep it absent throughout.
        deadline = time.monotonic() + args.observe_sec
        while time.monotonic() < deadline:
            require(not (gates / "release").exists(), "Release file appeared before owner decision")
            require(process.poll() is None, "Owned launch exited while actual JTC update was held")
            time.sleep(.01)
        held = read_log(log_path)
        (output / "held-before-release.log").write_text(held)
        record["held_snapshot_monotonic_ns"] = time.monotonic_ns()
        record["release_created_monotonic_ns"] = time.monotonic_ns()
        gate(gates / "release", run_id)
        process.wait(timeout=25.)
        final = read_log(log_path)
        events = [json.loads(line) for line in (gates / "events.jsonl").read_text().splitlines()]
        record["probe_events"] = events
        record["event_binding"] = validate_bound_events(record)
        record["ordering"] = ordering(held, final, events, args.expect)
        record["all_process_failures"] = process_failures(final)
        record["native_failures"] = [item for item in record["all_process_failures"] if item["pid"] == native_pid]
        unexpected = [item for item in record["all_process_failures"] if not (
            args.expect == "legacy" and item["pid"] == native_pid and item["exit_code"] == -11)]
        require(not unexpected, "Unexpected additional native/MoveIt/guard process failure")
        clean_native = f"process has finished cleanly [pid {native_pid}]" in final
        record["native_clean_exit_observed"] = clean_native
        if args.expect == "fixed":
            require(clean_native and not record["native_failures"], "Fixed owned native process did not exit cleanly")
            data = json.loads((output / "physics.json").read_text())
            check_physics(data, run_id=run_id, source_sha256=sha(ROOT / "native/step_monitor.hpp"),
                          timestep=cell.data["physics"]["timestep"], require_interlock=True)
            record["physics_checks_passed"] = True
            record["physics_sha256"] = sha(output / "physics.json")
            record["status"] = "passed"
        else:
            record["status"] = "expected_unsafe_overlap"
            record["native_sigsegv_observed"] = any(item["exit_code"] == -11 for item in record["native_failures"])
    except BaseException as error:
        record.update(status="failed", error=dict(type=type(error).__name__, message=str(error)))
    finally:
        # Always unblock this test-owned gate before touching its launch session.
        if not (gates / "release").exists():
            gate(gates / "release", run_id)
        if process is not None:
            try:
                forced = stop_process(process)
                record["cleanup"] = dict(forced=forced, launch_return_code=process.returncode,
                                         process_group_gone=group_gone(process.pid))
                require(not forced and record["cleanup"]["process_group_gone"], "Owned cleanup required escalation or left processes")
                if args.expect == "fixed":
                    require(process.returncode == 0, "Fixed launch process returned nonzero")
            except BaseException as error:
                record["cleanup_error"] = dict(type=type(error).__name__, message=str(error))
                record["status"] = "failed"
        record["source_sha256"] = {str(path.relative_to(ROOT)): sha(path) for path in (
            Path(__file__).resolve(), ROOT / "tests/native/shutdown_probe/update_probe.cpp",
            ROOT / "tests/native/shutdown_probe/probe.launch.py")}
        record["launch_log_sha256"] = sha(log_path) if log_path.exists() else None
        atomic_json(receipt, record)
    print(record["status"].upper() + ": " + str(receipt), flush=True)
    return 0 if record["status"] in ("passed", "expected_unsafe_overlap") else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expect", choices=("legacy", "fixed"), required=True)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--overlay", type=Path)
    parser.add_argument("--expected-native-sha256", required=True)
    parser.add_argument("--workcell", choices=("fr5_ur5e", "fr5_ur5e_eval_post"), default="fr5_ur5e_eval_post")
    parser.add_argument("--observe-sec", type=float, default=1.)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not .25 <= args.observe_sec <= 2. or not re.fullmatch(r"[a-f0-9]{64}", args.expected_native_sha256):
        parser.error("Expected unused domain 0..232, bounded .25..2-second hold observation and explicit SHA256")
    return verify(args)


if __name__ == "__main__":
    raise SystemExit(main())
