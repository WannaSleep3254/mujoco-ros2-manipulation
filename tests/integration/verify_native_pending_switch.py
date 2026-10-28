"""Owned simulation regression: pending broadcaster activation during SIGINT.

No debugger, ptrace permission changes, robot motion goals or automatic retries.
The runtime-only launch loads the original broadcaster inactive. Its real
SwitchController activation is requested only after an actual JTC update is
held, so the pending service condition is explicit and recorded.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid

HELPER = Path(__file__).with_name("verify_native_shutdown_race.py").resolve()
SPEC = importlib.util.spec_from_file_location("owned_shutdown_probe", HELPER)
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)

PENDING_LAUNCH = '''"""Generated test-only pending broadcaster launch; original config unchanged."""
import os
from pathlib import Path
def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import OpaqueFunction
    from mujoco_ros2_manipulation.workcells import launch as production
    def instrumented(context):
        original=production.Node
        native_count=broadcaster_count=0
        def node(*args,**kwargs):
            nonlocal native_count,broadcaster_count
            if kwargs.get("package")=="mujoco_ros2_control" and kwargs.get("executable")=="ros2_control_node":
                native_count+=1
                environment=dict(kwargs.get("additional_env",{}))
                environment["LD_PRELOAD"]=os.pathsep.join((os.environ["MANIPULATION_SHUTDOWN_PROBE_LIBRARY"],os.environ["MANIPULATION_SHUTDOWN_PROBE_JTC_LIBRARY"],environment["LD_PRELOAD"]))
                kwargs["additional_env"]=environment
                kwargs["ros_arguments"]=["--log-level","controller_manager:=debug"]
            if kwargs.get("package")=="controller_manager" and kwargs.get("executable")=="spawner" and kwargs.get("arguments",[None])[0]=="joint_state_broadcaster":
                broadcaster_count+=1
                kwargs["arguments"]=[*kwargs["arguments"],"--inactive"]
            return original(*args,**kwargs)
        production.Node=node
        try: result=production.nodes(context)
        finally: production.Node=original
        if (native_count,broadcaster_count)!=(1,1):
            raise RuntimeError("Expected one native node and one broadcaster spawner")
        return result
    description=production.generate_launch_description()
    original=[item for item in description.entities if isinstance(item,OpaqueFunction)]
    if len(original)!=1: raise RuntimeError("Production launch shape changed")
    return LaunchDescription([OpaqueFunction(function=instrumented) if item is original[0] else item for item in description.entities])
'''


def verify(args):
    import rclpy
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from mujoco_ros2_manipulation.workcells.verify_motion_reproduction import require_unused_domain
    from mujoco_ros2_manipulation.workcells.config import load_workcell
    from mujoco_ros2_manipulation.verification.diagnostics import check_physics

    root = probe.ROOT
    output = args.output.resolve()
    probe.require(not output.exists() and output.is_relative_to((root / "runtime").resolve()),
                  "Select a new output directory under runtime/")
    require_unused_domain(args.domain)
    prefix = args.overlay.resolve(strict=True) if args.overlay else root / "ros_ws/install"
    native = prefix / "mujoco_ros2_control/lib/mujoco_ros2_control/ros2_control_node"
    simulation = prefix / "mujoco_ros2_control/lib/libmujoco_ros2_control.so"
    probe.require(probe.sha(native) == args.expected_native_sha256, "Declared native binary SHA differs")
    if args.overlay:
        probe.require((prefix / "local_setup.bash").is_file(), "Overlay install prefix is incomplete")
    shim = args.probe.resolve(strict=True)
    jtc = Path("/opt/ros/humble/lib/libjoint_trajectory_controller.so")
    build = json.loads(shim.with_name("build.json").read_text())
    probe.require(build["sha256"].get(str(shim)) == probe.sha(shim)
                  and build["sha256"].get(str(jtc)) == probe.sha(jtc), "Probe build/JTC identity differs")
    cell = load_workcell("fr5_ur5e_eval_post")
    output.mkdir(parents=True, mode=0o700)
    gates = output / "private-gates"
    gates.mkdir(mode=0o700)
    launch = output / "pending.launch.py"
    launch.write_text(PENDING_LAUNCH)
    source_files = [Path(__file__).resolve(), HELPER, launch,
                    root / "tests/native/shutdown_probe/update_probe.cpp"]
    source_sha = {str(path): probe.sha(path) for path in source_files}
    for path in source_files:
        (output / ("source-" + path.name)).write_bytes(path.read_bytes())
    run_id = uuid.uuid4().hex
    record = dict(schema_version=1, status="running", expected="fixed", regression_expectation=args.expect,
        run_id=run_id, domain=args.domain, scope=__doc__, automatic_retry=False,
        actual_write_calls_measured=False,
        post_signal_write0_evidence="Native helper API/source and pure regression; this actual fixture checks callback/order/physics/exit but does not count write calls",
        native_path=str(native), native_sha256=probe.sha(native), simulation_library_sha256=probe.sha(simulation),
        jtc_sha256=probe.sha(jtc), probe_sha256=probe.sha(shim), model_manifest_sha256=probe.sha(cell.output / "source.json"),
        source_sha256=source_sha, native_pid=None, entered=None, signal=None,
        pending_switch=None, physics_checks_passed=False, error=None,
        old_w01_stall_resolved=False, native_root_cause_closed=False)
    receipt, log_path = output / "result.json", output / "launch.log"
    probe.atomic_json(receipt, record)
    process = node = future = None
    initialized = False

    def save():
        probe.atomic_json(receipt, record)

    def spin_until(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(node, timeout_sec=.02)
        raise TimeoutError("Bounded wait expired: " + description)

    try:
        environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), MANIPULATION_ROS_DOMAIN_ID=str(args.domain),
            MANIPULATION_DIAGNOSTICS_PATH=str(output / "physics.json"), MANIPULATION_DIAGNOSTICS_RUN_ID=run_id,
            MANIPULATION_SHUTDOWN_PROBE_DIR=str(gates), MANIPULATION_SHUTDOWN_PROBE_RUN_ID=run_id,
            MANIPULATION_SHUTDOWN_PROBE_MAX_HOLD_MS="10000", MANIPULATION_SHUTDOWN_PROBE_LIBRARY=str(shim),
            MANIPULATION_SHUTDOWN_PROBE_JTC_LIBRARY=str(jtc))
        shell = 'set -e\nsource "$1/manipulation_env.sh"\nif [[ -n "$2" ]]; then source "$2/local_setup.bash"; fi\nexec ros2 launch "$3" workcell:=fr5_ur5e_eval_post gui:=false rviz:=false'
        command = ["bash", "--noprofile", "--norc", "-c", shell, "owned-pending-switch", str(root),
                   str(prefix) if args.overlay else "", str(launch)]
        record["command"] = command
        with log_path.open("x") as log:
            process = subprocess.Popen(command, cwd=root, env=environment, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        record["launch_pid"] = process.pid
        arms = [item.controller for item in cell.instances]
        record["startup_readiness"] = probe.wait_for(
            lambda: probe.startup_readiness(probe.read_log(log_path), arms, inactive=("joint_state_broadcaster",)),
            60., "two active arms and inactive broadcaster; all original spawners clean", process)
        os.environ["ROS_DOMAIN_ID"] = str(args.domain)
        os.environ["MANIPULATION_ROS_DOMAIN_ID"] = str(args.domain)
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node("owned_pending_switch_" + run_id[:8])
        lists = node.create_client(ListControllers, "/controller_manager/list_controllers")
        switches = node.create_client(SwitchController, "/controller_manager/switch_controller")
        probe.require(lists.wait_for_service(timeout_sec=5.) and switches.wait_for_service(timeout_sec=5.),
                      "Original controller manager services are not ready")
        listed = lists.call_async(ListControllers.Request())
        spin_until(listed.done, 5., "actual complete controller states before arm")
        states = {item.name: item.state for item in listed.result().controller}
        probe.require(states.get("joint_state_broadcaster") == "inactive"
            and all(states.get(name) == "active" for name in arms), "Inactive/active fixture state differs")
        record["actual_prearm_controller_states"] = states
        probe.gate(gates / "arm", run_id)
        entry = probe.wait_for(lambda: probe.entered_record(gates / "entered.json"), 5., "actual held JTC update", process)
        pid = probe.validate_owner(entry, run_id=run_id, launch_pid=process.pid, native=native,
            expected_sha=args.expected_native_sha256, shim=shim, jtc=jtc, simulation=simulation)
        record.update(native_pid=pid, entered=entry)
        (output / "native-maps.txt").write_text((Path("/proc") / str(pid) / "maps").read_text())
        request = SwitchController.Request()
        request.activate_controllers = ["joint_state_broadcaster"]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        baseline = len(probe.read_log(log_path))
        record["pending_switch"] = dict(activate_controllers=list(request.activate_controllers),
            deactivate_controllers=[], strictness=int(request.strictness), activate_asap=True,
            timeout_sec=5, request_sent_monotonic_ns=time.monotonic_ns(), future_done_before_signal=None)
        future = switches.call_async(request)

        def pending_entered():
            probe.require(not future.done(), "Activation completed before the intended pending/SIGINT condition")
            post = probe.read_log(log_path)[baseline:]
            return "Requested atomic controller switch from realtime loop" in post

        spin_until(pending_entered, 3., "native callback queued the actual broadcaster activation")
        # Preserve the native callback's debug witness and the actual unresolved
        # response, rather than inferring callback entry from a slow client.
        (output / "pending-callback-before-signal.log").write_text(probe.read_log(log_path)[baseline:])
        record["pending_switch"].update(callback_queued_monotonic_ns=time.monotonic_ns(),
            future_done_before_signal=future.done(), native_callback_marker="Requested atomic controller switch from realtime loop")
        record["signal"] = dict(pid=pid, signal="SIGINT", before_monotonic_ns=time.monotonic_ns())
        save()
        os.kill(pid, signal.SIGINT)
        record["signal"]["after_monotonic_ns"] = time.monotonic_ns()
        hold_deadline = time.monotonic() + .25
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        (output / "held-before-release.log").write_text(probe.read_log(log_path))
        record["release_created_monotonic_ns"] = time.monotonic_ns()
        probe.gate(gates / "release", run_id)
        deadline = time.monotonic() + 8.
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        exited = process.poll() is not None
        record["pending_switch"].update(future_done_after_observation=future.done(),
            response_ok=bool(future.result().ok) if future.done() and future.exception() is None else None)
        record["observation"] = dict(wall_budget_after_release_sec=8., launch_exited=exited,
            observed_monotonic_ns=time.monotonic_ns(), native_alive=Path("/proc", str(pid)).exists())
        events = [json.loads(line) for line in (gates / "events.jsonl").read_text().splitlines()]
        record["probe_events"] = events
        record["event_binding"] = probe.validate_bound_events(record)
        probe.require([event["event"] for event in events] == ["entered", "released", "returned"],
                      "Held actual JTC update did not return after release")
        if args.expect == "hang":
            probe.require(not exited and record["observation"]["native_alive"], "Expected pre-fix liveness failure was not reproduced")
            record.update(status="expected_liveness_failure", error=dict(type="ObservedLivenessTimeout",
                message="Actual broadcaster activation was pending when SIGINT stopped native; still alive 8s after JTC returned"))
        else:
            probe.require(exited, "Pending-switch native did not finish within the wall budget")
            final = probe.read_log(log_path)
            record["ordering"] = probe.ordering(probe.read_log(output / "held-before-release.log"), final, events, "fixed")
            probe.require(f"process has finished cleanly [pid {pid}]" in final, "Native clean exit was not observed")
            physics = json.loads((output / "physics.json").read_text())
            check_physics(physics, run_id=run_id, source_sha256=probe.sha(root / "native/step_monitor.hpp"),
                timestep=cell.data["physics"]["timestep"], require_interlock=True)
            record.update(status="passed", physics_checks_passed=True, physics_sha256=probe.sha(output / "physics.json"))
    except BaseException as error:
        record.update(status="failed", error=dict(type=type(error).__name__, message=str(error)))
    finally:
        if not (gates / "release").exists():
            probe.gate(gates / "release", run_id)
        if process is not None:
            try:
                forced = probe.stop_process(process)
                record["cleanup"] = dict(forced=forced, launch_return_code=process.returncode,
                    process_group_gone=probe.group_gone(process.pid))
            except BaseException as error:
                record["cleanup_error"] = dict(type=type(error).__name__, message=str(error))
                record["status"] = "failed"
        if future is not None:
            record["future_done_after_cleanup"] = future.done()
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        try:
            event_path = gates / "events.jsonl"
            events = [json.loads(line) for line in event_path.read_text().splitlines()] if event_path.exists() else None
            held = output / "held-before-release.log"
            probe.final_observations(record, probe.read_log(log_path), events,
                probe.read_log(held) if held.exists() else None)
            if args.expect == "clean":
                probe.enforce_clean_final_proof(record)
            probe.require(record.get("cleanup", {}).get("process_group_gone") is True,
                          "Owned processes were not fully cleaned")
            if args.expect == "clean":
                probe.require(not record["all_process_failures"] and not record["launch_internal_escalations"]
                    and not record["cleanup"]["forced"] and record["cleanup"]["launch_return_code"] == 0,
                    "Clean regression needed escalation or had a child failure")
            probe.require(all(probe.sha(Path(path)) == digest for path, digest in source_sha.items()),
                          "Regression sources changed during this run")
        except BaseException as error:
            record["final_observation_error"] = dict(type=type(error).__name__, message=str(error))
            record["status"] = "failed"
        record["launch_log_sha256"] = probe.sha(log_path) if log_path.exists() else None
        save()
    print(record["status"].upper() + ": " + str(receipt), flush=True)
    return 0 if record["status"] in ("passed", "expected_liveness_failure") else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expect", choices=("hang", "clean"), required=True)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--overlay", type=Path)
    parser.add_argument("--expected-native-sha256", required=True)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not re.fullmatch(r"[a-f0-9]{64}", args.expected_native_sha256):
        parser.error("Expected unused domain 0..232 and explicit native SHA256")
    return verify(args)


if __name__ == "__main__":
    raise SystemExit(main())
