"""One live simulation: repeated requests, duplicate/busy rejection, failures and cancellation."""
import argparse
import json
from pathlib import Path
import random
import subprocess
import sys
import time
import signal
import uuid

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.diagnostics import diagnostic_environment, finalize_physics
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.motion_request import COMMAND_TOPIC, CANCEL_TOPIC, STATUS_TOPIC, command
from mujoco_ros2_manipulation.workcells.motion_server import MotionCommandServer, status_qos
from mujoco_ros2_manipulation.workcells.verify import require


class CommandVerification(MotionCommandServer):
    def __init__(self, cell, progress):
        super().__init__(cell, ROOT / "runtime/validation/command-sessions" / uuid.uuid4().hex, progress)
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.cancel_publisher = self.create_publisher(String, CANCEL_TOPIC, 10)
        self.feedback = []
        self.create_subscription(String, STATUS_TOPIC, lambda msg: self.feedback.append(json.loads(msg.data)), status_qos())
        self.checked = []

    def submit(self, payload, expected, *, cancel_before=False, no_motion=False):
        identifier = payload["request_id"]
        self.command_publisher.publish(String(data=json.dumps(payload)))
        self.until(lambda: self.pending is not None or identifier in self.replies, 5, "command topic receipt")
        if cancel_before:
            self.cancel_publisher.publish(String(data=json.dumps(dict(request_id=identifier))))
            self.until(lambda: self.cancel_requested, 5, "pending cancellation receipt")
        if self.pending is not None:
            self.process_next()
        record = json.loads((self.directory / (identifier + ".json")).read_text())
        require(record["status"] == expected, f"Unexpected command status: {record['status']}: {record['error']}")
        require(record["metrics"]["received_to_terminal_wall_sec"] >= 0, "Missing terminal latency")
        times = [e["wall_sec"] for e in record["events"]]
        require(times == sorted(times), "Request wall timeline is not monotonic")
        if expected == "succeeded" and no_motion:
            require(record["metrics"]["received_to_first_motion_wall_sec"] is None and record["dispatched_goals"] == 0 and record.get("no_motion_required"), "No-op must not fabricate movement or action completion")
        elif expected == "succeeded":
            require(record["metrics"]["received_to_first_motion_wall_sec"] is not None, "Physical movement was not observed")
            require(record["metrics"]["received_to_actions_done_wall_sec"] >= record["metrics"]["received_to_first_motion_wall_sec"], "Movement/result timeline reversed")
            require(record["dispatched_goals"] == 2 and all(r["status"] == 4 for r in record["action_results"]), "Successful request must complete both action goals")
        self.checked.append(dict(request_id=identifier, expected=expected, trace_path=str(self.path_for(identifier)), metrics=record["metrics"]))
        return record

    def path_for(self, identifier):
        return self.directory / (identifier + ".json")

    def run(self):
        self.prepare()
        self.until(lambda: self.command_publisher.get_subscription_count() >= 1, 5, "command subscriber")
        self.create_subscription(String, COMMAND_TOPIC, lambda message: None, 10)  # passive rqt/echo-style monitor
        original = self.state()
        target = dict(original)
        for inst in self.cell.instances:
            target[inst.joints[0]] += 0.2
        first = command(target)
        self.progress("persistent_coordinated_command")
        self.submit(first, "succeeded")
        self.progress("persistent_departure_command")
        timing = self.submit(command(original, "departure_timing"), "succeeded")
        require(timing["departure_search"]["candidates"], "Departure search timing was not captured")
        dispatched = self.dispatched
        observed = len([f for f in self.feedback if f.get("request_id") == first["request_id"]])
        self.command_publisher.publish(String(data=json.dumps(first)))
        self.until(lambda: len([f for f in self.feedback if f.get("request_id") == first["request_id"]]) > observed, 5, "idempotent terminal replay")
        require(self.pending is None and self.dispatched == dispatched, "Duplicate command executed again")
        invalid = command(original)
        joint = self.cell.joints[0]
        invalid["target_rad"][joint] = self.metadata["joint_limits"][joint]["upper"] + 1
        self.progress("invalid_command_rejection")
        self.submit(invalid, "rejected")
        require(self.dispatched == dispatched, "Rejected target dispatched a goal")
        rng = random.Random(42)
        collision = None
        for _ in range(400):
            candidate = {n: min(self.metadata["joint_limits"][n]["upper"] - 0.02,
                               max(self.metadata["joint_limits"][n]["lower"] + 0.02, v + rng.uniform(-0.9, 0.9)))
                         for n, v in original.items()}
            if self.evaluator.at(candidate)["distance_m"] < 0 and not self.validity_of(candidate).valid:
                collision = candidate
                break
        require(collision is not None, "No deterministic colliding planning target found")
        self.progress("failed_plan_retained")
        failure = self.submit(command(collision), "failed")
        require(failure["dispatched_goals"] == 0 and failure["metrics"]["received_to_first_motion_wall_sec"] is None, "Failed plan moved the robot")
        self.progress("pending_request_cancelled")
        pending = self.submit(command(target), "cancelled", cancel_before=True)
        require(pending["dispatched_goals"] == 0, "Pending cancellation dispatched a goal")
        busy = command(original)
        active_payload = command({n: (v + 0.5 if n in {i.joints[0] for i in self.cell.instances} else v) for n, v in original.items()})
        flags = dict(busy=False, cancel=False)
        def interrupt():
            if self.active and self.first_by_arm:
                if not flags["busy"]:
                    self.command_publisher.publish(String(data=json.dumps(busy)))
                    flags["busy"] = True
                elif not flags["cancel"] and self.replies.get(busy["request_id"], {}).get("status") == "rejected":
                    self.cancel_publisher.publish(String(data=json.dumps(dict(request_id=active_payload["request_id"]))))
                    flags["cancel"] = True
        timer = self.create_timer(0.02, interrupt)
        self.progress("moving_request_cancelled_and_busy_rejected")
        try:
            cancelled = self.submit(active_payload, "cancelled")
        finally:
            self.destroy_timer(timer)
        require(flags["busy"] and flags["cancel"], "Busy/cancel topic checks did not run")
        busy_record = json.loads(self.path_for(busy["request_id"]).read_text())
        require(busy_record["status"] == "rejected" and "Busy" in busy_record["error"]["message"], "Overlapping command was not rejected")
        require(cancelled["metrics"]["received_to_first_motion_wall_sec"] is not None, "Cancellation occurred before physical motion")
        require([r["status"] for r in cancelled["action_results"]] == [5, 5], "Both goals must be cancelled")
        self.progress("persistent_command_after_cancel")
        self.submit(command(original), "succeeded")
        self.progress("already_reached_target_has_no_fake_latency")
        self.submit(command(self.state()), "succeeded", no_motion=True)
        self.progress("separate_command_client_process")
        target_file, result_file = self.directory / "client-target.json", self.directory / "client-result.json"
        target = self.state()
        for inst in self.cell.instances:
            target[inst.joints[0]] += 0.15
        target_file.write_text(json.dumps(target))
        with (self.directory / "client.log").open("w") as log:
            process = subprocess.Popen([sys.executable, str(ROOT / "motion_command.py"), "--target", str(target_file), "--output", str(result_file)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 90
            try:
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.05)
                    self.process_next()
                require(process.poll() == 0, "Separate command client failed; inspect client.log")
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
        external = json.loads(result_file.read_text())
        require(external["status"] == "succeeded" and external["client_publish_to_result_wall_sec"] > 0, "Separate client timing missing")
        self.progress("standalone_server_shutdown_cancels_both_arms")
        self.destroy_subscription(self.command_subscription)
        self.initialized = False
        self.destroy_publisher(self.publisher)
        self.until(lambda: self.count_publishers(STATUS_TOPIC) == 0, 5, "previous server publisher removed")
        standalone_dir = self.directory / "standalone-shutdown"
        standalone_log = self.directory / "standalone.log"
        with standalone_log.open("w") as log:
            process = subprocess.Popen([sys.executable, str(ROOT / "motion_server.py"), "--output", str(standalone_dir)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            try:
                self.until(lambda: "READY:" in standalone_log.read_text(), 25, "standalone command server startup")
                target = self.state()
                before = dict(target)
                for inst in self.cell.instances:
                    target[inst.joints[0]] += 0.4
                shutdown_payload = command(target)
                self.command_publisher.publish(String(data=json.dumps(shutdown_payload)))
                self.until(lambda: all(max(abs(self.state()[n] - before[n]) for n in inst.joints) > 0.01 for inst in self.cell.instances), 25, "both arms moving before server SIGINT")
                process.send_signal(signal.SIGINT)
                self.until(lambda: process.poll() is not None, 12, "standalone server cancels then exits")
                require(process.returncode == 0, "Standalone server shutdown failed")
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
        shutdown_trace = json.loads((standalone_dir / (shutdown_payload["request_id"] + ".json")).read_text())
        require(shutdown_trace["status"] == "cancelled" and [r["status"] for r in shutdown_trace["action_results"]] == [5, 5], "Server exit did not cancel both moving goals")
        self.quiet(cancellable=False)
        require(not self.contact_guard["latched"] and not self.blocked, "Normal request/cancellation latched the physical interlock")
        return dict(standalone_shutdown_trace=shutdown_trace, separate_client_result=external, command_session_id=self.session_id, command_trace_directory=str(self.directory),
                    command_checks=self.checked, duplicate_dispatch_prevented=True, busy_request_rejected=True,
                    all_requests_used_one_simulation=True, contact_guard=self.contact_guard)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=138)
    args = parser.parse_args()
    args.robot, args.scene, args.gui, args.stop_mode = "fr5_ur5e", "default", False, "interrupt"
    args.require_gpu, args.screenshot_dir = False, None
    return run_verification(args, ROOT, rclpy, CommandVerification, Trigger,
        profile_loader=lambda *a, **kw: load_workcell(root=kw["root"]), validator=check_workcell,
        command_builder=lambda cell, args: ["bash", str(ROOT / "run_scene.sh"), f"workcell:={cell.id}", "gui:=false", "rviz:=false", "planning_case:=dual_arm_crossing"],
        report_prefix="motion_commands_", launch_environment=diagnostic_environment, finalize=finalize_physics)


if __name__ == "__main__":
    raise SystemExit(main())
