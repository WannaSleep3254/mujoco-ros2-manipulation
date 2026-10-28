"""Actual fixed-node clock/signal lifecycle probes, without hardware or controllers.

Explicit invocation only. A test-only LD_PRELOAD shim observes the pinned Humble
ControllerManager ABI and can inject SIGTERM immediately after handler installation.
The shim changes timing and these runs are excluded from performance measurements.
No robot description, model, physics engine, GUI or Action is launched. Real loaded
controller teardown races are covered by a separate native integration test.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
CM_SYMBOLS = {
    "read": "_ZN18controller_manager17ControllerManager4readERKN6rclcpp4TimeERKNS1_8DurationE",
    "update": "_ZN18controller_manager17ControllerManager6updateERKN6rclcpp4TimeERKNS1_8DurationE",
    "write": "_ZN18controller_manager17ControllerManager5writeERKN6rclcpp4TimeERKNS1_8DurationE",
}
CASES = (("missing_sigint", "missing", signal.SIGINT),
         ("missing_sigterm", "missing", signal.SIGTERM),
         ("paused_sigint", "paused", signal.SIGINT),
         ("paused_sigterm", "paused", signal.SIGTERM),
         ("early_sigterm", "early", signal.SIGTERM))
# The argument references are pointers in the pinned Linux Itanium C++ ABI. The
# update return type is exactly uint8_t, verified against the installed header.
SHIM = r'''#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
static void observe(const char *kind) {
  const char *path=getenv("MANIPULATION_CLOCK_PROBE_TRACE");
  if(!path) _exit(124);
  int fd=open(path,O_WRONLY|O_CREAT|O_APPEND,0600);
  if(fd<0) _exit(124);
  struct timespec now; clock_gettime(CLOCK_MONOTONIC,&now);
  char line[160]; int n=snprintf(line,sizeof(line),"%lld %s\n",
    (long long)now.tv_sec*1000000000LL+now.tv_nsec,kind);
  if(n<=0 || n>=(int)sizeof(line) || write(fd,line,(size_t)n)!=n) _exit(124);
  close(fd);
}
int sigaction(int number,const struct sigaction *action,struct sigaction *old) {
  typedef int (*Fn)(int,const struct sigaction*,struct sigaction*);
  Fn real=(Fn)dlsym(RTLD_NEXT,"sigaction"); if(!real) _exit(125);
  int result=real(number,action,old);
  static atomic_int sent=0;
  if(result==0 && number==SIGTERM && action && action->sa_handler!=SIG_DFL &&
    action->sa_handler!=SIG_IGN && getenv("MANIPULATION_CLOCK_PROBE_EARLY") &&
    atomic_exchange(&sent,1)==0) {
    observe("early_sigterm_sent");
    if(raise(SIGTERM)!=0) _exit(126);
    observe("early_sigterm_returned");
  }
  return result;
}
void probe_read(void*,const void*,const void*) __asm__("READ_SYMBOL");
void probe_read(void *owner,const void *stamp,const void *period) {
  typedef void (*Fn)(void*,const void*,const void*);
  Fn real=(Fn)dlsym(RTLD_NEXT,"READ_SYMBOL"); if(!real) _exit(125);
  observe("read_enter");real(owner,stamp,period);observe("read_done");
}
uint8_t probe_update(void*,const void*,const void*) __asm__("UPDATE_SYMBOL");
uint8_t probe_update(void *owner,const void *stamp,const void *period) {
  typedef uint8_t (*Fn)(void*,const void*,const void*);
  Fn real=(Fn)dlsym(RTLD_NEXT,"UPDATE_SYMBOL"); if(!real) _exit(125);
  observe("update_enter");uint8_t result=real(owner,stamp,period);observe("update_done");return result;
}
void probe_write(void*,const void*,const void*) __asm__("WRITE_SYMBOL");
void probe_write(void *owner,const void *stamp,const void *period) {
  typedef void (*Fn)(void*,const void*,const void*);
  Fn real=(Fn)dlsym(RTLD_NEXT,"WRITE_SYMBOL"); if(!real) _exit(125);
  observe("write_enter");real(owner,stamp,period);observe("write_done");
}
'''
for _name, _symbol in CM_SYMBOLS.items():
    SHIM = SHIM.replace(_name.upper()+"_SYMBOL", _symbol)


def require(value, message):
    if not value:
        raise AssertionError(message)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def save(path, value):
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n")
    temporary.replace(path)


def trace_events(path):
    if not path.exists():
        return []
    require(path.stat().st_size <= 1024*1024, "Unbounded instrumentation trace")
    rows = []
    for line in path.read_text().splitlines():
        stamp, kind = line.split(" ", 1)
        rows.append(dict(steady_ns=int(stamp), event=kind))
    require(all(a["steady_ns"] <= b["steady_ns"] for a,b in zip(rows, rows[1:])),
            "Instrumentation wall clock regressed")
    return rows


def counters(path):
    return dict(Counter(row["event"] for row in trace_events(path)))


def ordered_markers(log, expected_sha):
    phases = ("UPDATE_LOOP_JOINED", "CONTEXT_SHUTDOWN_BEGIN", "CONTEXT_SHUTDOWN_DONE")
    offsets = []
    for phase in phases:
        marker = f"CONTROL_LOOP_SHUTDOWN phase={phase} patch_sha256={expected_sha}"
        require(log.count(marker) == 1, "Missing/duplicate fixed-node phase marker: "+phase)
        offsets.append(log.index(marker))
    require(offsets == sorted(offsets), "Context shutdown preceded RT join")
    cleanup = ("Shutdown request received", "Shutting down all controllers", "Deactivating controller")
    require(all(log.find(text) < 0 or log.find(text) > offsets[0] for text in cleanup),
            "Controller cleanup started before the RT join marker")
    require("Control loop failed" not in log and "Control clock failed" not in log,
            "Clock/signal fixture encountered a control-loop error")
    return dict(phases=list(phases), controller_cleanup_present=any(text in log for text in cleanup))


def group_gone(pid):
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return True
    return False


def cleanup_child(process):
    actions = []
    for sig, wait in ((signal.SIGINT, 2.), (signal.SIGTERM, 2.), (signal.SIGKILL, 2.)):
        if process.poll() is not None:
            break
        actions.append(sig.name)
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue
    return dict(actions=actions, returncode=process.poll(), process_group_gone=group_gone(process.pid))


def prepare(output, binary):
    require(binary.is_file() and os.access(binary, os.X_OK), "Fixed native executable missing")
    require(binary.stat().st_size < 32*1024*1024, "Unexpected native executable size")
    header = ROOT/"native/control_loop_shutdown.hpp"
    patch = ROOT/"patches/mujoco_ros2_control-0.1.2-ordered-shutdown.patch"
    header_sha = sha(header)
    require(header_sha.encode() in binary.read_bytes(), "Binary lacks the current shutdown header SHA")
    cm_package = Path("/opt/ros/humble/share/controller_manager/package.xml")
    cm_version = ET.parse(cm_package).findtext("version")
    require(cm_version == "2.54.3", "This observation shim requires locked ControllerManager 2.54.3")
    cm_library = Path("/opt/ros/humble/lib/libcontroller_manager.so")
    interface_header = Path("/opt/ros/humble/include/controller_interface/controller_interface/controller_interface_base.hpp")
    require(re.search(r"enum\s+class\s+return_type\s*:\s*std::uint8_t", interface_header.read_text()),
            "Pinned update return ABI changed")
    symbols = subprocess.run(["nm", "-D", str(cm_library)], capture_output=True, text=True, check=True, timeout=10).stdout
    require(all(name in symbols for name in CM_SYMBOLS.values()), "Pinned ControllerManager method ABI changed")
    snapshot = output/"source-snapshot"
    snapshot.mkdir()
    sources = {"verifier.py": Path(__file__), "control_loop_shutdown.hpp": header,
               "ordered-shutdown.patch": patch, "controller_interface_base.hpp": interface_header,
               "controller-manager-package.xml": cm_package}
    source_sha = {}
    for name, source in sources.items():
        (snapshot/name).write_bytes(source.read_bytes())
        source_sha[name] = sha(source)
    source = output/"clock_probe.c"
    source.write_text(SHIM)
    library = output/"clock_probe.so"
    command = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", str(source), "-ldl", "-o", str(library)]
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=30)
    (output/"shim-build.log").write_text(compiled.stdout+compiled.stderr)
    require(compiled.returncode == 0, "Clock observation shim did not compile")
    manifest = dict(schema_version=1, binary=dict(path=str(binary), sha256=sha(binary)),
        shutdown_header_sha256=header_sha, patch_sha256=sha(patch), source_sha256=source_sha,
        controller_manager=dict(version=cm_version, library=str(cm_library), sha256=sha(cm_library)),
        shim=dict(source_sha256=sha(source), binary_sha256=sha(library), command=command,
                  abi="Pinned Linux Itanium C++ symbols; pointer references and uint8_t update return"))
    save(output/"manifest.json", manifest)
    return manifest, library


def run_case(folder, binary, library, domain, mode, requested_signal, manifest):
    # Lazy imports keep preparation and result parsing independent from ROS.
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    folder.mkdir()
    receipt = dict(schema_version=1, domain=domain, mode=mode, signal=requested_signal.name, passed=False,
        scope="Fixed native node with no robot description/hardware/controllers; test-only observation shim",
        performance_sample=False, physics_or_physical_stop_verified=False)
    process = context = probe = executor = None
    trace = folder/"events.log"
    log_path = folder/"native.log"
    try:
        context = Context()
        rclpy.init(args=[], context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
        probe = Node("native_clock_probe_"+uuid.uuid4().hex, context=context, enable_rosout=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(probe)
        def spin_for(seconds):
            deadline = time.monotonic()+seconds
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=min(.02, max(0., deadline-time.monotonic())))
        spin_for(1.)
        others = [dict(name=name, namespace=ns) for name,ns in probe.get_node_names_and_namespaces() if name != probe.get_name()]
        receipt["preflight_other_nodes"] = others
        require(not others, "ROS domain already has nodes")
        child_name = "native_shutdown_"+uuid.uuid4().hex
        command = [str(binary), "--ros-args", "-r", "__node:="+child_name,
                   "-p", "use_sim_time:=true", "-p", "update_rate:=100", "-p", "lock_memory:=false"]
        environment = os.environ.copy()
        environment.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", RCUTILS_COLORIZED_OUTPUT="0",
            LD_PRELOAD=str(library), MANIPULATION_CLOCK_PROBE_TRACE=str(trace))
        environment.pop("MANIPULATION_CLOCK_PROBE_EARLY", None)
        if mode == "early":
            environment["MANIPULATION_CLOCK_PROBE_EARLY"] = "1"
        receipt["command"] = command
        with log_path.open("wb") as raw_log:
            process = subprocess.Popen(command, env=environment, cwd=folder, stdout=raw_log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            receipt["owned_pid"] = process.pid
            require(os.getpgid(process.pid) == process.pid, "Child is not in its own process group")
            if mode == "early":
                process.wait(timeout=10.)
                counts = counters(trace)
                require(counts.get("early_sigterm_sent") == counts.get("early_sigterm_returned") == 1,
                        "Early signal did not run exactly after actual handler installation")
                require(counts.get("update_enter", 0) == counts.get("read_enter", 0) == 0,
                        "A control cycle began after the pre-initialization stop request")
                receipt["early_signal_injected_before_rclcpp_init"] = True
                receipt["terminal_counts"] = counts
            else:
                deadline = time.monotonic()+10.
                while time.monotonic() < deadline:
                    require(process.poll() is None, "Native node exited before clock fixture readiness")
                    executor.spin_once(timeout_sec=.03)
                    clock_subscribers = probe.get_subscriptions_info_by_topic("/clock")
                    if any(info.node_name == child_name for info in clock_subscribers) and "RT thread with scheduler priority" in log_path.read_text(errors="replace"):
                        break
                else:
                    raise TimeoutError("Native node did not expose its simulation-clock subscription")
                if mode == "paused":
                    publisher = probe.create_publisher(Clock, "/clock", QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
                    deadline = time.monotonic()+3.
                    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.02)
                    require(publisher.get_subscription_count() >= 1, "Clock publisher did not match the fixed node")
                    for index in range(60):
                        message = Clock()
                        message.clock.sec, message.clock.nanosec = divmod(1_000_000_000+index*20_000_000, 1_000_000_000)
                        publisher.publish(message)
                        executor.spin_once(timeout_sec=.02)
                    receipt["clock_messages_sent"] = 60
                    receipt["last_clock_stamp_ns"] = 2_180_000_000
                    spin_for(.3)
                    before = counters(trace)
                    require(before.get("update_done", 0) >= 3, "No actual CM cycles observed under advancing clock")
                    started = time.monotonic()
                    spin_for(.3)
                    after = counters(trace)
                    require(before == after, "Control cycle repeated while source clock was paused")
                    receipt["paused_observation"] = dict(wall_sec=time.monotonic()-started, before=before, after=after)
                    probe.destroy_publisher(publisher)
                else:
                    spin_for(.3)
                    require(counters(trace).get("read_enter", 0) == 0, "Control cycle ran before simulation clock started")
                signal_wall = time.monotonic()
                process.send_signal(requested_signal)
                process.wait(timeout=3.)
                receipt["signal_to_exit_wall_sec"] = time.monotonic()-signal_wall
                receipt["terminal_counts"] = counters(trace)
            require(process.returncode == 0, "Fixed node did not exit normally: "+str(process.returncode))
        receipt["markers"] = ordered_markers(log_path.read_text(errors="replace"), manifest["shutdown_header_sha256"])
        require(group_gone(process.pid), "Owned native process group remains")
        require(sha(binary) == manifest["binary"]["sha256"], "Fixed native executable changed during test")
        receipt["passed"] = True
    except Exception as error:
        receipt["error"] = dict(type=type(error).__name__, message=str(error)[:4096])
    finally:
        if process is not None:
            receipt["cleanup"] = cleanup_child(process)
            if not receipt["cleanup"]["process_group_gone"]:
                receipt["passed"] = False
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if context is not None and context.ok():
            context.shutdown()
        save(folder/"result.json", receipt)
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--domain", required=True, type=int, help="Five unused consecutive ROS domains, base 0..228")
    parser.add_argument("--output", required=True, type=Path, help="New directory, including failed/partial originals")
    parser.add_argument("--prepare-only", action="store_true", help="Compile the observation shim and pin sources; start no ROS")
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 228:
        parser.error("--domain must be between 0 and 228 inclusive")
    output, binary = args.output.resolve(), args.binary.resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = dict(schema_version=1, passed=False, started_utc=datetime.now(timezone.utc).isoformat(),
        scope="No-hardware clock/signal lifecycle probes; not a robot/controller/physics or performance evaluation",
        automatic_retry=False, cases=[])
    try:
        manifest, library = prepare(output, binary)
        result["manifest"] = "manifest.json"
        if args.prepare_only:
            result["status"] = "prepared_only_no_ros_started"
        else:
            for offset, (name, mode, requested_signal) in enumerate(CASES):
                result["cases"].append(run_case(output/name, binary, library, args.domain+offset, mode, requested_signal, manifest))
            result["passed"] = all(case["passed"] for case in result["cases"])
            result["status"] = "passed" if result["passed"] else "failed"
    except BaseException as error:
        result["status"] = "failed"
        result["error"] = dict(type=type(error).__name__, message=str(error)[:4096])
    finally:
        save(output/"verification.json", result)
    print(json.dumps(dict(status=result["status"], passed=result["passed"], output=str(output),
                         error=result.get("error")), ensure_ascii=False, indent=2))
    return 0 if result["passed"] or result["status"] == "prepared_only_no_ros_started" else 1


if __name__ == "__main__":
    raise SystemExit(main())
