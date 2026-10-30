"""Actual HTTP comparison jobs with explicit JPEG fallback and owned cleanup."""
import argparse
import hashlib
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
from urllib.parse import urlencode

from ..model_build.integrity import file_sha256
from ..paths import ROOT
from ..verification.reports import VerificationReport, utc_now
from ..verification.runner import group_gone, stop_process
from ..verification.process_faults import process_failures
from ..verification.diagnostics import check_stop_physics
from ..workcells.config import load_workcell
from ..workcells.contact_comparison import compare_runs
from ..workcells.contact_demo import OBSTACLE_BODY
from ..workcells.integrity import check_workcell
from ..workcells.motion_request import command
from ..workcells.timing_comparison import compare_timing
from .demo_worker import frame_matches
from .demo_jobs import matching_process


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def completed_job(state, identifier):
    """A worker's terminal status alone does not mean nested cleanup is done."""
    demo = state.get("demo") or {}
    current = demo.get("current") or {}
    if (current.get("job_id") == identifier
            and current.get("status") in ("succeeded", "failed", "cancelled")
            and current.get("cleanup_clean") is True
            and current.get("active_launch") is None
            and not (state.get("lifecycle", {}).get("owned") or {}).get("demo")
            and demo.get("can_start") is True):
        return current
    return None


def controller_spawner_readiness(log, controllers):
    """Require three one-shot helpers to have exited before active cancellation.

    Activation does not prove that a Python spawner has finished its own
    teardown. This is only a verification readiness barrier; cancellation and
    strict child-exit classification remain unchanged.
    """
    roles=("joint_state_broadcaster",*controllers)
    require(len(roles)==3 and len(set(roles))==3,"Expected three distinct controller startup helpers")
    # This fixed workcell launch emits simulation/state-publisher first, then
    # these helpers in role order. A changed launch must be reviewed explicitly.
    expected={"spawner-"+str(index):role for index,role in enumerate(roles,3)}
    plain=re.sub(r"\x1b\[[0-9;]*m","",log)
    lines=plain.splitlines()
    observed=dict(ready=False,reason="waiting_for_spawner_clean_exit",spawners=[])
    if process_failures(log) or any("process has died" in line for line in lines):
        observed["reason"]="child_process_failure"
        return observed
    if any("sending signal" in line or "user interrupted" in line for line in lines):
        observed["reason"]="launch_is_stopping"
        return observed
    starts=[];activated=[];exits=[]
    for number,line in enumerate(lines,1):
        match=re.fullmatch(r"\[INFO\] \[(spawner-[0-9]+)\]: process started with pid \[([0-9]+)\]",line)
        if match:starts.append((match[1],int(match[2]),number))
        match=re.fullmatch(r"\[(spawner-[0-9]+)\] \[INFO\] \[[^\]]+\] \[spawner_([^\]]+)\]: Configured and activated ([^ ]+)",line)
        if match:activated.append((match[1],match[2],match[3],number))
        match=re.fullmatch(r"\[INFO\] \[(spawner-[0-9]+)\]: process has finished cleanly \[pid ([0-9]+)\]",line)
        if match:exits.append((match[1],int(match[2]),number))
    names={item[0] for item in starts}
    all_names=set(re.findall(r"\[(spawner-[0-9]+)\]",plain))
    if (len(starts)!=3 or names!=set(expected) or len({item[1] for item in starts})!=3
            or any(item[1]<=1 for item in starts) or all_names!=names):
        observed["reason"]="missing_duplicate_or_unknown_spawner"
        return observed
    if (len(activated)!=3 or {item[0] for item in activated}!=names
            or {item[1] for item in activated}!=set(roles)
            or any(item[1]!=item[2] or item[1]!=expected[item[0]] for item in activated)):
        observed["reason"]="missing_duplicate_or_wrong_controller_activation"
        return observed
    if len(exits)!=3 or {item[0] for item in exits}!=names:
        return observed
    for name,pid,started in starts:
        activation=next(item for item in activated if item[0]==name)
        exited=next(item for item in exits if item[0]==name)
        if exited[1]!=pid or not started<activation[3]<exited[2]:
            observed["reason"]="wrong_spawner_pid_or_event_order"
            return observed
        observed["spawners"].append(dict(node=name,pid=pid,controller=activation[1],
            start_line=started,activated_line=activation[3],clean_exit_line=exited[2]))
    observed.update(ready=True,reason=None)
    return observed


def owned_controller_startup_readiness(launch, root, domain, started_utc, controllers):
    """Read only the exact active launch's bounded, identity-checked report/log."""
    require(type(launch) is dict,"Missing active launch for cancellation readiness")
    run=launch.get("run_id")
    require(type(run) is str and re.fullmatch(r"[A-Za-z0-9_.-]{1,128}",run),"Invalid active launch run identity")
    runs=(Path(root)/"runtime/validation/runs").resolve()
    directory=(runs/run).resolve()
    path=Path(launch.get("report_path","")).resolve()
    require(directory.parent==runs and path==directory/"result.json",
            "Active cancellation report escaped its owned run")
    require(type(launch.get("pid")) is int and launch["pid"]>1
            and type(launch.get("start_ticks")) is int and launch["start_ticks"]>0,
            "Invalid active launch process identity")
    if not matching_process(launch):
        return dict(ready=False,reason="active_launch_no_longer_matches",run_id=run,spawners=[])
    require(path.stat().st_size<=64*1024*1024,"Active cancellation report exceeded its bound")
    raw=path.read_bytes()
    require(len(raw)<=64*1024*1024,"Active cancellation report grew beyond its bound")
    value=json.loads(raw)
    log=directory/"launch.log"
    require(value.get("run_id")==run and value.get("ros_domain_id")==domain
            and Path(value.get("log_path","")).resolve()==log,
            "Active cancellation report belongs to another launch/domain/log")
    require(datetime.fromisoformat(value["started_utc"])>=datetime.fromisoformat(started_utc),
            "Active cancellation reused an archived report")
    if value.get("status")!="running" or value.get("stage")!="approach_fixed_obstacle":
        return dict(ready=False,reason="active_report_not_approaching",run_id=run,spawners=[])
    require(log.stat().st_size<=4*1024*1024,"Active cancellation log exceeded its bound")
    logged=log.read_bytes()
    require(len(logged)<=4*1024*1024,"Active cancellation log grew beyond its bound")
    observed=controller_spawner_readiness(logged.decode("utf-8",errors="replace"),controllers)
    observed.update(run_id=run,launch_pid=launch["pid"],launch_start_ticks=launch["start_ticks"],
        stage=value["stage"],report_sha256=hashlib.sha256(raw).hexdigest(),
        observed_log_sha256=hashlib.sha256(logged).hexdigest(),observed_log_bytes=len(logged))
    return observed


def active_contact_approach(state, identifier, controllers):
    current=(state.get("demo") or {}).get("current") or {}
    guard=state.get("guard") or {}
    age=state.get("state_age_sec")
    return (current.get("job_id")==identifier and current.get("variant")=="contact"
        and current.get("status")=="running" and current.get("stage")=="approach_fixed_obstacle"
        and guard.get("latched") is False and guard.get("phase")=="ready"
        and all((guard.get("controllers") or {}).get(name)=="active" for name in controllers)
        and type(age) in (int,float) and 0<=age<1)


def checked_reports(job, variants, root, domain, started_utc):
    """Read only reports named by this job and verify identity and fresh history."""
    results = job["results"]
    require(len(results) == len(variants) and {r["variant"] for r in results} == set(variants),
            "Unexpected comparison variants")
    require(len({r["run_id"] for r in results}) == len(results), "Comparison reused a launch")
    values = {}
    for summary in results:
        path = Path(summary["report_path"]).resolve()
        expected = root / "runtime/validation/runs" / summary["run_id"] / "result.json"
        require(path == expected.resolve() and path.parent.parent == (root / "runtime/validation/runs").resolve(),
                "Comparison result escaped its owned verification directory")
        require(path.stat().st_size <= 64 * 1024 * 1024, "Comparison result is unexpectedly large")
        require(file_sha256(path) == summary["report_sha256"], "Comparison report changed")
        value = json.loads(path.read_text())
        require(value["run_id"] == summary["run_id"] and value["ros_domain_id"] == domain,
                "Comparison report belongs to another launch/domain")
        require(datetime.fromisoformat(value["started_utc"]) >= datetime.fromisoformat(started_utc),
                "Comparison reused archived evidence")
        require(value["status"] == "passed" and value["shutdown_clean"] is True
                and value["process_group_gone"] is True and value["motion_checks_passed"] is True
                and value["physics_checks_passed"] is True, "Comparison did not complete physical verification")
        values[summary["variant"]] = (path, value)
    return values


def archive_console_evidence(session, destination):
    """Preserve failed jobs before checking whether their cleanup succeeded."""
    session, destination = Path(session), Path(destination)
    evidence = dict(path=str(destination), copied=False, audit_valid=False,
                    captured_utc=utc_now())
    errors = []
    try:
        shutil.copytree(session, destination)
        evidence["copied"] = True
    except Exception as error:
        errors.append("Console evidence copy: " + str(error))
    # Audit failure must not prevent retention of status, worker logs or stacks.
    # Check the source even if copying failed, so both errors remain visible.
    try:
        events = [json.loads(line) for line in (session / "events.jsonl").read_text().splitlines()]
        require(all(group_gone(e["pid"]) for e in events if e["action"] == "process_started"),
                "Owned console child process group remains")
        require(all(e.get("cleanup_clean") is True for e in events if e["action"] == "demo_finished"),
                "A demonstration needed forced or incomplete cleanup")
        require(not any(e.get("cleanup_errors") for e in events), "Owned console needed forced shutdown")
        evidence.update(audit_valid=True, event_count=len(events))
    except Exception as error:
        errors.append(str(error))
    return evidence, errors


def checked_cancelled_report(job, root, domain, started_utc, *, require_latched=False,
                             expected_timestep=None):
    """Validate the cancelled nested run, including its actual contact condition."""
    require(job.get("status") == "cancelled" and job.get("comparison") is None,
            "Expected a cancelled comparison without a completed comparison")
    results = job.get("results", [])
    require(len(results) == 1 and results[0].get("variant") == "contact",
            "Cancelled comparison ran another variant")
    summary = results[0]
    path = Path(summary["report_path"]).resolve()
    expected = root / "runtime/validation/runs" / summary["run_id"] / "result.json"
    require(path == expected.resolve() and path.parent.parent == (root / "runtime/validation/runs").resolve(),
            "Cancelled report escaped its owned verification directory")
    require(path.stat().st_size <= 64 * 1024 * 1024, "Cancelled report is unexpectedly large")
    require(file_sha256(path) == summary["report_sha256"], "Cancelled report changed")
    value = json.loads(path.read_text())
    require(value["run_id"] == summary["run_id"] and value["ros_domain_id"] == domain,
            "Cancelled report belongs to another launch/domain")
    require(datetime.fromisoformat(value["started_utc"]) >= datetime.fromisoformat(started_utc),
            "Cancelled report reused archived evidence")
    require(value["status"] == "interrupted" and value["shutdown_clean"] is True
            and value["process_group_gone"] is True
            and value.get("error", {}).get("type") == "DemoCancelled",
            "Nested cancellation did not finish with clean owned shutdown")
    if require_latched:
        physics = value.get("physics") or {}
        require(value.get("failure_stage") == "approach_fixed_obstacle"
                and physics.get("complete") is True and physics.get("stop_latched") is True
                and physics.get("contacts", 0) > 0 and physics.get("hold_ticks", 0) >= 10,
                "Cancellation did not occur after a latched contact with a stationary hold")
        physics_path = path.parent / "physics.json"
        require(physics_path.resolve().parent == path.parent
                and physics_path.stat().st_size <= 64 * 1024 * 1024,
                "Cancelled physics report escaped its run or exceeded its bound")
        require(file_sha256(physics_path) == value.get("physics_report_sha256")
                and json.loads(physics_path.read_text()) == physics,
                "Cancelled physics report changed or differs from embedded evidence")
        require(expected_timestep is not None, "Expected physics timestep is required")
        check_stop_physics(physics, run_id=value["run_id"],
                           source_sha256=file_sha256(root / "native/step_monitor.hpp"),
                           timestep=expected_timestep)
        require(physics.get("stop_reason") == "contact"
                and all(OBSTACLE_BODY in p.get("bodies", [])
                        and any(n.startswith("left_") for n in p["bodies"])
                        and p.get("min_distance_m", -1) > -.01 for p in physics["pairs"]),
                "Latched cancellation has an unexpected stop reason or contact pair")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=156)
    parser.add_argument("--port", type=int, default=8879)
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--cancel-after-latch", action="store_true",
                        help="Cancel after the fixed obstacle has latched and both controllers stopped")
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not 1024 <= args.port <= 65535:
        parser.error("Invalid isolated domain or loopback port")
    cell = load_workcell()
    check_workcell(cell)
    report = VerificationReport(ROOT, "operator_demos", "headless" if args.no_render else "render", args.domain)
    process = session = None
    token = None  # Never print or add this transient HTTP credential to a report.
    failed, cleanup, jobs, frames = False, [], [], {}
    console_evidence = None
    url = f"http://127.0.0.1:{args.port}"

    def api(route, body=None):
        request = Request(url + route) if body is None else Request(
            url + route, data=json.dumps(body, allow_nan=False).encode(),
            headers={"Content-Type": "application/json", "Origin": url, "X-Operator-Token": token})
        with urlopen(request, timeout=5) as response:
            data = response.read(4 * 1024 * 1024 + 1)
            require(len(data) <= 4 * 1024 * 1024, "Console JSON exceeded its bound")
            return json.loads(data)

    def until(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Owned console exited before " + label)
            try:
                result = predicate()
                if result:
                    return result
            except HTTPError:
                raise
            except (URLError, TimeoutError):
                pass
            time.sleep(.05)
        raise RuntimeError("Timed out: " + label)

    def reject(route, body, code):
        try:
            api(route, body)
        except HTTPError as error:
            require(error.code == code, f"Wrong HTTP rejection for {route}: {error.code}")
        else:
            raise RuntimeError("Unexpectedly accepted " + route)

    def observe(identifier):
        state = api("/api/state")
        current = (state.get("demo") or {}).get("current") or {}
        if current.get("job_id") == identifier:
            launch = current.get("active_launch")
            if launch and frame_matches(state.get("frame"), identifier, launch["run_id"], launch.get("model_sha256")):
                frame = state["frame"]
                query = urlencode({key:frame[key] for key in ("job_id", "run_id", "model_sha256", "sequence", "generation", "operator_session_id")
                                   if key in frame})
                with urlopen(url+"/api/frame?"+query, timeout=5) as response:
                    if response.status == 204:
                        return state  # A newer frame arrived; observe it on the next poll.
                    pixels = response.read(4*1024*1024+1)
                    require(response.status == 200 and response.headers.get_content_type() == "image/jpeg"
                            and 0 < len(pixels) <= 4*1024*1024
                            and hashlib.sha256(pixels).hexdigest() == frame.get("jpeg_sha256"),
                            "Live JPEG differs from its exact scene/frame metadata")
                frames[launch["run_id"]] = dict(
                    job_id=identifier, run_id=launch["run_id"], variant=current["variant"],
                    model_sha256=launch["model_sha256"],
                    joint_state_stamp_simulation_sec=state.get("joint_state_stamp_simulation_sec"))
        return state

    def finished(identifier):
        state = observe(identifier)
        current = (state.get("demo") or {}).get("current") or {}
        if (current.get("job_id") == identifier and current.get("status") == "failed"
                and current.get("cleanup_clean") is False):
            raise RuntimeError("Demo cleanup failed: " + str(current.get("error")))
        return completed_job(state, identifier)

    def start(scenario):
        until(lambda: api("/api/state")["demo"]["can_start"], 15, "empty demo domain")
        response = api("/api/demos/start", dict(scenario=scenario, native_gui=False))
        identifier = response["job_id"]
        require(re.fullmatch("[0-9a-f]{32}", identifier) is not None, "Invalid new job ID")
        require(identifier not in {j["job_id"] for j in jobs}, "Comparison reused a job ID")
        return identifier

    def save_job(identifier, timeout, expected="succeeded"):
        value = until(lambda: finished(identifier), timeout, "finished job and cleaned nested launches")
        jobs.append(value)
        report.update(checked_jobs=jobs, observed_frame_runs=list(frames.values()))
        require(value["status"] == expected, f"Expected {expected} comparison: {value.get('error')}")
        require(value["job_id"] == identifier and value["cleanup_clean"] is True,
                "Wrong comparison or incomplete cleanup")
        return value

    try:
        report.update(code_commit=subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
            model_manifest_sha256=file_sha256(cell.output / "source.json"),
            renderer_requested=not args.no_render)
        report.stage("owned_console_start")
        with report.log_path.open("w") as log:
            process = subprocess.Popen(
                ["/usr/bin/python3", str(ROOT / "operator_console.py"), "--domain", str(args.domain),
                 "--port", str(args.port)] + (["--no-render"] if args.no_render else []),
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def owned_session():
            # Bind the HTTP connection to our successful port reservation first.
            text = report.log_path.read_text()[-65536:]
            match = re.search(r"OPERATOR_READY http://127\.0\.0\.1:" + str(args.port)
                              + r"/ domain=" + str(args.domain) + r" session=([^\n]+)", text)
            if not match:
                return None
            path = Path(match.group(1)).resolve()
            require(path.is_relative_to((ROOT / "runtime/operator").resolve()), "Unexpected owned console directory")
            return path
        session = until(owned_session, 20, "owned console port reservation")
        bootstrap = until(lambda: api("/api/bootstrap"), 10, "console bootstrap")
        token = bootstrap["token"]
        require(bootstrap["config"]["domain"] == args.domain, "Console is on another ROS domain")
        require({s["id"] for s in bootstrap["demos"]} == {"contact_comparison", "timing_comparison"},
                "Fixed demonstration catalog is incomplete")
        if not args.no_render:
            display = api("/api/view/display", dict(mode="image"))
            require(display.get("display_mode") == "image", "JPEG fallback was not selected")
            report.update(display_mode="image", renderer_scope="Explicit JPEG fallback; web 3D has a separate verifier")
        until(lambda: api("/api/state")["demo"]["can_start"], 10, "isolated console readiness")
        reject("/api/demos/start", dict(scenario="unknown", native_gui=False), 400)

        report.stage("contact_pair_and_exclusive_ownership")
        identifier = start("contact_comparison")
        reject("/api/demos/start", dict(scenario="contact_comparison", native_gui=False), 409)
        reject("/api/start", dict(native_gui=False, planning_case="none"), 409)
        reject("/api/command", command(bootstrap["config"]["home_rad"]), 409)
        wrong = "f" * 32 if identifier != "f" * 32 else "e" * 32
        reject("/api/demos/stop", dict(job_id=wrong), 409)
        require(observe(identifier)["demo"]["current"]["job_id"] == identifier, "Wrong-job stop changed ownership")
        contact = save_job(identifier, 300)
        originals = checked_reports(contact, ("contact", "control"), ROOT, args.domain, report.data["started_utc"])
        expected_contact = compare_runs(originals["contact"][1], originals["control"][1])
        require(contact["comparison"] == expected_contact, "Contact HTTP result differs from original matched evidence")
        require(expected_contact["identical_commands"] is True
                and expected_contact["contact_goal_statuses"] == [5, 5]
                and expected_contact["control_goal_statuses"] == [4, 4]
                and expected_contact["commanded_motion_duration_sec"] == 8
                and 1.5 < expected_contact["native_contact_motion_elapsed_sec"] < 2.5
                and expected_contact["remaining_command_sec_at_contact"] > 4.5,
                "Contact did not interrupt the same unfinished eight-second command")

        report.stage("fresh_timing_pair")
        identifier = start("timing_comparison")
        timing = save_job(identifier, 900)
        originals = checked_reports(timing, ("prepare", "baseline", "timing"), ROOT, args.domain, report.data["started_utc"])
        job_directory = session / "demos" / identifier
        bundle_path, plan_path = job_directory / "bundle.json", job_directory / "timing-plan.json"
        bundle, plan = json.loads(bundle_path.read_text()), json.loads(plan_path.read_text())
        require(bundle["source_run_id"] == originals["prepare"][1]["run_id"]
                and bundle["source_report_sha256"] == file_sha256(originals["prepare"][0]),
                "Timing pair did not prepare a fresh source")
        require(plan["bundle_sha256"] == file_sha256(bundle_path), "Timing plan belongs to another bundle")
        expected_timing = compare_timing([originals["baseline"][0], originals["timing"][0]], plan_path)
        require(expected_timing["matched"] is True and timing["comparison"] == expected_timing,
                "Timing HTTP result differs from the matched original reports")
        run_ids = [r["run_id"] for job in jobs for r in job["results"]]
        require(len(run_ids) == len(set(run_ids)), "Separate comparison jobs reused a launch")
        if not args.no_render:
            require(set(run_ids).issubset(frames), "A live run had no matching fresh rendered frame")

        report.stage("cancel_latched_contact_job" if args.cancel_after_latch else "cancel_active_contact_job")
        identifier = start("contact_comparison")
        stopped_since = [None]
        startup_readiness = [None]
        controllers = tuple(instance.controller for instance in cell.instances)
        def approaching():
            state = observe(identifier)
            current = state["demo"]["current"]
            if current["job_id"] == identifier and current["status"] in ("succeeded", "failed", "cancelled"):
                raise RuntimeError("Contact job ended before cancellation was exercised")
            matching = (current["job_id"] == identifier and current.get("variant") == "contact"
                        and current.get("stage") in ("approach_fixed_obstacle", "waiting_for_frame"))
            if not matching:
                return None
            if args.cancel_after_latch:
                guard = state.get("guard") or {}
                if not (current.get("stage") == "approach_fixed_obstacle"
                        and guard.get("latched") is True and guard.get("phase") == "stopped"
                        and all((guard.get("controllers") or {}).get(i.controller) == "inactive"
                                for i in cell.instances)
                        and state.get("state_age_sec") is not None and state["state_age_sec"] < 1):
                    stopped_since[0] = None
                    return None
                if stopped_since[0] is None:
                    stopped_since[0] = time.monotonic()
                return state if time.monotonic() - stopped_since[0] >= .1 else None
            if not active_contact_approach(state,identifier,controllers):
                return None
            launch=current.get("active_launch") or {}
            startup_readiness[0]=owned_controller_startup_readiness(launch,ROOT,args.domain,
                report.data["started_utc"],controllers)
            report.update(cancellation_startup_readiness=startup_readiness[0])
            if not startup_readiness[0]["ready"]:
                return None
            # File observation is not an atomic stop admission. Recheck the
            # same active, unlatched approach after reading its startup proof.
            latest=observe(identifier)
            newest=(latest.get("demo") or {}).get("current") or {}
            if (not active_contact_approach(latest,identifier,controllers)
                    or newest.get("active_launch")!=launch):
                return None
            return latest
        cancellation_state = until(approaching, 120, "owned contact approach before cancellation")
        cancellation_launch = cancellation_state["demo"]["current"].get("active_launch") or {}
        require(cancellation_launch.get("run_id") and cancellation_launch["run_id"] not in run_ids,
                "Cancellation did not observe a new owned launch")
        report.update(cancellation_observation=dict(run_id=cancellation_launch["run_id"],
            stage=cancellation_state["demo"]["current"]["stage"],
            after_latch=args.cancel_after_latch, guard=cancellation_state.get("guard"),
            simulation_sec=cancellation_state.get("simulation_sec"),
            controller_startup_readiness=startup_readiness[0]))
        response = api("/api/demos/stop", dict(job_id=identifier))
        require(response["job_id"] == identifier and response["status"] == "cancel_requested", "Wrong cancellation acknowledgement")
        cancelled = save_job(identifier, 90, "cancelled")
        require(cancelled["comparison"] is None
                and all(r["variant"] != "control" for r in cancelled["results"]),
                "Cancelled contact job continued to its control/comparison")
        nested = checked_cancelled_report(cancelled, ROOT, args.domain, report.data["started_utc"],
                                          require_latched=args.cancel_after_latch,
                                          expected_timestep=cell.data["physics"]["timestep"])
        require(nested["run_id"] == cancellation_launch["run_id"],
                "Cancelled evidence differs from the observed launch")
        report.update(motion_checks_passed=True, exclusive_owner_checked=True,
                      invalid_scenario_rejected=True, wrong_job_stop_rejected=True,
                      manual_commands_rejected=True, active_cancel_checked=True,
                      latched_cancel_checked=args.cancel_after_latch,
                      cancelled_nested_run_id=nested["run_id"],
                      renderer_checked=not args.no_render)
    except (Exception, KeyboardInterrupt) as error:
        failed = True
        report.update(error=dict(type=type(error).__name__, message=str(error)), failure_stage=report.data["stage"])
    finally:
        token = None
        if process is not None:
            try:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                process.wait(timeout=70)
                require(process.returncode == 0 and group_gone(process.pid), "Owned console shutdown was not clean")
            except Exception as error:
                cleanup.append(str(error))
                stop_process(process)
        if session is not None and session.is_dir():
            console_evidence, evidence_errors = archive_console_evidence(
                session, report.directory / "console-session")
            cleanup.extend(evidence_errors)
        report.update(status="failed" if failed or cleanup else "passed", stage="complete",
                      completed_utc=utc_now(), cleanup_errors=cleanup, shutdown_clean=not cleanup,
                      launch_return_code=None if process is None else process.returncode,
                      console_session=None if session is None else str(session),
                      console_evidence=console_evidence)
    print(report.data["status"].upper() + ": " + str(report.path), flush=True)
    if report.data["error"]:
        print(report.data["error"], flush=True)
    if cleanup:
        print(cleanup, flush=True)
    return 1 if failed or cleanup else 0


if __name__ == "__main__":
    raise SystemExit(main())
