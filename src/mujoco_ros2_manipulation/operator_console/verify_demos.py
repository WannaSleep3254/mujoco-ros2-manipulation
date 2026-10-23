"""Actual HTTP comparison jobs, including matched evidence and owned cleanup."""
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
from ..workcells.config import load_workcell
from ..workcells.contact_comparison import compare_runs
from ..workcells.integrity import check_workcell
from ..workcells.motion_request import command
from ..workcells.timing_comparison import compare_timing
from .demo_worker import frame_matches


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=156)
    parser.add_argument("--port", type=int, default=8879)
    parser.add_argument("--no-render", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.domain <= 232 or not 1024 <= args.port <= 65535:
        parser.error("Invalid isolated domain or loopback port")
    cell = load_workcell()
    check_workcell(cell)
    report = VerificationReport(ROOT, "operator_demos", "headless" if args.no_render else "render", args.domain)
    process = session = None
    token = None  # Never print or add this transient HTTP credential to a report.
    failed, cleanup, jobs, frames = False, [], [], {}
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
                query = urlencode({key:frame[key] for key in ("job_id", "run_id", "model_sha256", "sequence")})
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

        report.stage("cancel_active_contact_job")
        identifier = start("contact_comparison")
        def approaching():
            state = observe(identifier)
            current = state["demo"]["current"]
            if current["job_id"] == identifier and current["status"] in ("succeeded", "failed", "cancelled"):
                raise RuntimeError("Contact job ended before cancellation was exercised")
            return state if (current["job_id"] == identifier and current.get("variant") == "contact"
                             and current.get("stage") in ("approach_fixed_obstacle", "waiting_for_frame")) else None
        until(approaching, 120, "owned contact approach before cancellation")
        response = api("/api/demos/stop", dict(job_id=identifier))
        require(response["job_id"] == identifier and response["status"] == "cancel_requested", "Wrong cancellation acknowledgement")
        cancelled = save_job(identifier, 90, "cancelled")
        require(cancelled["comparison"] is None
                and all(r["variant"] != "control" for r in cancelled["results"]),
                "Cancelled contact job continued to its control/comparison")
        report.update(motion_checks_passed=True, exclusive_owner_checked=True,
                      invalid_scenario_rejected=True, wrong_job_stop_rejected=True,
                      manual_commands_rejected=True, active_cancel_checked=True,
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
            try:
                events = [json.loads(line) for line in (session / "events.jsonl").read_text().splitlines()]
                require(all(group_gone(e["pid"]) for e in events if e["action"] == "process_started"),
                        "Owned console child process group remains")
                require(all(e.get("cleanup_clean") is True for e in events if e["action"] == "demo_finished"),
                        "A demonstration needed forced or incomplete cleanup")
                require(not any(e.get("cleanup_errors") for e in events), "Owned console needed forced shutdown")
                shutil.copytree(session, report.directory / "console-session")
            except Exception as error:
                cleanup.append(str(error))
        report.update(status="failed" if failed or cleanup else "passed", stage="complete",
                      completed_utc=utc_now(), cleanup_errors=cleanup, shutdown_clean=not cleanup,
                      launch_return_code=None if process is None else process.returncode,
                      console_session=None if session is None else str(session))
    print(report.data["status"].upper() + ": " + str(report.path), flush=True)
    if report.data["error"]:
        print(report.data["error"], flush=True)
    if cleanup:
        print(cleanup, flush=True)
    return 1 if failed or cleanup else 0


if __name__ == "__main__":
    raise SystemExit(main())
