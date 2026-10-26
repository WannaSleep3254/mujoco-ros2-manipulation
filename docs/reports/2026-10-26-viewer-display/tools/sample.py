#!/usr/bin/env python3
"""Read-only loopback /api/state and owned-console /proc sampler.

No browser control, token, HTTP mutations, process signals, or ROS imports.
CPU is Linux process utime+stime delta / monotonic seconds * 100 (one core=100%).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import time
from datetime import datetime, timezone
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

SCOPES = ("console", "scene_worker", "render_worker", "simulation", "commands", "other_descendant")
MAX_STATE_BYTES = 4 * 1024 * 1024
MAX_PHASE_BYTES = 4096
ID_RE = re.compile(r"[A-Za-z][A-Za-z0-9_-]{0,31}\Z")
STOP = False


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def finite_number(value):
    return value if type(value) in (int, float) and math.isfinite(value) and value >= 0 else None


def natural(value):
    return value if type(value) is int and value >= 0 else None


def label(value):
    return value if isinstance(value, str) and ID_RE.fullmatch(value) else None


def proc_stat(pid):
    """Fields 14/15/22/24; comm may contain spaces or parentheses."""
    raw = Path(f"/proc/{pid}/stat").read_text()
    fields = raw[raw.rfind(")") + 2:].split()
    return {"pid": pid, "ppid": int(fields[1]), "utime_ticks": int(fields[11]),
            "stime_ticks": int(fields[12]), "start_ticks": int(fields[19]),
            "rss_pages": max(0, int(fields[21]))}


def role(argv):
    for arg in argv:
        name = arg.rsplit("/", 1)[-1]
        if name == "scene_worker.py" or arg.endswith(".operator_console.scene_worker"):
            return "scene_worker"
        if name == "render_worker.py" or arg.endswith(".operator_console.render_worker"):
            return "render_worker"
        if name == "motion_server.py" or arg.endswith(".workcells.motion_server"):
            return "commands"
        if name == "run_scene.sh":
            return "simulation"
    return None


def owned_snapshot(root_pid, expected_start, page_size):
    # Read parent identity first/last. Never follow a reused console PID.
    first = proc_stat(root_pid)
    if first["start_ticks"] != expected_start:
        raise RuntimeError("console_pid_reused")
    stats = {}
    for entry in Path("/proc").iterdir():
        if entry.name.isdecimal():
            try:
                stats[int(entry.name)] = proc_stat(int(entry.name))
            except (OSError, ValueError, IndexError):
                pass  # Processes may exit during enumeration.
    root = stats.get(root_pid)
    if root is None or root["start_ticks"] != expected_start:
        raise RuntimeError("console_identity_lost")
    descendants = {root_pid: 0}
    while True:
        additions = {pid: descendants[s["ppid"]] + 1 for pid, s in stats.items()
                     if pid not in descendants and s["ppid"] in descendants}
        if not additions:
            break
        descendants.update(additions)
    records, assigned = {}, {root_pid: "console"}
    for pid in sorted(descendants, key=lambda p: (descendants[p], p)):
        if pid == os.getpid():
            continue
        item = stats[pid]
        try:
            argv = Path(f"/proc/{pid}/cmdline").read_bytes().decode("utf-8", "replace").split("\0")
            current = proc_stat(pid)
            if (current["start_ticks"], current["ppid"]) != (item["start_ticks"], item["ppid"]):
                continue
        except (OSError, ValueError, IndexError):
            continue
        inherited = assigned.get(item["ppid"], "other_descendant")
        scope = "console" if pid == root_pid else role(argv) or (
            inherited if inherited != "console" else "other_descendant")
        assigned[pid] = scope
        records[(pid, item["start_ticks"])] = dict(item, scope=scope,
            rss_bytes=item.pop("rss_pages") * page_size)
    last = proc_stat(root_pid)
    if last["start_ticks"] != expected_start:
        raise RuntimeError("console_pid_reused")
    return records


def read_phase(path, default_window_sec):
    try:
        payload = path.read_bytes()
        if len(payload) > MAX_PHASE_BYTES:
            return {"error": "phase_too_large"}
        data = json.loads(payload)
        if not isinstance(data, dict) or not set(data) <= {"mode", "round", "window_start", "window_duration_sec"}:
            return {"error": "phase_schema"}
        mode, rnd, start = data.get("mode"), data.get("round"), data.get("window_start")
        if label(mode) is None or (rnd is not None and (type(rnd) is not int or not 1 <= rnd <= 1000)):
            return {"error": "phase_labels"}
        if type(start) in (int, float) and finite_number(start) is not None:
            start_unix = float(start)
        elif isinstance(start, str) and len(start) <= 48:
            parsed = datetime.fromisoformat(start.replace("Z", "+00:00"))
            if parsed.tzinfo is None:
                return {"error": "phase_start_requires_timezone"}
            start_unix = parsed.timestamp()
        else:
            return {"error": "phase_start"}
        duration = data.get("window_duration_sec", default_window_sec)
        if finite_number(duration) is None or not 1 <= duration <= 300:
            return {"error": "phase_window"}
        return {"mode": mode, "round": rnd, "window_start": start,
                "window_start_unix": start_unix, "window_duration_sec": duration,
                "sha256": hashlib.sha256(payload).hexdigest()}
    except FileNotFoundError:
        return {"error": "phase_missing"}
    except (OSError, ValueError, TypeError, OverflowError):
        return {"error": "phase_invalid"}


def same_phase(a, b):
    return bool(a.get("sha256")) and a.get("sha256") == b.get("sha256")


def in_window(phase, start, end):
    if "error" in phase:
        return False
    return phase["window_start_unix"] <= start <= end <= phase["window_start_unix"] + phase["window_duration_sec"]


class NoRedirects(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def api_state(opener, url, timeout):
    begin = time.monotonic()
    result = {"ok": False, "http_status": None, "response_body_bytes": 0,
              "response_complete": False, "latency_sec": None, "error": None}
    try:
        request = Request(url + "/api/state", method="GET", headers={"Accept": "application/json", "Accept-Encoding": "identity", "X-Viewer-Benchmark-Probe": "1"})
        with opener.open(request, timeout=timeout) as response:
            result["http_status"] = response.status
            body = response.read(MAX_STATE_BYTES + 1)
        result["response_body_bytes"] = len(body)
        if len(body) > MAX_STATE_BYTES:
            result["error"] = "state_too_large"
            return result
        result["response_complete"] = True
        data = json.loads(body)
        if not isinstance(data, dict):
            result["error"] = "state_schema"
            return result
        frame = data.get("frame") if isinstance(data.get("frame"), dict) else {}
        view = data.get("view") if isinstance(data.get("view"), dict) else {}
        binding = view.get("binding_id")
        binding = binding if isinstance(binding, str) and re.fullmatch(r"[0-9a-f]{64}", binding) else None
        result.update(ok=response.status == 200,
            state_age_sec=finite_number(data.get("state_age_sec")),
            simulation_sec=finite_number(data.get("simulation_sec")),
            active_request_present=data.get("active_request_id") is not None,
            frame={"sequence": natural(frame.get("sequence")), "age_sec": finite_number(frame.get("age_sec")),
                   "source_state_age_sec": finite_number(frame.get("source_state_age_sec")),
                   "simulation_sec": finite_number(frame.get("simulation_sec"))},
            view={"binding_id": binding, "mode": label(view.get("mode")),
                  "display_mode": label(view.get("display_mode")), "status": label(view.get("status")),
                  "sequence": natural(view.get("sequence")), "pose_ready": view.get("pose_ready") is True,
                  "source_state_age_sec": finite_number(view.get("source_state_age_sec")),
                  "source_age_available": "source_state_age_sec" in view})
    except HTTPError as error:
        result.update(http_status=error.code, error="http_error")
        error.close()  # Do not retain or disclose error bodies.
    except (URLError, TimeoutError, OSError):
        result["error"] = "transport_error"
    except (ValueError, TypeError):
        result["error"] = "invalid_json"
    finally:
        result["latency_sec"] = time.monotonic() - begin
    return result


def cpu_records(current, previous, elapsed, hz):
    rows, grouped = [], {scope: {"process_count": 0, "rss_bytes": 0,
        "cpu_time_sec_delta": 0.0, "cpu_percent_one_core": 0.0,
        "cpu_measured_processes": 0, "cpu_unmeasured_processes": 0} for scope in SCOPES}
    for key, value in current.items():
        row = dict(value)
        old = previous.get(key) if previous is not None else None
        delta = None
        if old is not None and elapsed is not None and elapsed > 0:
            ticks = value["utime_ticks"] + value["stime_ticks"] - old["utime_ticks"] - old["stime_ticks"]
            if ticks >= 0:
                delta = ticks / hz
        row["cpu_time_sec_delta"] = delta
        row["cpu_percent_one_core"] = None if delta is None else delta / elapsed * 100
        rows.append(row)
        group = grouped[row["scope"]]
        group["process_count"] += 1
        group["rss_bytes"] += row["rss_bytes"]
        group["cpu_unmeasured_processes" if delta is None else "cpu_measured_processes"] += 1
        if delta is not None:
            group["cpu_time_sec_delta"] += delta
            group["cpu_percent_one_core"] += row["cpu_percent_one_core"]
    # Null first/new-only scope CPU rather than inventing zero at process creation.
    for group in grouped.values():
        if group["process_count"] > 0 and group["cpu_measured_processes"] == 0:
            group["cpu_time_sec_delta"] = group["cpu_percent_one_core"] = None
    grouped["total"] = {
        "process_count": sum(g["process_count"] for g in grouped.values()),
        "rss_bytes": sum(g["rss_bytes"] for g in grouped.values()),
        "cpu_time_sec_delta": sum(g["cpu_time_sec_delta"] or 0 for g in grouped.values()),
        "cpu_percent_one_core": sum(g["cpu_percent_one_core"] or 0 for g in grouped.values()),
        "cpu_measured_processes": sum(g["cpu_measured_processes"] for g in grouped.values()),
        "cpu_unmeasured_processes": sum(g["cpu_unmeasured_processes"] for g in grouped.values())}
    if grouped["total"]["cpu_measured_processes"] == 0:
        grouped["total"]["cpu_time_sec_delta"] = grouped["total"]["cpu_percent_one_core"] = None
    return rows, grouped


def distribution(values):
    values = sorted(v for v in values if v is not None)
    if not values:
        return {"n": 0, "mean": None, "p50": None, "p95": None, "max": None}
    return {"n": len(values), "mean": statistics.fmean(values),
            "p50": statistics.median(values), "p95": values[max(0, math.ceil(0.95 * len(values)) - 1)], "max": values[-1]}


def summarize(samples, reason, metadata):
    windows = {}
    for sample in samples:
        phase = sample["phase"]
        if "error" in phase:
            continue
        key = (phase["mode"], phase["round"], str(phase["window_start"]))
        windows.setdefault(key, []).append(sample)
    results = []
    for (mode, rnd, start), rows in windows.items():
        eligible = [r for r in rows if r["window_eligible"]]
        cpu = [r for r in rows if r["cpu_window_eligible"]]
        scopes = {}
        for scope in (*SCOPES, "total"):
            valid = [r for r in cpu if r["scopes"][scope]["cpu_time_sec_delta"] is not None]
            duration = sum(r["cpu_interval_sec"] for r in valid)
            scopes[scope] = {"cpu_observed_interval_sec": duration,
                "cpu_percent_one_core_time_weighted_mean": None if not duration else
                    sum(r["scopes"][scope]["cpu_time_sec_delta"] for r in valid) / duration * 100,
                "cpu_percent_one_core_sampled": distribution([r["scopes"][scope]["cpu_percent_one_core"] for r in valid]),
                "rss_bytes_sampled": distribution([r["scopes"][scope]["rss_bytes"] for r in eligible]),
                "process_count_sampled": distribution([r["scopes"][scope]["process_count"] for r in eligible]),
                "cpu_unmeasured_process_observations": sum(r["scopes"][scope]["cpu_unmeasured_processes"] for r in eligible)}
        api = [r["api"] for r in eligible]
        successful = [a for a in api if a["ok"]]
        results.append({"mode": mode, "round": rnd, "window_start": start,
            "sample_count": len(eligible), "cpu_interval_count": len(cpu), "scopes": scopes,
            "api": {"request_count": len(api), "success_count": len(successful), "error_count": len(api)-len(successful),
                "response_body_bytes_sum": sum(a["response_body_bytes"] for a in api),
                "latency_sec": distribution([a["latency_sec"] for a in api]),
                "state_age_sec": distribution([a.get("state_age_sec") for a in successful]),
                "frame_source_state_age_sec": distribution([a["frame"]["source_state_age_sec"] for a in successful]),
                "view_source_state_age_sec": distribution([a["view"]["source_state_age_sec"] for a in successful]),
                "display_modes_observed": sorted({a["view"]["display_mode"] for a in successful if a["view"]["display_mode"]}),
                "active_request_present_samples": sum(a["active_request_present"] for a in successful)}})
    return {"schema_version": 1, "record_type": "summary", "completed_utc": utc_now(),
        "stop_reason": reason, "sample_count": len(samples), "metadata": metadata, "windows": results,
        "limitations": ["Backend process scope excludes browser CPU, GPU, VRAM, WebGL FPS and browser network traffic.",
            "API body byte totals are sampler GET traffic only; headers/TCP and browser traffic are not measured.",
            "Summed process RSS can double-count shared pages; sampled peaks are not continuous maxima.",
            "Short-lived/exited/reparented processes between scans can be missed; first/new process CPU is unmeasured.",
            "No motion commands or HTTP mutations are issued; user supplies display phase windows.",
            "View source age is unavailable when /api/state does not expose it; no extra pose polling is performed."]}


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--pid", required=True, type=int, help="Exact console PID; start_ticks is pinned before sampling")
    p.add_argument("--url", default="http://127.0.0.1:8892")
    p.add_argument("--duration", type=float, default=300, help="Total wall duration, 60..300 sec")
    p.add_argument("--interval", type=float, default=0.5, help="Sampling interval, 0.5..1 sec")
    p.add_argument("--timeout", type=float, default=0.4, help="GET timeout, 0.1..1 sec")
    p.add_argument("--window-seconds", type=float, default=30)
    p.add_argument("--phase", type=Path, default=Path(__file__).with_name("phase.json"))
    p.add_argument("--output", type=Path, default=Path(__file__).with_name("samples.jsonl"))
    a = p.parse_args(argv)
    parsed = urlsplit(a.url)
    if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "::1") or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in ("", "/"):
        p.error("--url must be a literal-loopback http origin without credentials/path/query")
    try:
        port = parsed.port
    except ValueError:
        p.error("invalid URL port")
    if port is None or not 1024 <= port <= 65535:
        p.error("explicit nonprivileged loopback port required")
    a.url = a.url.rstrip("/")
    if a.pid <= 1 or a.pid == os.getpid():
        p.error("--pid must identify a separate console process")
    if not 60 <= a.duration <= 300 or not 0.5 <= a.interval <= 1 or not 0.1 <= a.timeout <= 1 or not 1 <= a.window_seconds <= 300:
        p.error("duration/interval/timeout/window outside permitted bounds")
    return a


def main(argv=None):
    global STOP
    a = arguments(argv)
    root = proc_stat(a.pid)
    start_ticks = root["start_ticks"]
    # The human supplies the exact owned PID, including instrumented launchers.
    # Pinning start_ticks prevents a PID replacement from silently entering scope.
    if Path(f"/proc/{a.pid}").stat().st_uid != os.getuid():
        raise SystemExit("Console PID must belong to the sampler user")
    hz, pages = os.sysconf("SC_CLK_TCK"), os.sysconf("SC_PAGE_SIZE")
    summary_path = a.output.with_suffix(".summary.json")
    if a.output.exists() or summary_path.exists():
        raise SystemExit("Refusing to overwrite existing sample/summary files")
    metadata = {"schema_version": 1, "record_type": "metadata", "started_utc": utc_now(),
        "console_pid": a.pid, "console_start_ticks": start_ticks, "sampler_pid": os.getpid(),
        "endpoint": a.url + "/api/state", "duration_limit_sec": a.duration, "interval_sec": a.interval,
        "http_timeout_sec": a.timeout, "default_window_sec": a.window_seconds,
        "clock_ticks_per_sec": hz, "page_size_bytes": pages, "logical_cpu_count": os.cpu_count(),
        "cpu_basis": "100 percent equals one fully occupied core; no division by logical CPU count",
        "scope": "Exact pinned console and current /proc descendants only; role inherited from known worker/launch entry points",
        "traffic_basis": "Sampler GET response body bytes only; not browser transfer volume",
        "cpu_clock": "time.monotonic interval; wall timestamps only label user windows",
        "process_cmdlines_retained": False}
    opener = build_opener(ProxyHandler({}), NoRedirects())
    def stop(*_):
        global STOP
        STOP = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)  # Handler only stops this sampler; never signals another PID.
    samples, previous, previous_at, previous_wall, previous_phase = [], None, None, None, {}
    beginning, deadline = time.monotonic(), time.monotonic() + a.duration
    scheduled, reason = beginning, "duration_limit"
    with a.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(metadata, allow_nan=False) + "\n"); stream.flush()
        while not STOP and time.monotonic() < deadline:
            phase = read_phase(a.phase, a.window_seconds)
            try:
                current = owned_snapshot(a.pid, start_ticks, pages)
            except (OSError, ValueError, IndexError, RuntimeError):
                reason = "console_identity_lost_or_unreadable"
                break
            proc_at, wall_at = time.monotonic(), time.time()
            elapsed = None if previous_at is None else proc_at - previous_at
            processes, scopes = cpu_records(current, previous, elapsed, hz)
            api = api_state(opener, a.url, a.timeout)
            phase_after = read_phase(a.phase, a.window_seconds)
            stable = same_phase(phase, phase_after)
            sample = {"schema_version": 1, "record_type": "sample", "sample_index": len(samples),
                "sample_utc": datetime.fromtimestamp(wall_at, timezone.utc).isoformat(),
                "wall_unix_sec": wall_at, "elapsed_sec": proc_at-beginning,
                "schedule_lag_sec": max(0, proc_at-scheduled), "phase": phase,
                "phase_stable_during_request": stable,
                "window_eligible": stable and in_window(phase, wall_at, time.time()),
                "cpu_window_eligible": bool(stable and elapsed is not None and same_phase(previous_phase, phase)
                    and in_window(phase, previous_wall, wall_at)),
                "cpu_interval_sec": elapsed, "processes": processes, "scopes": scopes,
                "previous_processes_not_seen": [] if previous is None else [
                    {"pid": k[0], "start_ticks": k[1]} for k in previous.keys()-current.keys()],
                "api": api}
            stream.write(json.dumps(sample, allow_nan=False) + "\n"); stream.flush()
            samples.append(sample)
            previous, previous_at, previous_wall, previous_phase = current, proc_at, wall_at, phase if stable else {}
            scheduled += a.interval
            now = time.monotonic()
            if scheduled < now:
                scheduled += (math.floor((now-scheduled)/a.interval)+1)*a.interval
            time.sleep(max(0, min(scheduled, deadline)-time.monotonic()))
        if STOP:
            reason = "sampler_stop_requested"
        summary = summarize(samples, reason, metadata)
        stream.write(json.dumps(summary, allow_nan=False) + "\n"); stream.flush()
    with summary_path.open("x", encoding="utf-8") as out:
        json.dump(summary, out, indent=2, allow_nan=False); out.write("\n")
    print(json.dumps({"samples": str(a.output), "summary": str(summary_path), "sample_count": len(samples), "stop_reason": reason}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
