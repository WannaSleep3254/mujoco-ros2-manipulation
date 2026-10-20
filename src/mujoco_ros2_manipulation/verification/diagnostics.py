"""Fail-closed native step reports and a paced, steady ROS process benchmark."""

import json
import math
import os
from pathlib import Path
import platform
import time

from mujoco_ros2_manipulation.model_build.integrity import file_sha256


def diagnostic_environment(report, args):
    return dict(
        os.environ,
        MANIPULATION_DIAGNOSTICS_PATH=str(report.directory / "physics.json"),
        MANIPULATION_DIAGNOSTICS_RUN_ID=report.run_id,
    )


def check_step_coverage(data, *, run_id, source_sha256, timestep):
    """Any detected contact fails the free-space workcell test, including inactive contacts."""

    def require(condition, message):
        if not condition:
            raise ValueError(message)

    def finite(value):
        return type(value) in (float, int) and math.isfinite(value)

    require(
        data.get("schema_version") == 1 and data.get("complete") is True,
        "Incomplete native physics report",
    )
    require(data.get("run_id") == run_id, "Physics report belongs to another run")
    require(
        data.get("monitor_source_sha256") == source_sha256,
        "Rebuild native runtime: stale step monitor",
    )
    steps = data.get("steps")
    require(
        type(steps) is int and steps > 0 and steps == data.get("engine_steps"),
        "Physics step coverage mismatch",
    )
    for name in (
        "timestep_sec",
        "simulated_sec",
        "wall_span_sec",
        "paced_real_time_factor",
    ):
        require(
            finite(data.get(name)) and data[name] > 0, "Invalid physics timing: " + name
        )
    require(
        abs(data["timestep_sec"] - timestep) < 1e-12,
        "Physics timestep differs from workcell",
    )
    require(
        abs(data["simulated_sec"] - steps * timestep) < max(1e-7, steps * 1e-10),
        "Missing or discontinuous physics steps",
    )
    for name in ("invalid_values", "time_discontinuities", "pair_overflow"):
        require(data.get(name) == 0, "Native monitor diagnostic failure: " + name)
    for name in ("physics_step", "monitor_step"):
        timing = data.get(name, {})
        require(timing.get("count") == steps, "Timing coverage mismatch: " + name)
        for key in ("mean_sec", "max_sec"):
            require(
                finite(timing.get(key)) and timing[key] >= 0,
                "Invalid step timing: " + name,
            )
    return data


def check_physics(data, *, run_id, source_sha256, timestep, require_interlock=False):
    check_step_coverage(
        data, run_id=run_id, source_sha256=source_sha256, timestep=timestep
    )
    if require_interlock and (
        data.get("stop_enabled") is not True or data.get("stop_latched") is not False
    ):
        raise ValueError("Native contact stop interlock is not ready")
    if not (
        data.get("contacts") == 0
        and data.get("contact_steps") == 0
        and data.get("active_contacts") == 0
        and data.get("pairs") == []
    ):
        raise ValueError("Unexpected MuJoCo contacts during free-space execution")
    return data


def check_stop_physics(data, *, run_id, source_sha256, timestep):
    check_step_coverage(
        data, run_id=run_id, source_sha256=source_sha256, timestep=timestep
    )
    if not (
        data.get("stop_enabled") is True
        and data.get("stop_latched") is True
        and data.get("contact_steps") == 1
        and data.get("contacts", 0) > 0
        and data.get("active_contacts", 0) > 0
        and data.get("stop_step") == data["steps"]
        and data.get("hold_ticks", 0) >= 10
        and data.get("hold_time_sec", 0) > data.get("stop_time_sec", 0)
        and data.get("stop_time_sec") == data["last_state_time_sec"]
        and len(data.get("stopped_qpos", [])) > 0
        and all(math.isfinite(v) for v in data["stopped_qpos"])
        and sum(p["occurrences"] for p in data.get("pairs", [])) == data["contacts"]
    ):
        raise ValueError("Contact did not produce a complete latched stop")
    return data


def check_clock_stop_physics(data, *, run_id, source_sha256, timestep):
    check_step_coverage(
        data, run_id=run_id, source_sha256=source_sha256, timestep=timestep
    )
    if not (
        data.get("stop_enabled") is True
        and data.get("stop_latched") is True
        and data.get("stop_reason") == "clock_stall"
        and data.get("contact_steps") == 0
        and data.get("contacts") == 0
        and data.get("active_contacts") == 0
        and data.get("pairs") == []
        and data.get("stop_step") == data["steps"]
        and data.get("hold_ticks", 0) >= 10
        and data.get("hold_time_sec", 0) > data.get("stop_time_sec", 0)
        and data.get("stop_time_sec") == data["last_state_time_sec"]
        and len(data.get("stopped_qpos", [])) > 0
        and all(math.isfinite(v) for v in data["stopped_qpos"])
    ):
        raise ValueError(
            "Clock stall did not produce a complete zero-contact latched stop"
        )
    return data


def finalize_physics(report, cell, args):
    path = report.directory / "physics.json"
    data = json.loads(path.read_text())
    # Keep contact pairs and timing evidence even when the zero-contact policy fails.
    report.update(physics=data, physics_report_sha256=file_sha256(path))
    stop_test = getattr(args, "contact_stop_test", False)
    clock_test = getattr(args, "clock_stop_test", False)
    validator = (
        check_clock_stop_physics
        if clock_test
        else (check_stop_physics if stop_test else check_physics)
    )
    options = {} if stop_test or clock_test else {"require_interlock": True}
    validator(
        data,
        run_id=report.run_id,
        source_sha256=file_sha256(cell.root / "native/step_monitor.hpp"),
        timestep=cell.data["physics"]["timestep"],
        **options,
    )
    if stop_test:
        fixture = report.directory / "contact_test.xml"
        if not fixture.is_file():
            raise ValueError("Contact test fixture missing")
        report.update(contact_test_model_sha256=file_sha256(fixture))
    report.update(physics_checks_passed=True)


def process_sample(group, proc=Path("/proc")):
    """RSS is summed (shared pages may count twice); CPU excludes the verifier."""
    result = {}
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            if int(fields[2]) == group:
                result[int(entry.name)] = dict(
                    cpu_ticks=int(fields[11]) + int(fields[12]),
                    rss_bytes=int(fields[21]) * os.sysconf("SC_PAGE_SIZE"),
                )
        except (OSError, ValueError, IndexError):
            continue
    return result


def benchmark(node, duration):
    """Measure stationary holding after motion; not an unpaced throughput or FPS benchmark."""
    warmup = time.monotonic()
    node.until(lambda: time.monotonic() - warmup >= 5, 10, "performance warmup")
    start_joints, start_clocks = node.joint_count, node.clock_count
    start_wall = time.monotonic()
    start_clock = node.clock_last
    first = process_sample(node.process_group)
    samples = [first]
    last_sample = [start_wall]

    def elapsed():
        now = time.monotonic()
        if now - last_sample[0] >= 0.2:
            samples.append(process_sample(node.process_group))
            last_sample[0] = now
        return now - start_wall >= duration

    node.until(elapsed, duration + 5, "steady holding performance window")
    wall = time.monotonic() - start_wall
    simulated = node.clock_last - start_clock
    final = process_sample(node.process_group)
    samples.append(final)
    if not first or set(first) != set(final):
        raise RuntimeError("ROS process set changed during performance window")
    cpu = sum(
        final[pid]["cpu_ticks"] - first[pid]["cpu_ticks"] for pid in first
    ) / os.sysconf("SC_CLK_TCK")
    if simulated <= 0 or cpu < 0:
        raise RuntimeError("Invalid simulation clock or process timing in benchmark")
    return dict(
        kind="paced stationary holding after verified motion",
        requested_wall_sec=duration,
        warmup_wall_sec=5,
        joint_state_received_hz=(node.joint_count - start_joints) / wall,
        clock_received_hz=(node.clock_count - start_clocks) / wall,
        wall_sec=wall,
        simulated_sec=simulated,
        real_time_factor=simulated / wall,
        cpu_core_equivalents=cpu / wall,
        rss_sum_peak_bytes=max(
            sum(p["rss_bytes"] for p in sample.values()) for sample in samples
        ),
        sample_count=len(samples),
        process_ids=sorted(first),
        cpu_scope="launch process group, excludes verifier; 1.0 = one full CPU core",
        memory_scope="sum of process RSS; shared pages can be counted more than once",
    )


def host_info():
    cpu = next(
        (
            line.split(":", 1)[1].strip()
            for line in Path("/proc/cpuinfo").read_text().splitlines()
            if line.startswith("model name")
        ),
        platform.machine(),
    )
    memory = next(
        line
        for line in Path("/proc/meminfo").read_text().splitlines()
        if line.startswith("MemTotal:")
    )
    return dict(
        cpu=cpu,
        logical_cpus=os.cpu_count(),
        memory_total_bytes=int(memory.split()[1]) * 1024,
        kernel=platform.release(),
        platform=platform.platform(),
    )
