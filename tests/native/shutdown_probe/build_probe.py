"""Build only the test interposer, never the simulation runtime or ROS workspace."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

SYMBOL = "_ZN27joint_trajectory_controller25JointTrajectoryController6updateERKN6rclcpp4TimeERKNS1_8DurationE"


def build(output):
    source = Path(__file__).with_name("update_probe.cpp").resolve()
    root = source.parents[3]
    output = Path(output).resolve()
    if output.exists() or not output.is_relative_to(root / "runtime"):
        raise ValueError("Select a new output directory under this workspace runtime/")
    jtc = Path("/opt/ros/humble/lib/libjoint_trajectory_controller.so")
    version = subprocess.check_output(["dpkg-query", "-W", "-f=${Version}",
        "ros-humble-joint-trajectory-controller"], text=True).strip()
    if not version.startswith("2.54.1-"):
        raise ValueError("Probe ABI is reviewed only for installed Humble JTC 2.54.1")
    exported = subprocess.check_output(["nm", "-D", "--defined-only", str(jtc)], text=True)
    if not any(line.split()[-1] == SYMBOL for line in exported.splitlines()):
        raise ValueError("Expected actual JTC update symbol is absent")
    output.mkdir(parents=True)
    library = output / "update_probe.so"
    command = ["c++", "-std=c++17", "-O2", "-g", "-fPIC", "-shared", "-Wall", "-Wextra", "-Werror",
        str(source), "-o", str(library), "-ldl", "-pthread"]
    subprocess.run(command, check=True, timeout=60)
    # Loading the actual installed plugin is not starting ROS or calling update.
    # Preloading it after the shim also makes RTLD_NEXT resolution explicit even
    # if a plugin loader would otherwise use a local symbol group.
    environment = dict(os.environ, LD_PRELOAD=os.pathsep.join((str(library), str(jtc))))
    subprocess.run(["/usr/bin/python3", "-c",
        "import ctypes; p=ctypes.CDLL(None); assert p.manipulation_shutdown_probe_resolves()==1"],
        env=environment, check=True, timeout=15)
    files = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in (source, library, jtc)}
    receipt = dict(schema_version=1, scope="Test interposer build and real-symbol resolution only; no ROS launch/update",
        installed_jtc_version=version, symbol=SYMBOL, return_abi="uint8_t; opaque this/const Time&/const Duration&",
        command=command, sha256=files)
    (output / "build.json").write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n")
    print(library)
    return library


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    build(parser.parse_args().output)
