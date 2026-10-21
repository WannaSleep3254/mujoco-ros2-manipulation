"""Reproduce the phase-one standard JSON from preserved evidence, without ROS."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mujoco_ros2_manipulation.reporting.data import json_text, ReportError
from mujoco_ros2_manipulation.reporting.phase1 import phase1_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    directory = Path(__file__).resolve().parents[1] / "docs/reports/2026-10-21-phase1"
    target = directory / "report.json"
    expected = json_text(phase1_report(directory))
    if target.exists():
        if target.read_text() != expected:
            raise ReportError("Existing report.json differs; preserve it and review a new revision")
    elif args.check:
        raise ReportError("Missing standard report.json")
    else:
        target.write_text(expected, encoding="utf-8")
    print("Phase-one standard JSON matches preserved evidence")


if __name__ == "__main__":
    main()
