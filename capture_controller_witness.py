#!/usr/bin/env python3
"""Passive, bounded controller-state/action-status recorder."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from mujoco_ros2_manipulation.workcells.controller_witness import main

if __name__ == "__main__":
    raise SystemExit(main())
