#!/usr/bin/env python3
"""Compatibility entry point for the fixed-motion evaluation suite."""
import _bootstrap
from mujoco_ros2_manipulation.workcells.benchmark_fixed_motion import main

if __name__ == "__main__":
    raise SystemExit(main())
