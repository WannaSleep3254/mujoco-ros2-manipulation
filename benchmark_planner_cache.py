#!/usr/bin/env python3
"""Compatibility entry point for the separate opt-in planner/cache experiment."""
import _bootstrap
from mujoco_ros2_manipulation.workcells.benchmark_planner_cache import main

if __name__ == "__main__":
    raise SystemExit(main())
