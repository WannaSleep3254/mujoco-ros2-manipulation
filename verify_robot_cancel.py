#!/usr/bin/env python3
"""Compatibility entry point for owned single-robot cancellation verification."""
import _bootstrap
from mujoco_ros2_manipulation.verification.verify_robot_cancel import main

if __name__ == "__main__":
    raise SystemExit(main())
