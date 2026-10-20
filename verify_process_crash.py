"""Verify bounded cleanup after an owned workcell process crashes."""
import _bootstrap
from mujoco_ros2_manipulation.workcells.verify_process_crash import main

if __name__ == "__main__":
    raise SystemExit(main())
