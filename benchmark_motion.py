"""Source-checkout entry point for the paired random-goal benchmark."""
import _bootstrap
from mujoco_ros2_manipulation.workcells.benchmark_motion import main

if __name__ == "__main__":
    raise SystemExit(main())
