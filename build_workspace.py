"""Compatibility checkout command for manifest-driven builds."""
import _bootstrap
from mujoco_ros2_manipulation.build import main

if __name__ == '__main__':
    raise SystemExit(main())
