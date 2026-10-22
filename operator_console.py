"""Compatibility entry point for the local simulation operator console."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from mujoco_ros2_manipulation.operator_console.__main__ import main

if __name__ == "__main__":
    raise SystemExit(main())
