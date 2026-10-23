"""Compatibility entry point for the local environment design procedure."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from mujoco_ros2_manipulation.workcells.import_step import main

if __name__ == "__main__":
    raise SystemExit(main())
