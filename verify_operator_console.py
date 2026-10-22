"""Compatibility entry point for the actual operator HTTP/ROS regression."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from mujoco_ros2_manipulation.operator_console.verify import main
if __name__ == "__main__":
    raise SystemExit(main())
