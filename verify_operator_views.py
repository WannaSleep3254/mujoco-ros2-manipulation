"""Compatibility entry point for actual read-only local 3D/ROS verification."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from mujoco_ros2_manipulation.operator_console.verify_views import main
if __name__ == "__main__":
    raise SystemExit(main())
