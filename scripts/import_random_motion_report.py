"""Import the complete paired random-motion benchmark into the common report format."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mujoco_ros2_manipulation.reporting.random_motion import main

if __name__ == "__main__":
    main()
