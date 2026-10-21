"""Publish or check a complete persistent-command regression snapshot."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from mujoco_ros2_manipulation.reporting.motion_requests import main

if __name__ == "__main__":
    main()
