"""Compatibility launch adapter."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / 'src'))
from mujoco_ros2_manipulation.ros_launch import generate_launch_description
