"""Checkout entry point for deployed report byte verification."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mujoco_ros2_manipulation.reporting.published import main

if __name__ == "__main__":
    raise SystemExit(main())
