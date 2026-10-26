"""Export one fixed benchmark as audited public evidence."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from mujoco_ros2_manipulation.workcells.export_fixed_motion import main

if __name__ == "__main__":
    raise SystemExit(main())
