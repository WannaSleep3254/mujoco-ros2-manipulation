"""Compare full-grid and pruned departure search using unchanged frozen inputs."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from mujoco_ros2_manipulation.workcells.search_comparison import main

if __name__=="__main__":
    main()
