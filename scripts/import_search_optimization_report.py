"""Import search equivalence/GUI evidence into the common publication report."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src"))
from mujoco_ros2_manipulation.reporting.search_optimization import main

if __name__=="__main__":main()
