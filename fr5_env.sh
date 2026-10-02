# Source this file in each terminal participating in the FR5 simulation.
# Use the installed Humble packages instead of unrelated login-shell overlays.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
task_fr5_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${task_fr5_root}/ros2_env.sh"
export ROS_DOMAIN_ID="${FR5_ROS_DOMAIN_ID:-89}"
export ROS_LOG_DIR=/tmp/fr5_ros2_logs
task_fr5_runtime="${task_fr5_root}/runtime/root/opt/ros/humble"
export AMENT_PREFIX_PATH="${task_fr5_runtime}:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${task_fr5_runtime}/lib:${task_fr5_runtime}/opt/mujoco_vendor/lib:${task_fr5_root}/runtime/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${task_fr5_runtime}/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
source "${task_fr5_root}/ros_ws/install/local_setup.bash"
unset task_fr5_root task_fr5_runtime
