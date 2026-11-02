# Source this file in every terminal used for the MuJoCo ROS 2 demo.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=87
export ROS_LOCALHOST_ONLY=1
task_ros2_log_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ROS_LOG_DIR="${task_ros2_log_root}/runtime/ros-logs"
mkdir -p "$ROS_LOG_DIR"
unset task_ros2_log_root
