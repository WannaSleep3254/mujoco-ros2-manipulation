# Source this file in each terminal participating in the robot simulation.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
task_manipulation_env_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${task_manipulation_env_root}/ros2_env.sh"
export ROS_DOMAIN_ID="${MANIPULATION_ROS_DOMAIN_ID:-89}"
export ROS_LOG_DIR=/tmp/manipulation_ros2_logs
task_manipulation_env_runtime="${task_manipulation_env_root}/runtime/root/opt/ros/humble"
export AMENT_PREFIX_PATH="${task_manipulation_env_runtime}:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${task_manipulation_env_runtime}/lib:${task_manipulation_env_runtime}/opt/mujoco_vendor/lib:${task_manipulation_env_root}/runtime/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${task_manipulation_env_runtime}/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
if [[ -f "${task_manipulation_env_root}/ros_ws/install/local_setup.bash" ]]; then
  source "${task_manipulation_env_root}/ros_ws/install/local_setup.bash"
fi
unset task_manipulation_env_root task_manipulation_env_runtime
