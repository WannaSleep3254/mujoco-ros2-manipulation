# Source independently of ROS so the middleware policy is testable without ROS.
export RMW_IMPLEMENTATION="${MANIPULATION_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "$RMW_IMPLEMENTATION" == "rmw_cyclonedds_cpp" && -z "${CYCLONEDDS_URI:-}" ]]; then
  task_manipulation_middleware_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
  export CYCLONEDDS_URI="file://${task_manipulation_middleware_root}/config/ros/cyclonedds.xml"
  unset task_manipulation_middleware_root
fi
