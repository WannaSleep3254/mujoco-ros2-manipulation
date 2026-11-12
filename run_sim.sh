#!/usr/bin/env bash
set -e
task_manipulation_launch_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${task_manipulation_launch_root}/manipulation_env.sh"
cd "${task_manipulation_launch_root}"
exec python3 -m mujoco_ros2_manipulation.launch_cli launch "${task_manipulation_launch_root}/launch/manipulation.launch.py" "$@"
