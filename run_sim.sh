#!/usr/bin/env bash
set -e
task_manipulation_launch_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${task_manipulation_launch_root}/manipulation_env.sh"
cd "${task_manipulation_launch_root}"
exec ros2 launch "${task_manipulation_launch_root}/manipulation.launch.py" "$@"
