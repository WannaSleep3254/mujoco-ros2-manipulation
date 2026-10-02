#!/usr/bin/env bash
set -e
task_fr5_launch_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${task_fr5_launch_root}/fr5_env.sh"
cd "${task_fr5_launch_root}"
exec ros2 launch "${task_fr5_launch_root}/fr5.launch.py" "$@"
