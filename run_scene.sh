#!/usr/bin/env bash
set -e
task_workcell_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${task_workcell_root}/manipulation_env.sh"
cd "$task_workcell_root"
exec ros2 launch "${task_workcell_root}/launch/workcell.launch.py" "$@"
