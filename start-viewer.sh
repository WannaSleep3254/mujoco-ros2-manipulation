#!/usr/bin/env bash
set -e
viewer_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$viewer_root"
mkdir -p runtime
viewer_port="${VIEWER_PORT:-8933}"
viewer_domain="${VIEWER_ROS_DOMAIN:-218}"
[[ "$viewer_port" =~ ^[0-9]{4,5}$ ]] || { echo "Invalid local port" >&2; exit 1; }
exec 9>"runtime/viewer-${viewer_port}.lock"
if ! flock -n 9; then
  echo "Viewer port $viewer_port is already running from this folder." >&2
  exit 1
fi
source ./manipulation_env.sh
export GIT_OPTIONAL_LOCKS=0
export ROS_LOG_DIR="$viewer_root/runtime/ros-logs"
mkdir -p "$ROS_LOG_DIR"
exec /usr/bin/python3 operator_console.py --domain "$viewer_domain" --port "$viewer_port" "$@"
