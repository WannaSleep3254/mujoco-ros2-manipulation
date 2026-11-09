#!/usr/bin/env bash
set -e
viewer_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$viewer_root"
viewer_domain="${VIEWER_ROS_DOMAIN:-218}"
# Match the operator's final port, including its unique option abbreviations.
viewer_port="$(/usr/bin/python3 - "$@" <<'PY'
import argparse
import os

parser = argparse.ArgumentParser(add_help=False, allow_abbrev=True)
parser.add_argument("--port", type=int, default=os.environ.get("VIEWER_PORT", "8933"))
arguments, _ = parser.parse_known_args()
if not 1024 <= arguments.port <= 65535:
    parser.error("port must be 1024..65535")
print(arguments.port)
PY
)"
mkdir -p runtime
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
