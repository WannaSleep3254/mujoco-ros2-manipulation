#!/usr/bin/env bash
# Compatibility command; source pins and build targets live in config/build.yaml.
set -euo pipefail
task_build_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$task_build_root/build_workspace.py" --component runtime "$@"
