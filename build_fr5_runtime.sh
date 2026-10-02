#!/usr/bin/env bash
# Compatibility entry point; the native runtime is shared by all robot profiles.
set -euo pipefail
exec bash "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/build_runtime.sh" "$@"
