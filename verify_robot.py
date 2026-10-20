"""Compatibility entry point; implementation lives in the src package."""

import _bootstrap
import importlib
import sys

_module = importlib.import_module("mujoco_ros2_manipulation.verification.ros_checks")
if __name__ == "__main__":
    raise SystemExit(_module.main())
sys.modules[__name__] = _module
