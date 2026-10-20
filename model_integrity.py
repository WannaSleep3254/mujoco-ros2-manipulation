"""Compatibility entry point; implementation lives in the src package."""

import _bootstrap
import importlib
import sys

_module = importlib.import_module("mujoco_ros2_manipulation.model_build.integrity")
sys.modules[__name__] = _module
