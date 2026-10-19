"""Resolve an explicit workspace independently of the package install location."""
import os
from pathlib import Path


def workspace_root():
    override = os.environ.get('MUJOCO_ROS2_WORKSPACE')
    candidates = [Path(override)] if override else [Path.cwd(), *Path.cwd().parents, *Path(__file__).resolve().parents]
    for candidate in candidates:
        root = candidate.expanduser().resolve()
        if (root / 'config/robots').is_dir() and (root / 'config/build.yaml').is_file():
            return root
    raise RuntimeError('Workspace not found; set MUJOCO_ROS2_WORKSPACE to the repository checkout')


ROOT = workspace_root()
