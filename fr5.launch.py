"""Compatibility launch file; the shared launch defaults to the FR5 profile."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    source = Path(__file__).resolve().with_name('manipulation.launch.py')
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)))])
