#!/usr/bin/env python3

"""Replacement MoveIt RViz launch file that keeps parameters but skips RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    # Preserve the SetParameter side-effect so other nodes still see use_sim_time.
    return LaunchDescription(
        [
            declare_use_sim_arg,
            SetParameter(name="use_sim_time", value=use_sim),
        ]
    )
