#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from nav2_common.launch import ReplaceString
from ament_index_python import get_package_share_directory


def _strip_slashes(value: str) -> str:
    return value.strip("/")


def fix_depth_image_tf(context, *args, **kwargs):
    robot_namespace_raw = LaunchConfiguration("robot_namespace").perform(context)
    device_namespace_raw = LaunchConfiguration("device_namespace").perform(context)
    tf_prefix_raw = LaunchConfiguration("tf_prefix").perform(context)

    robot_namespace = _strip_slashes(robot_namespace_raw)
    device_namespace = _strip_slashes(device_namespace_raw)
    tf_prefix = _strip_slashes(tf_prefix_raw)

    camera_prefix = device_namespace or "camera"
    tf_prefix_ext = f"{tf_prefix}_" if tf_prefix else ""

    parent_frame = f"{tf_prefix_ext}{camera_prefix}_depth_optical_frame"
    child_frame = f"{tf_prefix_ext}{camera_prefix}_orbbec_astra_depth"

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="point_cloud_tf",
        output="log",
        arguments=["0", "0", "0", "1.57", "-1.57", "0", parent_frame, child_frame],
        parameters=[{"use_sim_time": True}],
        namespace=robot_namespace,
    )
    return [static_transform_publisher]


def generate_launch_description():
    ros_components_description = get_package_share_directory("ros_components_description")
    gz_bridge_config_path = os.path.join(
        ros_components_description, "config", "gz_orbbec_astra_remappings.yaml"
    )

    robot_namespace = LaunchConfiguration("robot_namespace")
    device_namespace = LaunchConfiguration("device_namespace")
    gz_bridge_name = LaunchConfiguration("gz_bridge_name")

    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=gz_bridge_config_path,
        replacements={
            "<robot_namespace>": robot_namespace,
            "<device_namespace>": device_namespace,
        },
    )

    cleaned_gz_bridge_config_path = ReplaceString(
        source_file=namespaced_gz_bridge_config_path,
        replacements={
            "//": "/",
        },
    )

    declare_device_namespace = DeclareLaunchArgument(
        "device_namespace",
        default_value="",
        description=(
            "Sensor namespace that will appear before all non absolute topics and TF frames, "
            "used for distinguishing multiple cameras on the same robot."
        ),
    )

    declare_tf_prefix = DeclareLaunchArgument(
        "tf_prefix",
        default_value="",
        description="Prefix added for all links of device. Here used as fix for static transform publisher.",
    )

    declare_robot_namespace = DeclareLaunchArgument(
        "robot_namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace which will appear in front of all topics (including /tf and /tf_static).",
    )

    declare_gz_bridge_name = DeclareLaunchArgument(
        "gz_bridge_name",
        default_value="gz_bridge",
        description="Name of gz bridge node.",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=gz_bridge_name,
        parameters=[{"config_file": cleaned_gz_bridge_config_path}],
        namespace=robot_namespace,
        output="screen",
    )

    return LaunchDescription(
        [
            declare_device_namespace,
            declare_robot_namespace,
            declare_tf_prefix,
            declare_gz_bridge_name,
            gz_bridge,
            OpaqueFunction(function=fix_depth_image_tf),
        ]
    )
