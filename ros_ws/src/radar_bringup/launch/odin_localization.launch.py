#!/usr/bin/env python3
"""Odin + radar_lidar 联合启动文件。
只起两个节点：odin_ros_driver（原始点云）和 radar_lidar_node（定位）。
用法:
    ros2 launch radar_bringup odin_localization.launch.py \
        map_path:=/path/to/map.pcd
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")
    radar_dir   = get_package_share_directory("radar_lidar")

    # ── arguments ────────────────────────────────────────────────
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="/workspace/model/generated/map.pcd",
        description="Absolute path to the map PCD file",
    )
    radar_params_arg = DeclareLaunchArgument(
        "radar_params",
        default_value=os.path.join(radar_dir, "config", "runtime.yaml"),
        description="radar_lidar runtime parameter YAML",
    )
    odin_config_arg = DeclareLaunchArgument(
        "odin_config",
        default_value=os.path.join(bringup_dir, "config", "lidar", "odin_driver.yaml"),
        description="Odin driver control_command.yaml",
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "static_tf.launch.py")
        )
    )

    # ── Odin driver ─────────────────────────────────────────────
    odin_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("odin_config"),
        }],
    )

    # ── radar_lidar ─────────────────────────────────────────────
    radar_node = Node(
        package="radar_lidar",
        executable="radar_lidar_node",
        name="radar_lidar_node",
        output="screen",
        parameters=[
            LaunchConfiguration("radar_params"),
            {"map_path": LaunchConfiguration("map_path")},
            {"scan_topic": "/odin1/cloud_raw"},
            {"hardware_id": "odin1"},
        ],
    )

    return LaunchDescription([
        map_path_arg,
        radar_params_arg,
        odin_config_arg,
        static_tf_launch,
        odin_node,
        radar_node,
    ])
