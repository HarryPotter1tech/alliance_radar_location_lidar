#!/usr/bin/env python3
"""Odin 重定位 + radar_lidar 联合启动文件。

Odin1 以 custom_map_mode=2 运行内置重定位，radar_lidar 订阅其 map->odin1_base_link
TF 作为主位姿源；重定位未成功前，radar_lidar 自动回退到自身的 GICP scan-to-map。

用法:
    ros2 launch radar_bringup odin_relocalization_localization.launch.py \
        map_path:=/path/to/map.pcd \
        relocalization_map_path:=/path/to/map.bin \
        init_pos:="[x,y,z,qx,qy,qz,qw]"
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
    radar_dir = get_package_share_directory("radar_lidar")

    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="/workspace/model/generated/map_zup.pcd",
        description="GICP 回退用的地图 PCD 绝对路径",
    )
    relocalization_map_path_arg = DeclareLaunchArgument(
        "relocalization_map_path",
        default_value="",
        description="Odin1 SLAM 模式保存的 .bin 地图绝对路径（必填）",
    )
    radar_params_arg = DeclareLaunchArgument(
        "radar_params",
        default_value=os.path.join(radar_dir, "config", "runtime.yaml"),
        description="radar_lidar 运行时参数 YAML",
    )
    odin_config_arg = DeclareLaunchArgument(
        "odin_config",
        default_value=os.path.join(
            bringup_dir, "config", "lidar", "odin_driver_relocalization.yaml"
        ),
        description="Odin 驱动 control_command.yaml（重定位模式）",
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "static_tf.launch.py")
        )
    )

    odin_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[
            {"config_file": LaunchConfiguration("odin_config")},
            {"relocalization_map_abs_path": LaunchConfiguration("relocalization_map_path")},
        ],
    )

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
            {"use_odin_relocalization_tf": True},
        ],
    )

    return LaunchDescription([
        map_path_arg,
        relocalization_map_path_arg,
        radar_params_arg,
        odin_config_arg,
        static_tf_launch,
        odin_node,
        radar_node,
    ])
