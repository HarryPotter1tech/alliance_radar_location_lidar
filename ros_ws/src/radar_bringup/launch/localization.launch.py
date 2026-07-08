#!/usr/bin/env python3
"""radar_lidar 通用定位启动文件。

按 sensor 参数选择对应的参数 YAML, 并按需启动传感器驱动。

用法:
    ros2 launch radar_bringup localization.launch.py sensor:=odin map_path:=/path/to/map.pcd
    ros2 launch radar_bringup localization.launch.py sensor:=mid70 map_path:=/path/to/map.pcd

参数:
    sensor   传感器型号: odin | mid70  (默认 odin)
    map_path 地图 PCD 绝对路径          (默认 /workspace/model/generated/map.pcd)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


VALID_SENSORS = {"odin", "mid70"}


def _make_radar_node(context: LaunchContext):
    sensor_val = LaunchConfiguration("sensor").perform(context)
    if sensor_val not in VALID_SENSORS:
        message = f"Unsupported sensor '{sensor_val}'. Expected one of: {sorted(VALID_SENSORS)}"
        get_logger("localization_launch").error(message)
        raise RuntimeError(message)

    radar_dir = get_package_share_directory("radar_lidar")
    radar_params = LaunchConfiguration("radar_params").perform(context)
    map_path = LaunchConfiguration("map_path").perform(context)
    scan_topic = "/odin1/cloud_raw" if sensor_val == "odin" else "/livox/lidar"
    hardware_id = "odin1" if sensor_val == "odin" else "livox_mid70"

    return [
        Node(
            package="radar_lidar",
            executable="radar_lidar_node",
            name="radar_lidar_node",
            output="screen",
            parameters=[
                radar_params,
                {"map_path": map_path},
                {"scan_topic": scan_topic},
                {"hardware_id": hardware_id},
            ],
        )
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")

    sensor_arg = DeclareLaunchArgument(
        "sensor",
        default_value="odin",
        description="传感器型号: odin | mid70",
    )
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value="/workspace/model/generated/map.pcd",
        description="地图 PCD 绝对路径",
    )
    radar_params_arg = DeclareLaunchArgument(
        "radar_params",
        default_value=os.path.join(
            get_package_share_directory("radar_lidar"), "config", "runtime.yaml"
        ),
        description="radar_lidar runtime parameter YAML",
    )

    odin_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(
                    bringup_dir, "config", "lidar", "odin_driver.yaml"
                ),
            }
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("sensor"), "' == 'odin'"]
            )
        ),
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "static_tf.launch.py")
        )
    )

    radar_node = OpaqueFunction(function=_make_radar_node)

    return LaunchDescription([
        sensor_arg,
        map_path_arg,
        radar_params_arg,
        static_tf_launch,
        odin_node,
        radar_node,
    ])
