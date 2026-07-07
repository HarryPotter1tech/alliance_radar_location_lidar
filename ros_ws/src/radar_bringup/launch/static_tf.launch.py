#!/usr/bin/env python3

from __future__ import annotations

import os
from typing import Final

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir: Final[str] = get_package_share_directory("radar_bringup")
    extrinsics_path: Final[str] = os.path.join(
        bringup_dir, "config", "common", "extrinsics.yaml"
    )

    with open(extrinsics_path, encoding="utf-8") as extrinsics_file:
        config = yaml.safe_load(extrinsics_file)

    transforms = config["static_transforms"]
    nodes: list[Node] = []
    for name, transform in transforms.items():
        translation = transform["translation"]
        rotation = transform["rotation"]
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_tf_{name}",
                output="screen",
                arguments=[
                    "--x",
                    str(translation["x"]),
                    "--y",
                    str(translation["y"]),
                    "--z",
                    str(translation["z"]),
                    "--qx",
                    str(rotation["qx"]),
                    "--qy",
                    str(rotation["qy"]),
                    "--qz",
                    str(rotation["qz"]),
                    "--qw",
                    str(rotation["qw"]),
                    "--frame-id",
                    transform["parent"],
                    "--child-frame-id",
                    transform["child"],
                ],
            )
        )

    return LaunchDescription(nodes)
