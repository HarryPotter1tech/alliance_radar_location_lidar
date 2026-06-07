#!/bin/bash
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
rm -rf build/radar_localization_lidar
colcon build --packages-select radar_localization_lidar
