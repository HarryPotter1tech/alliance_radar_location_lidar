#!/bin/bash
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select radar_localization_lidar
cd ..
source /opt/ros/humble/setup.bash
source /workspace/ros_ws/install/setup.bash
ros2 run radar_localization_lidar radar_localization_lidar_node

