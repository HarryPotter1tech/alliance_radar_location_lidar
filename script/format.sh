#!/bin/bash
cd /workspace/ros_ws/src/radar_localization_lidar/src
clang-format -i *.cpp *.hpp
SRC_DIR="/workspace/ros_ws/src/radar_localization_lidar/src"
echo "Formatted all cpp files and hpp files with clang-format in ${SRC_DIR}"
