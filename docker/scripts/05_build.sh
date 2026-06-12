#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# ============================================================
# 创建 workspace 目录结构
# ============================================================
mkdir -p /workspace/ros_ws/build /workspace/ros_ws/install /workspace/ros_ws/log
touch /workspace/ros_ws/build/COLCON_IGNORE /workspace/ros_ws/install/COLCON_IGNORE /workspace/ros_ws/log/COLCON_IGNORE
mkdir -p /workspace/lidar_ros_driver/build /workspace/lidar_ros_driver/install /workspace/lidar_ros_driver/log
touch /workspace/lidar_ros_driver/build/COLCON_IGNORE /workspace/lidar_ros_driver/install/COLCON_IGNORE /workspace/lidar_ros_driver/log/COLCON_IGNORE

# ============================================================
echo ""
echo "========================================"
echo "[5/7] 编译 third-party (small_gicp + hikcamera)"
echo "========================================"
echo "  → colcon build small_gicp + hikcamera (Release)..."
cd /workspace/ros_ws
colcon build \
  --packages-select small_gicp hikcamera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev \
  || { echo "  [FAIL] third-party 编译失败"; exit 1; }
echo "  [OK] small_gicp + hikcamera 编译完成"

# ============================================================
echo ""
echo "========================================"
echo "[6/7] 编译 radar_localization_lidar (主包)"
echo "========================================"
sed -i "s/-Wextra/-Wextra -Wno-missing-field-initializers -Wno-unused-parameter/" \
  /workspace/ros_ws/src/radar_localization_lidar/CMakeLists.txt
echo "  → colcon build radar_localization_lidar (Release)..."
colcon build \
  --packages-select radar_localization_lidar \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev \
  || { echo "  [FAIL] radar_localization_lidar 编译失败"; exit 1; }
echo "  [OK] radar_localization_lidar 编译完成"

# ============================================================
echo ""
echo "========================================"
echo "[7/7] 编译 livox_ros_driver2 (Livox ROS2 驱动)"
echo "========================================"
cd /workspace/lidar_ros_driver
cp -f livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml
cp -rf livox_ros_driver2/launch_ROS2/ livox_ros_driver2/launch/
echo "  → colcon build livox_ros_driver2 (Release, ROS2)..."
colcon build \
  --packages-select livox_ros_driver2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -Wno-dev \
  || { echo "  [FAIL] livox_ros_driver2 编译失败"; exit 1; }
echo "  [OK] livox_ros_driver2 编译完成"
