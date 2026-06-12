FROM ghcr.io/harrypotter1tech/harryh_radar:latest

# =============================================================================
# RADAR-LOCATION-LIDAR 编译环境镜像
#
# 用法:
#   1. git submodule update --init --recursive
#   2. docker build -t radar:full .
#   3. docker run -it -d --name RADAR --privileged --restart=always \
#        -v /dev:/dev \
#        -v $(pwd):/workspace \
#        --network host \
#        radar:full
#
# docker build 成功 = 所有 5 个包编译验证通过
# =============================================================================

COPY docker/scripts/ /tmp/docker/

# ===== 工具链 =====
RUN /tmp/docker/00_mirror.sh
RUN /tmp/docker/01_gcc.sh
RUN /tmp/docker/02_cmake.sh
RUN /tmp/docker/03_clang.sh

# ===== Livox SDK (子模块) =====
COPY lidar_ros_driver/livox_sdk2 /tmp/livox_sdk2
RUN /tmp/docker/04_livox_sdk.sh

# ===== 子模块源码 → workspace =====
COPY ros_ws/third-party/small_gicp       /workspace/ros_ws/third-party/small_gicp
COPY ros_ws/third-party/ros2-hikcamera   /workspace/ros_ws/third-party/ros2-hikcamera
COPY ros_ws/src/radar_localization_lidar /workspace/ros_ws/src/radar_localization_lidar
COPY lidar_ros_driver/livox_ros_driver2  /workspace/lidar_ros_driver/livox_ros_driver2

# ===== 编译 + Banner =====
RUN /tmp/docker/05_build.sh
RUN /tmp/docker/06_banner.sh

WORKDIR /workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
