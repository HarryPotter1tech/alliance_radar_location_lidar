FROM ghcr.io/harrypotter1tech/harryh_radar:latest

# =============================================================================
# [0/7] 国内网络优化：替换 apt 源为 tuna 镜像
# =============================================================================
RUN printf "\n\n\n\n" && \
    echo "========================================" && \
    echo "[0/7] apt 源切换为 tuna 镜像" && \
    echo "========================================" && \
    echo "  → 替换 ubuntu 源..." && \
    sed -i \
      -e 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' \
      -e 's|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' \
      /etc/apt/sources.list && \
    rm -f /etc/apt/sources.list.d/*ros* /etc/apt/sources.list.d/*ros*.sources && \
    echo "  → apt-get update..." && \
    apt-get update && \
    echo "  [OK] apt 源已切换为 tuna 镜像"

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

# =============================================================================
# [1/7] 安装 GCC 13 (C++23 编译器)
# =============================================================================
RUN printf "\n\n\n\n" && \
    echo "========================================" && \
    echo "[1/7] 安装 GCC 13 (C++23 编译器)" && \
    echo "========================================" && \
    echo "  → 安装 add-apt-repository..." && \
    apt-get install -y --no-install-recommends software-properties-common && \
    echo "  → 添加 ubuntu-toolchain-r/test PPA..." && \
    add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    echo "  → apt-get update..." && \
    apt-get update && \
    echo "  → apt-get install gcc-13 g++-13..." && \
    apt-get install -y --no-install-recommends gcc-13 g++-13 && \
    echo "  → update-alternatives 设置默认编译器..." && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100 && \
    rm -rf /var/lib/apt/lists/* && \
    echo "" && \
    echo "  GCC: $(gcc --version | head -1)" && \
    echo "  G++: $(g++ --version | head -1)" && \
    echo "  [OK] GCC 13 安装完成"

# =============================================================================
# [2/7] 安装 CMake 4.3.3
# =============================================================================
RUN printf "\n\n\n\n" && \
    echo "========================================" && \
    echo "[2/7] 安装 CMake 4.3.3" && \
    echo "========================================" && \
    echo "  → 下载中 (主源: GitHub Releases)..." && \
    { curl -fsSL --connect-timeout 10 --max-time 300 --retry 3 \
        -o /tmp/cmake.tar.gz \
        "https://github.com/Kitware/CMake/releases/download/v4.3.3/cmake-4.3.3-linux-x86_64.tar.gz"; \
    } || { \
      echo "  ⚠ 主源超时，切换镜像: ghproxy.com..."; \
      curl -fsSL --connect-timeout 10 --max-time 300 --retry 3 \
        -o /tmp/cmake.tar.gz \
        "https://mirror.ghproxy.com/https://github.com/Kitware/CMake/releases/download/v4.3.3/cmake-4.3.3-linux-x86_64.tar.gz" \
        || { echo "  [FAIL] CMake 下载失败，请检查网络"; exit 1; }; \
    } && \
    echo "  → 解压到 /opt/cmake-4.3.3..." && \
    mkdir -p /opt/cmake-4.3.3 && \
    tar -xzf /tmp/cmake.tar.gz --strip-components=1 -C /opt/cmake-4.3.3 && \
    echo "  → 软链接到 /usr/local/bin (PATH 优先于 /usr/bin)..." && \
    ln -sf /opt/cmake-4.3.3/bin/cmake /usr/local/bin/cmake && \
    ln -sf /opt/cmake-4.3.3/bin/ctest /usr/local/bin/ctest && \
    ln -sf /opt/cmake-4.3.3/bin/cpack /usr/local/bin/cpack && \
    rm /tmp/cmake.tar.gz && \
    echo "" && \
    echo "  CMake: $(cmake --version | head -1)" && \
    echo "  [OK] CMake 4.3.3 安装完成"

# =============================================================================
# [3/7] 安装 clang-22 (格式化 / LSP)
# =============================================================================
RUN printf "\n\n\n\n" && \
    echo "========================================" && \
    echo "[3/7] 安装 clang-22 (clang-format / clangd / clang-tidy)" && \
    echo "========================================" && \
    echo "  → 配置 apt.llvm.org 源..." && \
    curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key | \
      gpg --dearmor -o /usr/share/keyrings/llvm-snapshot.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/llvm-snapshot.gpg] http://apt.llvm.org/jammy/ llvm-toolchain-jammy-22 main" \
      > /etc/apt/sources.list.d/llvm.list && \
    echo "  → apt-get update..." && \
    { apt-get update; } || { \
      echo "  ⚠ apt.llvm.org 不可达，切换镜像: tuna.tsinghua.edu.cn..."; \
      echo "deb [signed-by=/usr/share/keyrings/llvm-snapshot.gpg] https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/jammy/ llvm-toolchain-jammy-22 main" \
        > /etc/apt/sources.list.d/llvm.list && \
      apt-get update \
        || { echo "  [FAIL] LLVM apt 源不可达，请检查网络"; exit 1; }; \
    } && \
    echo "  → apt-get install clang-22 clangd-22 clang-tidy-22..." && \
    apt-get install -y --no-install-recommends \
      clang-format-22 clangd-22 clang-tidy-22 && \
    rm -rf /var/lib/apt/lists/* && \
    echo "" && \
    echo "  clang-format: $(clang-format-22 --version | head -1)" && \
    echo "  clangd:      $(clangd-22 --version | head -1)" && \
    echo "  clang-tidy:  $(clang-tidy-22 --version | head -1)" && \
    echo "  [OK] clang-22 安装完成"

# =============================================================================
# [4/7] 编译 Livox SDK2 (子模块)
# =============================================================================
COPY lidar_ros_driver/livox_sdk2 /tmp/livox_sdk2

RUN printf "\n\n\n\n" && \
    echo "========================================" && \
    echo "[4/7] 编译 Livox SDK2 (子模块 → /usr/local)" && \
    echo "========================================" && \
    echo "  → cmake 配置 (Release)..." && \
    mkdir -p /tmp/livox_sdk2/build && cd /tmp/livox_sdk2/build && \
    sed -i '6i set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-c++20-compat -include cstdint")' ../CMakeLists.txt && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      || { echo "  [FAIL] Livox SDK2 cmake 配置失败"; exit 1; } && \
    echo "  → make 编译中 (-j$(nproc))..." && \
    make -j$(nproc) \
      || { echo "  [FAIL] Livox SDK2 编译失败"; exit 1; } && \
    echo "  → make install 安装到 /usr/local..." && \
    make install \
      || { echo "  [FAIL] Livox SDK2 安装失败"; exit 1; } && \
    echo "  → 清理源码..." && \
    rm -rf /tmp/livox_sdk2 && \
    echo "" && \
    echo "  liblivox_lidar_sdk_shared.so: /usr/local/lib" && \
    echo "  livox_lidar_api.h:           /usr/local/include" && \
    echo "  [OK] Livox SDK2 编译安装完成"

# =============================================================================
# 拷贝子模块源码到 workspace
# =============================================================================
COPY ros_ws/third-party/small_gicp       /workspace/ros_ws/third-party/small_gicp
COPY ros_ws/third-party/ros2-hikcamera   /workspace/ros_ws/third-party/ros2-hikcamera
COPY ros_ws/src/radar_localization_lidar /workspace/ros_ws/src/radar_localization_lidar
COPY lidar_ros_driver/livox_ros_driver2  /workspace/lidar_ros_driver/livox_ros_driver2

# 创建 workspace 目录结构和 COLCON_IGNORE (避免 colcon 扫描 build/install/log)
RUN mkdir -p /workspace/ros_ws/build \
             /workspace/ros_ws/install \
             /workspace/ros_ws/log && \
    touch /workspace/ros_ws/build/COLCON_IGNORE \
          /workspace/ros_ws/install/COLCON_IGNORE \
          /workspace/ros_ws/log/COLCON_IGNORE && \
    mkdir -p /workspace/lidar_ros_driver/build \
             /workspace/lidar_ros_driver/install \
             /workspace/lidar_ros_driver/log && \
    touch /workspace/lidar_ros_driver/build/COLCON_IGNORE \
          /workspace/lidar_ros_driver/install/COLCON_IGNORE \
          /workspace/lidar_ros_driver/log/COLCON_IGNORE

# =============================================================================
# [5/7] 编译 small_gicp + hikcamera (第三方依赖)
# =============================================================================
RUN printf "\n\n\n\n" && \
    echo "========================================" && \
    echo "[5/7] 编译 third-party (small_gicp + hikcamera)" && \
    echo "========================================" && \
    bash -c 'source /opt/ros/humble/setup.bash && \
    cd /workspace/ros_ws && \
    echo "  → colcon build small_gicp + hikcamera (Release)..." && \
    colcon build \
      --packages-select small_gicp hikcamera \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev \
      || { echo "  [FAIL] third-party 编译失败"; exit 1; }' && \
    echo "" && \
    echo "  [OK] small_gicp + hikcamera 编译完成"

# =============================================================================
# [6/7] 编译 radar_localization_lidar (主包)
# =============================================================================
RUN printf "\n\n\n\n" && \
    echo "========================================" && \
    echo "[6/7] 编译 radar_localization_lidar (主包)" && \
    echo "========================================" && \
    bash -c 'source /opt/ros/humble/setup.bash && \
    cd /workspace/ros_ws && \
    echo "  → colcon build radar_localization_lidar (Release)..." && \
    sed -i "s/-Wextra/-Wextra -Wno-missing-field-initializers -Wno-unused-parameter/" \
      /workspace/ros_ws/src/radar_localization_lidar/CMakeLists.txt && \
    colcon build \
      --packages-select radar_localization_lidar \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev \
      || { echo "  [FAIL] radar_localization_lidar 编译失败"; exit 1; }' && \
    echo "" && \
    echo "  [OK] radar_localization_lidar 编译完成"

# =============================================================================
# [7/7] 编译 livox_ros_driver2 (Livox ROS2 驱动)
# =============================================================================
RUN printf "\n\n\n\n" && \
    echo "========================================" && \
    echo "[7/7] 编译 livox_ros_driver2 (Livox ROS2 驱动)" && \
    echo "========================================" && \
    bash -c 'source /opt/ros/humble/setup.bash && \
    cd /workspace/lidar_ros_driver && \
    cp -f livox_ros_driver2/package_ROS2.xml livox_ros_driver2/package.xml && \
    cp -rf livox_ros_driver2/launch_ROS2/ livox_ros_driver2/launch/ && \
    echo "  → colcon build livox_ros_driver2 (Release, ROS2)..." && \
    colcon build \
      --packages-select livox_ros_driver2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -Wno-dev \
      || { echo "  [FAIL] livox_ros_driver2 编译失败"; exit 1; }' && \
    echo "" && \
    echo "  [OK] livox_ros_driver2 编译完成"

# =============================================================================
# 环境配置
# =============================================================================
RUN printf "\n\n\n\n" && \
    echo "    █████╗ ██╗     ██╗     ██╗ █████╗ ███╗   ██╗ ██████╗███████╗" && \
    echo "   ██╔══██╗██║     ██║     ██║██╔══██╗████╗  ██║██╔════╝██╔════╝" && \
    echo "   ███████║██║     ██║     ██║███████║██╔██╗ ██║██║     █████╗  " && \
    echo "   ██╔══██║██║     ██║     ██║██╔══██║██║╚██╗██║██║     ██╔══╝  " && \
    echo "   ██║  ██║███████╗███████╗██║██║  ██║██║ ╚████║╚██████╗███████╗" && \
    echo "   ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝╚══════╝" && \
    echo "" && \
    echo "                  ██████╗  █████╗ ██████╗  █████╗ ██████╗ " && \
    echo "                  ██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔══██╗" && \
    echo "                  ██████╔╝███████║██║  ██║███████║██████╔╝" && \
    echo "                  ██╔══██╗██╔══██║██║  ██║██╔══██║██╔══██╗" && \
    echo "                  ██║  ██║██║  ██║██████╔╝██║  ██║██║  ██║" && \
    echo "                  ╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝" && \
    echo "" && \
    echo "  ██╗      ██████╗  ██████╗ █████╗ ████████╗██╗ ██████╗ ███╗   ██╗" && \
    echo "  ██║     ██╔═══██╗██╔════╝██╔══██╗╚══██╔══╝██║██╔═══██╗████╗  ██║" && \
    echo "  ██║     ██║   ██║██║     ███████║   ██║   ██║██║   ██║██╔██╗ ██║" && \
    echo "  ██║     ██║   ██║██║     ██╔══██║   ██║   ██║██║   ██║██║╚██╗██║" && \
    echo "  ███████╗╚██████╔╝╚██████╗██║  ██║   ██║   ██║╚██████╔╝██║ ╚████║" && \
    echo "  ╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝" && \
    echo "                    ██╗     ██╗██████╗  █████╗ ██████╗ " && \
    echo "                    ██║     ██║██╔══██╗██╔══██╗██╔══██╗" && \
    echo "                    ██║     ██║██║  ██║███████║██████╔╝" && \
    echo "                    ██║     ██║██║  ██║██╔══██║██╔══██╗" && \
    echo "                    ███████╗██║██████╔╝██║  ██║██║  ██║" && \
    echo "                    ╚══════╝╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝" && \
    echo "" && \
    echo "========================================" && \
    echo "  构建完成!  7/7 全部通过" && \
    echo "========================================" && \
    echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> /etc/skel/.bashrc && \
    echo "" && \
    echo "  [OK] 环境配置完成"

# =============================================================================
# 最终信息
# =============================================================================
RUN printf "\n\n\n\n\n" && \
    echo "========================================" && \
    echo "  构建完成!  7/7 全部通过" && \
    echo "========================================" && \
    echo "" && \
    echo "  GCC:   $(gcc --version | head -1)" && \
    echo "  CMake: $(cmake --version | head -1)" && \
    echo "  clang: $(clang-format-22 --version | head -1)" && \
    echo "" && \
    echo "  全都编译成功: Livox SDK2 | small_gicp | hikcamera | radar | livox_driver" && \
    echo "" && \
    echo "  docker run --privileged -v /dev:/dev -v \$(pwd):/workspace --network host radar:full" && \
    echo ""

WORKDIR /workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
