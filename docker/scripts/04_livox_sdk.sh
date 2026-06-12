#!/bin/bash
set -e

echo ""
echo "========================================"
echo "[4/7] 编译 Livox SDK2 (子模块 → /usr/local)"
echo "========================================"

echo "  → cmake 配置 (Release)..."
mkdir -p /tmp/livox_sdk2/build && cd /tmp/livox_sdk2/build

sed -i '6i set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-pragmas -Wno-c++20-compat -include cstdint")' ../CMakeLists.txt

cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  || { echo "  [FAIL] Livox SDK2 cmake 配置失败"; exit 1; }

echo "  → make 编译中 (-j$(nproc))..."
make -j$(nproc) \
  || { echo "  [FAIL] Livox SDK2 编译失败"; exit 1; }

echo "  → make install 安装到 /usr/local..."
make install \
  || { echo "  [FAIL] Livox SDK2 安装失败"; exit 1; }

echo "  → 清理源码..."
rm -rf /tmp/livox_sdk2

echo ""
echo "  liblivox_lidar_sdk_shared.so: /usr/local/lib"
echo "  livox_lidar_api.h:           /usr/local/include"
echo "  [OK] Livox SDK2 编译安装完成"
