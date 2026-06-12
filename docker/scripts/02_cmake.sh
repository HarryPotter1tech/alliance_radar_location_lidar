#!/bin/bash
set -e

echo ""
echo "========================================"
echo "[2/7] 安装 CMake 4.3.3"
echo "========================================"

URLS=(
  "https://cmake.org/files/v4.3/cmake-4.3.3-linux-x86_64.tar.gz"
  "https://github.com/Kitware/CMake/releases/download/v4.3.3/cmake-4.3.3-linux-x86_64.tar.gz"
  "https://mirror.ghproxy.com/https://github.com/Kitware/CMake/releases/download/v4.3.3/cmake-4.3.3-linux-x86_64.tar.gz"
)

download() {
  for url in "${URLS[@]}"; do
    echo "  → 尝试: $url"
    if curl -fsSL --connect-timeout 10 --max-time 300 --retry 3 -o /tmp/cmake.tar.gz "$url"; then
      return 0
    fi
    echo "  ⚠ 失败，尝试下一个..."
  done
  return 1
}

download || { echo "  [FAIL] CMake 所有源下载失败，请检查网络"; exit 1; }

echo "  → 解压到 /opt/cmake-4.3.3..."
mkdir -p /opt/cmake-4.3.3
tar -xzf /tmp/cmake.tar.gz --strip-components=1 -C /opt/cmake-4.3.3

echo "  → 软链接到 /usr/local/bin (PATH 优先于 /usr/bin)..."
ln -sf /opt/cmake-4.3.3/bin/cmake /usr/local/bin/cmake
ln -sf /opt/cmake-4.3.3/bin/ctest /usr/local/bin/ctest
ln -sf /opt/cmake-4.3.3/bin/cpack /usr/local/bin/cpack

rm /tmp/cmake.tar.gz

echo ""
echo "  CMake: $(cmake --version | head -1)"
echo "  [OK] CMake 4.3.3 安装完成"
