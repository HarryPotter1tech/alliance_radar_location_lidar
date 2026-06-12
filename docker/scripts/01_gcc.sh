#!/bin/bash
set -e

echo ""
echo "========================================"
echo "[1/7] 安装 GCC 13 (C++23 编译器)"
echo "========================================"

echo "  → 安装 add-apt-repository..."
apt-get install -y --no-install-recommends software-properties-common

echo "  → 添加 ubuntu-toolchain-r/test PPA..."
add-apt-repository -y ppa:ubuntu-toolchain-r/test

echo "  → apt-get update..."
apt-get update

echo "  → apt-get install gcc-13 g++-13..."
apt-get install -y --no-install-recommends gcc-13 g++-13

echo "  → update-alternatives 设置默认编译器..."
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100

rm -rf /var/lib/apt/lists/*

echo ""
echo "  GCC: $(gcc --version | head -1)"
echo "  G++: $(g++ --version | head -1)"
echo "  [OK] GCC 13 安装完成"
