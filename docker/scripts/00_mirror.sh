#!/bin/bash
set -e

echo ""
echo "========================================"
echo "[0/7] apt 源切换为 tuna 镜像"
echo "========================================"

echo "  → 替换 ubuntu 源..."
sed -i \
  -e 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' \
  -e 's|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' \
  /etc/apt/sources.list

rm -f /etc/apt/sources.list.d/*ros* /etc/apt/sources.list.d/*ros*.sources

echo "  → apt-get update..."
apt-get update

echo "  [OK] apt 源已切换为 tuna 镜像"
