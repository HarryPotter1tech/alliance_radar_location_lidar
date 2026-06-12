#!/bin/bash
set -e

echo ""
echo "========================================"
echo "[3/7] 安装 clang-22 (clang-format / clangd / clang-tidy)"
echo "========================================"

echo "  → 配置 apt.llvm.org GPG key..."
curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key | \
  gpg --dearmor -o /usr/share/keyrings/llvm-snapshot.gpg

echo "deb [signed-by=/usr/share/keyrings/llvm-snapshot.gpg] http://apt.llvm.org/jammy/ llvm-toolchain-jammy-22 main" \
  > /etc/apt/sources.list.d/llvm.list

echo "  → apt-get update..."
if apt-get update; then
  echo "  ✓ apt.llvm.org 可用"
else
  echo "  ⚠ apt.llvm.org 不可达，切换镜像: tuna.tsinghua.edu.cn..."
  echo "deb [signed-by=/usr/share/keyrings/llvm-snapshot.gpg] https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/jammy/ llvm-toolchain-jammy-22 main" \
    > /etc/apt/sources.list.d/llvm.list
  apt-get update || { echo "  [FAIL] LLVM apt 源不可达，请检查网络"; exit 1; }
fi

echo "  → apt-get install clang-format-22 clangd-22 clang-tidy-22..."
apt-get install -y --no-install-recommends \
  clang-format-22 clangd-22 clang-tidy-22

rm -rf /var/lib/apt/lists/*

echo ""
echo "  clang-format: $(clang-format-22 --version | head -1)"
echo "  clangd:      $(clangd-22 --version | head -1)"
echo "  clang-tidy:  $(clang-tidy-22 --version | head -1)"
echo "  [OK] clang-22 安装完成"
