#!/bin/bash
# post-create.sh - First-time container setup
set -euo pipefail

echo "========================================="
echo "  RADAR-LOCATION-LIDAR post-create setup"
echo "========================================="

# Check host opencode config
if [ -f /home/ubuntu/.opencode/bin/opencode ]; then
    echo "[OK] opencode binary found from host mount"
fi

# Check host codex config
if [ -f /home/ubuntu/.codex/config.toml ]; then
    echo "[OK] codex config found from host mount"
fi

# Initialize git submodules if needed
cd /workspace
if [ -f .gitmodules ]; then
    NEED_INIT=false
    for submod in ros_ws/third-party/small_gicp ros_ws/third-party/ros2-hikcamera; do
        if [ ! -f "${submod}/.git" ] && [ ! -d "${submod}/.git" ]; then
            NEED_INIT=true
            break
        fi
    done
    if [ "$NEED_INIT" = true ]; then
        echo "[..] Initializing git submodules..."
        git submodule update --init --recursive
    fi
fi

# Build third-party packages if not already built
NEED_BUILD=false
for pkg in small_gicp hikcamera; do
    if [ ! -d "/workspace/ros_ws/install/${pkg}" ]; then
        NEED_BUILD=true
        break
    fi
done

if [ "$NEED_BUILD" = true ]; then
    echo "[..] First-time build: third-party packages..."
    cd /workspace/ros_ws
    source /opt/ros/jazzy/setup.bash
    colcon build --packages-select small_gicp hikcamera \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev
fi

echo ""
echo "[OK] post-create complete. Ready to develop."
