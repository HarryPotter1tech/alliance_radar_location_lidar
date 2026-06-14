#!/bin/bash
# banner.sh - Display ALLIANCE RADAR banner on container start
set -e

ACCENT=$'\033[38;2;249;158;26m'
RST=$'\033[0m'

CLANG_VER=$(clang-format --version 2>/dev/null | head -1)
CLANG_VER=${CLANG_VER:-"(not installed)"}

CPUS=$(nproc)
OS=$(cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d= -f2 | tr -d '"')
ROS_DISTRO=$(source /opt/ros/jazzy/setup.bash 2>/dev/null && echo $ROS_DISTRO || echo "jazzy")

mapfile -t LOGO < /home/ubuntu/.script/ow_logo.txt

RIGHT=(
  "${ACCENT}█████╗ ██╗     ██╗     ██╗ █████╗ ███╗   ██╗ ██████╗███████╗      ██████╗  █████╗ ██████╗  █████╗ ██████╗ ${RST}"
  "${ACCENT}██╔══██╗██║     ██║     ██║██╔══██╗████╗  ██║██╔════╝██╔════╝      ██╔══██╗██╔══██╗██╔══██╗██╔══██╗██╔══██╗${RST}"
  "${ACCENT}███████║██║     ██║     ██║███████║██╔██╗ ██║██║     █████╗        ██████╔╝███████║██║  ██║███████║██████╔╝${RST}"
  "${ACCENT}██╔══██║██║     ██║     ██║██╔══██║██║╚██╗██║██║     ██╔══╝        ██╔══██╗██╔══██║██║  ██║██╔══██║██╔══██╗${RST}"
  "${ACCENT}██║  ██║███████╗███████╗██║██║  ██║██║ ╚████║╚██████╗███████╗      ██║  ██║██║  ██║██████╔╝██║  ██║██║  ██║${RST}"
  "${ACCENT}╚═╝  ╚═╝╚══════╝╚══════╝╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝╚══════╝      ╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝${RST}"
  ""
  "${ACCENT}─────────────────────────────────────────────────────${RST}"
  "${ACCENT}  OS        │ ${OS}${RST}"
  "${ACCENT}  Kernel    │ $(uname -r)${RST}"
  "${ACCENT}  ROS 2     │ ${ROS_DISTRO}${RST}"
  "${ACCENT}  Shell     │ zsh${RST}"
  "${ACCENT}─────────────────────────────────────────────────────${RST}"
  "${ACCENT}  CPU       │ $(grep 'model name' /proc/cpuinfo 2>/dev/null | head -1 | cut -d: -f2 | xargs)${RST}"
  "${ACCENT}  Cores     │ ${CPUS} cores${RST}"
  "${ACCENT}  Memory    │ $(free -h 2>/dev/null | awk '/Mem:/{print $2}' || echo "N/A")${RST}"
  "${ACCENT}─────────────────────────────────────────────────────${RST}"
  "${ACCENT}  GCC       │ $(gcc --version | head -1)${RST}"
  "${ACCENT}  CMake     │ $(cmake --version | head -1)${RST}"
  "${ACCENT}  clang     │ ${CLANG_VER}${RST}"
  "${ACCENT}─────────────────────────────────────────────────────${RST}"
  "${ACCENT}  Workspace │ /workspace${RST}"
  "${ACCENT}  Image     │ radar:develop${RST}"
  "${ACCENT}─────────────────────────────────────────────────────${RST}"
)

printf "\n\n"
MAX=${#LOGO[@]}
[ ${#RIGHT[@]} -gt $MAX ] && MAX=${#RIGHT[@]}

for ((i=0; i<MAX; i++)); do
  l="${LOGO[$i]:-}"
  r="${RIGHT[$i]:-}"
  printf "${ACCENT}%-42s    %b${RST}\n" "$l" "$r"
done
printf "\n"
