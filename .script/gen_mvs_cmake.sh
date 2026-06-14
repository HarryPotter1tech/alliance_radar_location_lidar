#!/bin/bash
# gen_mvs_cmake.sh - Generate MVSUSB3Core CMake config
set -eu

: "${TARGETARCH:=amd64}"

case "${TARGETARCH}" in
    amd64) arch_tag=x86_64; arch_dir=64 ;;
    arm64) arch_tag=aarch64; arch_dir=aarch64 ;;
    *) echo "unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;;
esac

# Install MVS SDK
download_url="https://github.com/Alliance-Algorithm/hik-mvs/releases/latest/download/mvs-sdk-${arch_tag}.tar.gz"
mkdir -p /tmp/mvs-src \
    "/opt/mvs-usb3-core/lib/${arch_dir}" \
    /opt/mvs-usb3-core/lib/cmake/MVSUSB3Core \
    /opt/mvs-usb3-core/include
curl -fsSL -o /tmp/mvs.tar.gz "${download_url}"
tar -xzf /tmp/mvs.tar.gz -C /tmp/mvs-src
cp -a "/tmp/mvs-src/lib/${arch_dir}/." "/opt/mvs-usb3-core/lib/${arch_dir}/"
cp -a /tmp/mvs-src/include/. /opt/mvs-usb3-core/include/
rm -f "/opt/mvs-usb3-core/lib/${arch_dir}/libusb-1.0.so.0"
rm -rf /var/lib/apt/lists/* /tmp/mvs-src /tmp/mvs.tar.gz

# Generate CMake config
cmake_dir=/opt/mvs-usb3-core/lib/cmake/MVSUSB3Core
cat > "${cmake_dir}/MVSUSB3CoreConfig.cmake" << 'EOF'
get_filename_component(MVSUSB3Core_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(MVSUSB3Core_INCLUDE_DIR "${MVSUSB3Core_ROOT}/include")
set(MVSUSB3Core_INCLUDE_DIRS "${MVSUSB3Core_INCLUDE_DIR}")
EOF

echo "set(MVSUSB3Core_LIBRARY_DIR \"\${MVSUSB3Core_ROOT}/lib/${arch_dir}\")" >> "${cmake_dir}/MVSUSB3CoreConfig.cmake"

cat >> "${cmake_dir}/MVSUSB3CoreConfig.cmake" << 'EOF'
set(MVSUSB3Core_LIBRARY "${MVSUSB3Core_LIBRARY_DIR}/libMvCameraControl.so")
set(MVSUSB3Core_LIBRARIES "${MVSUSB3Core_LIBRARY}")
if(NOT EXISTS "${MVSUSB3Core_INCLUDE_DIR}/MvCameraControl.h")
  set(MVSUSB3Core_FOUND FALSE)
  message(FATAL_ERROR "MVSUSB3Core headers not found under ${MVSUSB3Core_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${MVSUSB3Core_LIBRARY}")
  set(MVSUSB3Core_FOUND FALSE)
  message(FATAL_ERROR "MVSUSB3Core library not found under ${MVSUSB3Core_LIBRARY_DIR}")
endif()
if(NOT TARGET MVSUSB3Core::MVSUSB3Core)
  add_library(MVSUSB3Core::MVSUSB3Core SHARED IMPORTED GLOBAL)
  set_target_properties(MVSUSB3Core::MVSUSB3Core PROPERTIES
    IMPORTED_LOCATION "${MVSUSB3Core_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MVSUSB3Core_INCLUDE_DIR}")
endif()
set(MVSUSB3Core_FOUND TRUE)
EOF

echo "[OK] MVS SDK installed and CMake config generated"
