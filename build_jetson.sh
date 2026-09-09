#!/bin/bash
# Build script for Jetson Orin NX.
#
# CRITICAL: ov_msckf CMakeLists tries OpenCV 3 before OpenCV 4.
# cv_bridge (ROS2 Humble) is compiled against OpenCV 4. A version mismatch
# corrupts cv::Mat struct sizes at runtime (garbage TB-scale allocations →
# SIGABRT). We must force CMake to use the same OpenCV as cv_bridge.
#
# We detect cv_bridge's OpenCV path at build time so this stays correct
# even if the JetPack system is upgraded.

set -e

# Auto-detect the OpenCV cmake directory from cv_bridge's linked library.
# This guarantees ov_msckf uses the exact same OpenCV build as cv_bridge.
CV_BRIDGE_LIB=/opt/ros/humble/lib/libcv_bridge.so
OPENCV_CMAKE_DIR=""

if [[ -f "$CV_BRIDGE_LIB" ]]; then
    OPENCV_CORE_SO=$(ldd "$CV_BRIDGE_LIB" 2>/dev/null \
        | grep libopencv_core | awk '{print $3}' | head -1)
    if [[ -n "$OPENCV_CORE_SO" ]]; then
        OPENCV_LIB_DIR=$(dirname "$OPENCV_CORE_SO")
        # Look for OpenCVConfig.cmake alongside or above the lib directory
        OPENCV_CMAKE_DIR=$(find \
            "${OPENCV_LIB_DIR}/cmake" \
            "${OPENCV_LIB_DIR}/../lib/cmake" \
            /usr/lib/aarch64-linux-gnu/cmake \
            /usr/local/lib/cmake \
            /usr/share/OpenCV \
            -name "OpenCVConfig.cmake" 2>/dev/null \
            | head -1 | xargs -r dirname)
    fi
fi

if [[ -z "$OPENCV_CMAKE_DIR" ]]; then
    echo "[build_jetson] WARNING: Could not auto-detect cv_bridge OpenCV cmake dir."
    echo "  Falling back to: /usr/lib/aarch64-linux-gnu/cmake/opencv4"
    OPENCV_CMAKE_DIR="/usr/lib/aarch64-linux-gnu/cmake/opencv4"
fi

echo "[build_jetson] Using OpenCV cmake dir: ${OPENCV_CMAKE_DIR}"

rm -rf build install log
source /opt/ros/humble/setup.bash

colcon build --symlink-install --cmake-args \
    "-DOpenCV_DIR=${OPENCV_CMAKE_DIR}" \
    -DEigen3_DIR=/usr/share/eigen3/cmake \
    -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
    -DCMAKE_BUILD_TYPE=Release
