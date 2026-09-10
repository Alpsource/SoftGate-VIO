#!/bin/bash
# Build script for Jetson Orin NX.
#
# CRITICAL — OpenCV selection.
# This machine carries three OpenCV installs:
#   4.5.4   /lib/aarch64-linux-gnu   Ubuntu runtime; what cv_bridge links. No cmake config.
#   4.8.0   /usr/lib                 JetPack libopencv-dev. Headers + cmake config.
#   4.13.0  /usr/local               Self-built.
#
# ov_msckf must not link the self-built 4.13. Loading 4.13 alongside cv_bridge's
# 4.5.4 in one process makes each library read the other's cv::Mat header with the
# wrong ABI, yielding garbage dimensions and multi-TB allocations -> SIGABRT.
# It fires on the first rgb8 frame, because cv_bridge allocates a new Mat for the
# colour->gray conversion. mono8 input (ZED X live) is wrapped without allocating
# and never crosses the seam -- which is why live ran fine while every VIODE
# timing run died instantly.
#
# CMAKE_IGNORE_PATH is the lever that works. ov_core and ov_msckf both call
# find_package(OpenCV 3 QUIET) first; that failed probe discards any -DOpenCV_DIR,
# and the OpenCV 4 search that follows reaches /usr/local before /usr.
# (OpenCV_ROOT has no effect here; CMAKE_PREFIX_PATH would break ament discovery.)
#
# ENABLE_ARUCO_TAGS=OFF: JetPack's 4.8 ships no contrib module, so opencv2/aruco.hpp
# does not exist. The VIODE and ZED X configs set use_aruco: false regardless.

set -e

OPENCV_IGNORE="/usr/local/lib/cmake/opencv4"

echo "[build_jetson] Ignoring OpenCV cmake dir : ${OPENCV_IGNORE}"
echo "[build_jetson] Building against JetPack OpenCV 4.8 (/usr/lib/cmake/opencv4)"

rm -rf build install log
source /opt/ros/humble/setup.bash

colcon build --symlink-install --cmake-args \
    "-DCMAKE_IGNORE_PATH=${OPENCV_IGNORE}" \
    -DENABLE_ARUCO_TAGS=OFF \
    -DEigen3_DIR=/usr/share/eigen3/cmake \
    -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
    -DCMAKE_BUILD_TYPE=Release

# The mismatch is silent until the first rgb8 frame, so assert it here instead.
BIN=install/ov_msckf/lib/ov_msckf/run_subscribe_msckf
if [[ -f "$BIN" ]]; then
    if ldd "$BIN" | grep -q "libopencv_core.so.413"; then
        echo "[build_jetson] ERROR: linked self-built OpenCV 4.13 — VIO will abort on rgb8 input."
        exit 1
    fi
    echo "[build_jetson] OpenCV OK: $(ldd "$BIN" | grep -o 'libopencv_core\.so\.[0-9]*' | head -1)"
fi
