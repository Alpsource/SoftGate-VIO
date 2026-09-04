rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --cmake-args \
-DEigen3_DIR=/usr/share/eigen3/cmake \
-DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
-DCMAKE_BUILD_TYPE=Release