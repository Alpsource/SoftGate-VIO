source /opt/ros/humble/setup.bash
source ~/sensor_ws/install/setup.bash
# Fast DDS Configuration
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/neurolab/alp/SoftGate-VIO/fastdds.xml
# unset CYCLONEDDS_URI
# Cyclone DDS Configuration
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/neurolab/alp/SoftGate-VIO/cyclonedds.xml
sudo sysctl -w net.core.rmem_max=2147483647
ros2 launch sensor_system bringup.launch.py