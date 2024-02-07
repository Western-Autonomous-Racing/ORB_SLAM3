echo "Building ROS2 nodes"

cd Examples/ROS2/ORB_SLAM3
rm -rf build
rm -rf install
rm -rf log
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select orb_slam3_ros2 --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 
. install/setup.bash