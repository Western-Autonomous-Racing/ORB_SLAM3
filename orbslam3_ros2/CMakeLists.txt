cmake_minimum_required(VERSION 3.5)
project(orbslam3_ros2)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

message(STATUS "Project source dir: ${PROJECT_SOURCE_DIR}")

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/ORB_SLAM3
   ${PROJECT_SOURCE_DIR}/ORB_SLAM3/include
   ${PROJECT_SOURCE_DIR}/ORB_SLAM3/include/CameraModels
   ${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/Sophus
   ${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
   ${OpenCV_LIBS} 
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
   ${PROJECT_SOURCE_DIR}/ORB_SLAM3/lib/libORB_SLAM3.so
   -lboost_system
)

add_executable(mono_inertial
   src/mono_inertial_node.cpp
   src/ros_mono_inertial.cc
)

add_executable(stereo_inertial
   src/stereo_inertial_node.cpp
   src/ros_stereo_inertial.cc
)

add_executable(stereo
   src/stereo_node.cpp
   src/ros_stereo.cc
)

add_executable(map
   src/map_node.cpp
   src/mono_inertial_node.cpp
   src/stereo_inertial_node.cpp
   src/stereo_node.cpp
   src/ros_map.cc
)

ament_target_dependencies(mono_inertial
   rclcpp
   sensor_msgs
   cv_bridge
)

ament_target_dependencies(stereo_inertial
   rclcpp
   sensor_msgs
   cv_bridge
)

ament_target_dependencies(stereo
   rclcpp
   sensor_msgs
   message_filters
   cv_bridge
)

ament_target_dependencies(map
   rclcpp
   sensor_msgs
   message_filters
   cv_bridge
   geometry_msgs
   nav_msgs
   tf2_ros
)


target_link_libraries(mono_inertial
   ${LIBS}
)

target_link_libraries(stereo_inertial
   ${LIBS}
)

target_link_libraries(stereo
   ${LIBS}
)

target_link_libraries(map
   ${LIBS}
)

install(TARGETS
   mono_inertial
   stereo_inertial
   stereo
   map_node
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
   launch
   DESTINATION share/${PROJECT_NAME}
)


ament_package()
