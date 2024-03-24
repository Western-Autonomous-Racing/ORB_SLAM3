#ifndef __MAP_CREATION_H__
#define __MAP_CREATION_H__

#include <iostream>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "../ORB_SLAM3/include/System.h"
#include "../ORB_SLAM3/include/ImuTypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class MapNode : public rclcpp::Node
{
public:

    MapNode(ORB_SLAM3::System *pSLAMal); // Constructor - gets SLAM system pointer
    ~MapNode();

    void RunMapping(); // run everything else
    void RefinePointCloud();
    void OccupancyGrid();
    double GetSeconds(builtin_interfaces::msg::Time stamp);
    // void record_Data(std::shared_ptr<sensor_msgs::msg::PointCloud2> raw_map_points_msg, std::shared_ptr<nav_msgs::msg::Odometry> odom_msg);

private:


    ORB_SLAM3::System *mpSLAM; // Pointer to the SLAM system

    // Cloud point maps
    std::vector<ORB_SLAM3::MapPoint*> raw_map_points_, refined_map_points_; //raw_map 3D points, refined_map 2D points

    // Trajectory
    std::vector<ORB_SLAM3::KeyFrame*> trajectory_;

    // Pose
    Sophus::SE3f current_pose_, prev_pose_;

    // publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_map_points_pub_, refined_map_points_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::msg::TransformStamped odom_trans_;

    thread *publishing_thread_;
};


#endif // __EXTRACT_POINTS_H__