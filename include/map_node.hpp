#ifndef __MAP_CREATION_H__
#define __MAP_CREATION_H__

#define RAW_MAP_POINT_TOPIC "/raw_map_points"
#define REFINED_MAP_POINT_TOPIC "/refined_map_points"
#define ODOM_TOPIC "/egocar/odom"
#define RAW_COORDS 3
#define REFINED_COORDS 2
#define UPPER_BOUND_DEFAULT 4.5
#define LOWER_BOUND_DEFAULT 0.0

#include <iostream>
#include <thread>
#include <mutex>
#include <memory>
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

    MapNode(ORB_SLAM3::System *pSLAM, string config); // Constructor - gets SLAM system pointer
    ~MapNode();

    void RunMapping(); // run everything else
    void MapPointsPublish(rclcpp::Time frame_timestamp); // publish raw map points
    void OdomPublish(rclcpp::Time frame_timestamp, rclcpp::Time prev_timestamp);
    void RefinePointCloud(rclcpp::Time frame_timestamp);
    double GetSeconds(builtin_interfaces::msg::Time stamp);

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

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::msg::TransformStamped odom_trans_;

    thread *publishing_thread_;

    // subscribers to save
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_map_points_sub_, refined_map_points_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    string map_name_, map_dir_;
    int map_type_;

    sensor_msgs::msg::PointCloud2 raw_map_points_msg_, refined_map_points_msg_;
    nav_msgs::msg::Odometry odom_msg_;

    double cloud_transform_z_, upper_bound_, lower_bound_;
    int publish_last_;
};


#endif // __EXTRACT_POINTS_H__