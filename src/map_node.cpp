#include "../include/map_node.hpp"
#include <opencv2/core.hpp>
#include <cmath>

using namespace std;
using std::placeholders::_1;

MapNode::MapNode(ORB_SLAM3::System *pSLAM, string config)
    : Node("map_node"), mpSLAM(pSLAM), tf_broadcaster_(this)
{
    
    cv::FileStorage fsSettings(config.c_str(), cv::FileStorage::READ);

    cv::FileNode node = fsSettings["System.CloudTransformZ"];

    if (!node.empty())
    {
        cloud_transform_z_ = static_cast<double>(node);
    }
    else
    {
        cloud_transform_z_ = 0.0;
    }

    raw_map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(RAW_MAP_POINT_TOPIC, 100);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 100);
    refined_map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(REFINED_MAP_POINT_TOPIC, 100);

    publishing_thread_ = new thread(&MapNode::RunMapping, this);
}

MapNode::~MapNode()
{
    publishing_thread_->join();
    delete publishing_thread_;
    // Stop all
}

// Map points -> GetAllMapPoints() (Atlas.h/Map.h)
// Trajectory -> GetAllKeyFrames() (Atlas.h/Map.h)
// Get Current Pose() -> TrackStereo(), etc. (System.h) (Tcw)
void MapNode::RunMapping()
{
    rclcpp::Time frame_timestamp, prev_timestamp = rclcpp::Time(0);
    prev_pose_ = Sophus::SE3f();

    while (rclcpp::ok())
    {
        frame_timestamp = this->now();

        MapPointsPublish(frame_timestamp);

        OdomPublish(frame_timestamp, prev_timestamp);

        prev_timestamp = frame_timestamp;
        prev_pose_ = current_pose_;
    }
}

double MapNode::GetSeconds(builtin_interfaces::msg::Time stamp)
{
    return stamp.sec + stamp.nanosec / 1000000000.0;
}

void MapNode::MapPointsPublish(rclcpp::Time frame_timestamp)
{
    // Get the map
    raw_map_points_ = mpSLAM->GetAtlas()->GetAllMapPoints();

    // Publish the map topics

    // Publish the raw map points
    raw_map_points_msg_.header.frame_id = "odom"; // "map"
    raw_map_points_msg_.header.stamp = frame_timestamp;
    raw_map_points_msg_.height = 1;
    raw_map_points_msg_.width = raw_map_points_.size();
    raw_map_points_msg_.is_dense = true;
    raw_map_points_msg_.is_bigendian = false;
    raw_map_points_msg_.point_step = RAW_COORDS * sizeof(float);
    raw_map_points_msg_.row_step = raw_map_points_msg_.point_step * raw_map_points_msg_.width;
    raw_map_points_msg_.fields.resize(RAW_COORDS);

    for (int i = 0; i < RAW_COORDS; i++)
    {
        raw_map_points_msg_.fields[i].name = "xyz"[i];
        raw_map_points_msg_.fields[i].offset = i * sizeof(float);
        raw_map_points_msg_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        raw_map_points_msg_.fields[i].count = 1;
    }

    raw_map_points_msg_.data.resize(raw_map_points_msg_.row_step * raw_map_points_msg_.height);
    unsigned char *raw_map_points_data = &(raw_map_points_msg_.data[0]);
    float coords_array[RAW_COORDS];

    if (raw_map_points_.size() > 0)
    {

        for (int i = 0; i < raw_map_points_.size(); i++)
        {
            if (raw_map_points_.at(i)->isBad())
            {
                continue;
            }

            coords_array[0] = raw_map_points_.at(i)->GetWorldPos()(2);
            coords_array[1] = -raw_map_points_.at(i)->GetWorldPos()(0);
            coords_array[2] = -raw_map_points_.at(i)->GetWorldPos()(1) - (raw_map_points_.at(i)->GetWorldPos()(2) * tan(cloud_transform_z_));

            memcpy(raw_map_points_data + i * raw_map_points_msg_.point_step, coords_array, RAW_COORDS * sizeof(float));
        }

        RefinePointCloud(frame_timestamp);

        raw_map_points_pub_->publish(raw_map_points_msg_);
        refined_map_points_pub_->publish(refined_map_points_msg_);
    }
}

// void RefinePointCloud();
void MapNode::RefinePointCloud(rclcpp::Time frame_timestamp)
{
    // Copy the raw map points to the refined map points
    
    refined_map_points_ = raw_map_points_;
;
    refined_map_points_msg_.header.frame_id = "odom"; // "map"
    refined_map_points_msg_.header.stamp = frame_timestamp;
    refined_map_points_msg_.height = 1;
    refined_map_points_msg_.width = raw_map_points_.size();
    refined_map_points_msg_.is_dense = true;
    refined_map_points_msg_.is_bigendian = false;
    refined_map_points_msg_.point_step = RAW_COORDS * sizeof(float);
    refined_map_points_msg_.row_step = refined_map_points_msg_.point_step * refined_map_points_msg_.width;
    refined_map_points_msg_.fields.resize(RAW_COORDS);

    for (int i = 0; i < RAW_COORDS; i++)
    {
        refined_map_points_msg_.fields[i].name = "xyz"[i];
        refined_map_points_msg_.fields[i].offset = i * sizeof(float);
        refined_map_points_msg_.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        refined_map_points_msg_.fields[i].count = 1;
    }

    refined_map_points_msg_.data.resize(refined_map_points_msg_.row_step * refined_map_points_msg_.height);
    unsigned char *refined_map_points_data = &(refined_map_points_msg_.data[0]);
    float coords_array[RAW_COORDS];

    // Remove all nullptr elements from the vector
    refined_map_points_.erase(std::remove(refined_map_points_.begin(), refined_map_points_.end(), nullptr), refined_map_points_.end());

    for (int i = 0; i < refined_map_points_.size(); i++)
    {
        if (refined_map_points_.at(i)->isBad())
        {
            continue;
        }

        coords_array[0] = refined_map_points_.at(i)->GetWorldPos()(2);
        coords_array[1] = -refined_map_points_.at(i)->GetWorldPos()(0);
        coords_array[2] = -refined_map_points_.at(i)->GetWorldPos()(1) - (refined_map_points_.at(i)->GetWorldPos()(2) * tan(cloud_transform_z_));

        // Remove the outliers
        if (coords_array[2] < 0.25 || coords_array[2] > 4.5)
        {
            refined_map_points_.at(i) = nullptr;
            continue;
        }

        memcpy(refined_map_points_data + i * refined_map_points_msg_.point_step, coords_array, RAW_COORDS * sizeof(float));
    }

    // Remove all nullptr elements from the vector
    refined_map_points_.erase(std::remove(refined_map_points_.begin(), refined_map_points_.end(), nullptr), refined_map_points_.end());


}

void MapNode::OdomPublish(rclcpp::Time frame_timestamp, rclcpp::Time prev_timestamp)
{

    // Get the current pose
    current_pose_ = mpSLAM->getCurrentTwC().inverse();
    Eigen::Quaternionf R2 = current_pose_.unit_quaternion();

    // Publish odom
    if (GetSeconds(prev_timestamp) == 0.0)
    {
        prev_timestamp = frame_timestamp;
        prev_pose_ = current_pose_;
        cout << "Initial Pose" << endl;
    }

    if (current_pose_.translation()(2) == prev_pose_.translation()(2) && current_pose_.translation()(0) == prev_pose_.translation()(0) && current_pose_.translation()(1) == prev_pose_.translation()(1))
        return;

    // Create odom transform
    odom_trans_.header.stamp = frame_timestamp;
    odom_trans_.header.frame_id = "odom";
    odom_trans_.child_frame_id = "base_link";
    odom_trans_.transform.translation.x = static_cast<double>(current_pose_.translation().z());
    odom_trans_.transform.translation.y = static_cast<double>(-current_pose_.translation().x());
    odom_trans_.transform.translation.z = static_cast<double>(-current_pose_.translation().y());
    odom_trans_.transform.translation.z = 0.0;
    odom_trans_.transform.rotation.x = static_cast<double>(R2.z());
    odom_trans_.transform.rotation.y = static_cast<double>(-R2.x());
    odom_trans_.transform.rotation.z = static_cast<double>(-R2.y());
    odom_trans_.transform.rotation.w = static_cast<double>(R2.w());

    tf_broadcaster_.sendTransform(odom_trans_);

    odom_msg_.header.stamp = frame_timestamp;
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";
    odom_msg_.pose.pose.position.x = static_cast<double>(current_pose_.translation().z());
    odom_msg_.pose.pose.position.y = static_cast<double>(-current_pose_.translation().x());
    odom_msg_.pose.pose.position.z = static_cast<double>(-current_pose_.translation().y());
    odom_msg_.pose.pose.position.z = 0.0;

    odom_msg_.pose.pose.orientation.x = static_cast<double>(R2.z());
    odom_msg_.pose.pose.orientation.y = static_cast<double>(-R2.x());
    odom_msg_.pose.pose.orientation.z = static_cast<double>(-R2.y());
    odom_msg_.pose.pose.orientation.w = static_cast<double>(R2.w());
    odom_msg_.pose.covariance = {0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.01, 0,
                                 0, 0, 0, 0, 0, 0.01};

    odom_msg_.twist.twist.linear.x = static_cast<double>(current_pose_.translation().z() - prev_pose_.translation().z()) / (GetSeconds(frame_timestamp) - GetSeconds(prev_timestamp));
    odom_msg_.twist.twist.linear.y = static_cast<double>((-current_pose_.translation().x()) - (-prev_pose_.translation().x())) / (GetSeconds(frame_timestamp) - GetSeconds(prev_timestamp));
    odom_msg_.twist.twist.linear.z = static_cast<double>((-current_pose_.translation().y()) - (-prev_pose_.translation().y())) / (GetSeconds(frame_timestamp) - GetSeconds(prev_timestamp));
    odom_msg_.twist.twist.angular.x = static_cast<double>(current_pose_.unit_quaternion().z() - prev_pose_.unit_quaternion().z()) / (GetSeconds(frame_timestamp) - GetSeconds(prev_timestamp));
    odom_msg_.twist.twist.angular.y = static_cast<double>((-current_pose_.unit_quaternion().x()) - (-prev_pose_.unit_quaternion().x())) / (GetSeconds(frame_timestamp) - GetSeconds(prev_timestamp));
    odom_msg_.twist.twist.angular.z = static_cast<double>((-current_pose_.unit_quaternion().y()) - (-prev_pose_.unit_quaternion().y())) / (GetSeconds(frame_timestamp) - GetSeconds(prev_timestamp));
    odom_msg_.twist.covariance = {0.01, 0, 0, 0, 0, 0,
                                  0, 0.01, 0, 0, 0, 0,
                                  0, 0, 0.01, 0, 0, 0,
                                  0, 0, 0, 0.01, 0, 0,
                                  0, 0, 0, 0, 0.01, 0,
                                  0, 0, 0, 0, 0, 0.01};

    odom_pub_->publish(odom_msg_);    
}