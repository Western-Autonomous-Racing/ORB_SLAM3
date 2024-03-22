#include "../include/map_node.hpp"

using namespace std;

MapNode::MapNode(ORB_SLAM3::System *pSLAM)
    : Node("map_node"), mpSLAM(pSLAM)
{

    raw_map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_map_points", 100);
    // trajectory_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/trajectory", 100);
    pose_stamped_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped", 100);
    // refined_map_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/refined_map_points", 100);
    // occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 100);
    

    publishing_thread_ = new thread(&MapNode::GeneratingMap, this);
}

MapNode::~MapNode()
{
    publishing_thread_->join();
    delete publishing_thread_;
    // Stop all threads

    if (mpSLAM)
    {
        mpSLAM->Shutdown();
        delete mpSLAM;
    }
}

// Map points -> GetAllMapPoints() (Atlas.h/Map.h)
// Trajectory -> GetAllKeyFrames() (Atlas.h/Map.h)
// Get Current Pose() -> TrackStereo(), etc. (System.h) (Tcw)
void MapNode::GeneratingMap()
{
    const int num_coords = 3; // x, y, z
    sensor_msgs::msg::PointCloud2 raw_map_points_msg;
    nav_msgs::msg::Odometry trajectory_msg;
    geometry_msgs::msg::PoseStamped pose_msg;

    while (rclcpp::ok())
    {
        // Get the map
        raw_map_points_ = mpSLAM->GetAtlas()->GetAllMapPoints();
        // Get the trajectory
        trajectory_ = mpSLAM->GetAtlas()->GetAllKeyFrames();
        // Get the current pose
        current_pose_ = mpSLAM->getCurrentTwC();

        // Refine the point cloud
        RefinePointCloud();

        // Create the occupancy grid
        OccupancyGrid();

        // Publish the map topics

        rclcpp::Time frame_timestamp = this->now();

        // Publish the raw map points
        raw_map_points_msg.header.frame_id = "map";
        raw_map_points_msg.header.stamp = frame_timestamp;
        raw_map_points_msg.height = 1;
        raw_map_points_msg.width = raw_map_points_.size();
        raw_map_points_msg.is_dense = true;
        raw_map_points_msg.is_bigendian = false;
        raw_map_points_msg.point_step = num_coords * sizeof(float);
        raw_map_points_msg.row_step = raw_map_points_msg.point_step * raw_map_points_msg.width;
        raw_map_points_msg.fields.resize(num_coords);

        for (int i = 0; i < num_coords; i++)
        {
            raw_map_points_msg.fields[i].name = "xyz"[i];
            raw_map_points_msg.fields[i].offset = i * sizeof(float);
            raw_map_points_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
            raw_map_points_msg.fields[i].count = 1;
        }

        raw_map_points_msg.data.resize(raw_map_points_msg.row_step * raw_map_points_msg.height);
        unsigned char *raw_map_points_data = &(raw_map_points_msg.data[0]);
        float coords_array[num_coords];

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
                coords_array[2] = -raw_map_points_.at(i)->GetWorldPos()(1);

                memcpy(raw_map_points_data + i * raw_map_points_msg.point_step, coords_array, num_coords * sizeof(float));
            }

            raw_map_points_pub_->publish(raw_map_points_msg);
        }

        // Publish the trajectory
        trajectory_msg.header.frame_id = "trajectory";
        trajectory_msg.header.stamp = frame_timestamp;

        // Publish the pose
        pose_msg.header.frame_id = "pose";
        pose_msg.header.stamp = frame_timestamp;
        pose_msg.pose.position.x = current_pose_.translation()(2);
        pose_msg.pose.position.y = -current_pose_.translation()(0);
        pose_msg.pose.position.z = -current_pose_.translation()(1);
        
        Eigen::Quaternionf R = current_pose_.unit_quaternion();
        pose_msg.pose.orientation.x = R.x();
        pose_msg.pose.orientation.y = R.y();
        pose_msg.pose.orientation.z = R.z();
        pose_msg.pose.orientation.w = R.w();

        pose_stamped_pub_->publish(pose_msg);

        // Publish the refined map points


        // Publish the occupancy grid
    }
}

// void RefinePointCloud();
void MapNode::RefinePointCloud()
{
    // Copy the raw map points to the refined map points
    // refined_map_points_ = raw_map_points_;

    // while(rclcpp::ok())
    // {
    //     // print coordinates of map_points
    //     for (int i = 0; i < raw_map_points_->size(); i++)
    //     {
    //         cout << "Map Point " << i << ": " << raw_map_points_->at(i)->GetWorldPos() << endl;
    //     }
    // }
}

// void OccupancyGrid();
void MapNode::OccupancyGrid()
{
}