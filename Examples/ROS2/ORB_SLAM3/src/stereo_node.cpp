/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "stereo_node.hpp"
#include <opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core.hpp>

using std::placeholders::_1;

StereoNode::StereoNode(ORB_SLAM3::System* pSLAM, const bool bRect, const bool bClahe, string stereo_config) :
    mpSLAM(pSLAM),
    mbClahe(bClahe),
    do_rectify(bRect),
    Node("stereo_inertial_node")
{
  if(do_rectify)
  {
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(stereo_config, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "ERROR: Wrong path to settings");
      return;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
      RCLCPP_ERROR(this->get_logger(), "ERROR: Calibration parameters to rectify stereo are missing!");
      return;
    }

    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
  }

  sub_img_left_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/stereo_camera/left/image_raw");
  sub_img_right_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/stereo_camera/right/image_raw");
  syncApproximate = std::make_shared<message_filters::Synchronizer<approx_sync_policy>>(approx_sync_policy(10), *sub_img_left_, *sub_img_right_);
  syncApproximate->registerCallback(&StereoNode::SyncStereo, this);
}

StereoNode::~StereoNode()
{
  // Save camera trajectory
  mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  // Stop all threads
  mpSLAM->Shutdown();
}

cv::Mat StereoNode::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    // cout << "Hello there" << endl;
    // cv::imshow("image", cv_ptr->image);
    // cv::waitKey(1);
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

double StereoNode::GetSeconds(builtin_interfaces::msg::Time stamp)
{
  return stamp.sec + stamp.nanosec / 1e9;
}

void StereoNode::SyncStereo(const sensor_msgs::msg::Image::ConstSharedPtr leftImg, const sensor_msgs::msg::Image::ConstSharedPtr rightImg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try
  {
    cv_ptrLeft = cv_bridge::toCvShare(leftImg);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try
  {
    cv_ptrRight = cv_bridge::toCvShare(rightImg);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (do_rectify)
  {
    cv::Mat imLeft, imRight;
    cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
    mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.sec + cv_ptrLeft->header.stamp.nanosec / 1e9);
  }
  else
  {
    mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, GetSeconds(leftImg->header.stamp));
  }
}