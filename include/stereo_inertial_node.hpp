#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "../ORB_SLAM3/include/System.h"
#include "../ORB_SLAM3/include/ImuTypes.h"

using namespace std;

class StereoInertialNode : public rclcpp::Node
{
public:
  StereoInertialNode(ORB_SLAM3::System *pSLAM, const bool bRect, const bool bClahe, string stereo_config);
  ~StereoInertialNode();

  void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
  void GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  void GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  double GetSeconds(builtin_interfaces::msg::Time stamp);
  void SyncWithImu();

  queue<sensor_msgs::msg::Image::ConstSharedPtr> imgLeftBuf, imgRightBuf;
  queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;

private:
  thread *syncThread_;
  mutex mBufMutexLeft, mBufMutexRight, mBufMutex;
  ORB_SLAM3::System *mpSLAM;

  bool mbClahe;
  bool do_rectify;
  cv::Mat M1l, M2l, M1r, M2r;

  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr sub_img_left_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr sub_img_right_;
};

#endif