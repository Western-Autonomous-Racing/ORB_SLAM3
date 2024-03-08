#ifndef __STEREO_NODE_HPP__
#define __STEREO_NODE_HPP__

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../../include/System.h"
#include "../include/ImuTypes.h"

using namespace std;

class StereoNode : public rclcpp::Node
{
public:
  StereoNode(ORB_SLAM3::System *pSLAM, const bool bRect, const bool bClahe, string stereo_config);
  ~StereoNode();

  void SyncStereo(const sensor_msgs::msg::Image::ConstSharedPtr leftImg, const sensor_msgs::msg::Image::ConstSharedPtr rightImg);
  cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
  double GetSeconds(builtin_interfaces::msg::Time stamp);

  queue<sensor_msgs::msg::Image::ConstSharedPtr> imgLeftBuf, imgRightBuf;

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approx_sync_policy;
  thread *syncThread_;
  mutex mBufMutexLeft, mBufMutexRight;
  ORB_SLAM3::System *mpSLAM;

  bool mbClahe;
  bool do_rectify;
  cv::Mat M1l, M2l, M1r, M2r;

  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_img_left_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_img_right_;

  std::shared_ptr<message_filters::Synchronizer<approx_sync_policy>> syncApproximate;
};

#endif