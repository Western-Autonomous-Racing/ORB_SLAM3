#include "../include/mono_inertial_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui.hpp>

using std::placeholders::_1;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System *pSLAM, const bool bClahe) : mpSLAM(pSLAM),
                                                                                  mbClahe(bClahe),
                                                                                  Node("mono_inertial_node")
{
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 1000, std::bind(&MonoInertialNode::GrabImu, this, _1));
  sub_img0_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 100, std::bind(&MonoInertialNode::GrabImage, this, _1));

  syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);
}

MonoInertialNode::~MonoInertialNode()
{
  // Delete sync thread
  syncThread_->join();
  delete syncThread_;

  // Save camera trajectory
  mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  // Stop all threads
  mpSLAM->Shutdown();
}

void MonoInertialNode::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

void MonoInertialNode::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  mBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

cv::Mat MonoInertialNode::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

double MonoInertialNode::GetSeconds(builtin_interfaces::msg::Time stamp)
{
  return stamp.sec + stamp.nanosec / 1e9;
}

void MonoInertialNode::SyncWithImu()
{
  while (rclcpp::ok())
  {
    auto startTime = std::chrono::high_resolution_clock::now();

    cv::Mat im;
    double tImg = 0;
    double tImu = 0;
    if (!img0Buf.empty() && !imuBuf.empty())
    {
      tImg = this->GetSeconds(img0Buf.front()->header.stamp);
      if (tImg > this->GetSeconds(imuBuf.back()->header.stamp))
        continue;

      this->mBufMutex.lock();
      im = GetImage(img0Buf.front());
      img0Buf.pop();
      this->mBufMutex.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mBufMutex.lock();
      if (!imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!imuBuf.empty() && (imuBuf.front()->header.stamp.sec + imuBuf.front()->header.stamp.nanosec / 1e9) <= tImg)
        {
          double t = this->GetSeconds(imuBuf.front()->header.stamp);
          // float acc_x_offset = 0.231939;
          // float acc_y_offset = 0;
          // float acc_z_offset = 0.070294;
          // float angvel_x_offset = 0.482711; // --
          // float angvel_y_offset = 0.353649; // --
          // float angvel_z_offset = 0.024390;

          // float acc_x = imuBuf.front()->linear_acceleration.x + acc_x_offset;
          // float acc_y = imuBuf.front()->linear_acceleration.y - acc_y_offset;
          // float acc_z = imuBuf.front()->linear_acceleration.z - acc_z_offset;
          // float gyro_x = imuBuf.front()->angular_velocity.x + angvel_x_offset;
          // float gyro_y = imuBuf.front()->angular_velocity.y + angvel_y_offset;
          // float gyro_z = imuBuf.front()->angular_velocity.z + angvel_z_offset;

          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          // cv::Point3f acc(acc_x, acc_y, acc_z);
          // cv::Point3f gyr(gyro_x, gyro_y, gyro_z);
          // cv::Point3f acc(0.00000001, 0.00000001, 0.00000001);
          // cv::Point3f gyr(0.00000001, 0.00000001, 0.00000001);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
          // cout << "Timestamp: " << t << endl;
          // cout << "Linear Acceleration - x: " << acc_x << " y: " << acc_y << " z: " << acc_y << endl;
          // cout << "Angular Velocity - x: " << gyro_x << " y: " << gyro_y << " z: " << gyro_z << endl;

          imuBuf.pop();
        }
      }
      mBufMutex.unlock();

      if (mbClahe)
        mClahe->apply(im, im);

      // cout << "Linear Acceleration - x: " << vImuMeas.back().a << " Timestamp: " << vImuMeas.back().t;
      mpSLAM->TrackMonocular(im, tImg, vImuMeas);

      std::chrono::milliseconds tSleep(32);
      std::this_thread::sleep_for(tSleep);
    }
  }
}