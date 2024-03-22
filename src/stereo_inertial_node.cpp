#include "../include/stereo_inertial_node.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *pSLAM, const bool bRect, const bool bClahe, string stereo_config) : mpSLAM(pSLAM),
                                                                                                                              mbClahe(bClahe),
                                                                                                                              do_rectify(bRect),
                                                                                                                              Node("stereo_inertial_node")
{
  if (do_rectify)
  {
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(stereo_config, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
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

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "ERROR: Calibration parameters to rectify stereo are missing!");
      return;
    }

    cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l, M2l);
    cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r, M2r);
  }

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
  sub_img_left_ = this->create_subscription<sensor_msgs::msg::Image>("/stereo_camera/left/image_raw", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
  sub_img_right_ = this->create_subscription<sensor_msgs::msg::Image>("/stereo_camera/right/image_raw", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

  syncThread_ = new std::thread(&StereoInertialNode::SyncWithImu, this);
}

StereoInertialNode::~StereoInertialNode()
{
  // Save camera trajectory
  mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  // Stop all threads
  mpSLAM->Shutdown();
}

void StereoInertialNode::GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutexLeft.unlock();
}

void StereoInertialNode::GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  mBufMutexRight.unlock();
}

void StereoInertialNode::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}

cv::Mat StereoInertialNode::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
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

double StereoInertialNode::GetSeconds(builtin_interfaces::msg::Time stamp)
{
  return stamp.sec + stamp.nanosec / 1e9;
}

void StereoInertialNode::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while (rclcpp::ok())
  {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !imuBuf.empty())
    {
      tImLeft = this->GetSeconds(imgLeftBuf.front()->header.stamp);
      tImRight = this->GetSeconds(imgRightBuf.front()->header.stamp);

      this->mBufMutexRight.lock();
      while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1)
      {
        imgRightBuf.pop();
        tImRight = this->GetSeconds(imgRightBuf.front()->header.stamp);
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1)
      {
        imgLeftBuf.pop();
        tImLeft = this->GetSeconds(imgLeftBuf.front()->header.stamp);
      }
      this->mBufMutexLeft.unlock();

      if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if (tImLeft > this->GetSeconds(imuBuf.back()->header.stamp))
        continue;

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mBufMutex.lock();
      if (!imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!imuBuf.empty() && this->GetSeconds(imuBuf.front()->header.stamp) <= tImLeft)
        {
          double t = this->GetSeconds(imuBuf.front()->header.stamp);
          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
          imuBuf.pop();
        }
      }
      mBufMutex.unlock();
      if (mbClahe)
      {
        mClahe->apply(imLeft, imLeft);
        mClahe->apply(imRight, imRight);
      }

      if (do_rectify)
      {
        cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
      }

      mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}
