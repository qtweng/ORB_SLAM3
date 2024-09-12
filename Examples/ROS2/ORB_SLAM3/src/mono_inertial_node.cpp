#include "mono_inertial_node.hpp"

#include<rclcpp/rclcpp.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/core.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<sensor_msgs/image_encodings.hpp>
#include <opencv2/highgui.hpp>
#include <pthread.h>
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System* pSLAM, const bool bClahe) : 
    mpSLAM(pSLAM),
    mbClahe(bClahe), 
    Node("mono_inertial_node")
{
 
  imu_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions imu_options;
  imu_options.callback_group = imu_callback_group_;

  rclcpp::SubscriptionOptions image_options;
  image_options.callback_group = image_callback_group_;

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS().keep_last(20), std::bind(&MonoInertialNode::GrabImu, this, _1), imu_options);
  sub_img0_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SensorDataQoS().keep_last(2), std::bind(&MonoInertialNode::GrabImage, this, _1), image_options);

  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 1);
  syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);
  pthread_setname_np(syncThread_->native_handle(), "SyncThread");
}

MonoInertialNode::~MonoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

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
  if (img0Buf.size() == 1)
    img0Buf.pop();
  img0Buf.push(img_msg);
  mBufMutex.unlock();
}

void MonoInertialNode::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg, cv::Mat *output_image)
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

  cv_ptr->image.copyTo(*output_image);
}

double MonoInertialNode::GetSeconds(builtin_interfaces::msg::Time stamp)
{
  return stamp.sec + stamp.nanosec / 1e9;
}

geometry_msgs::msg::PoseStamped MonoInertialNode::ConvertToRosPose(const Sophus::SE3f &pose, const std::string &frame_id)
{
  geometry_msgs::msg::PoseStamped ros_pose;
  ros_pose.header.stamp = this->now();
  ros_pose.header.frame_id = frame_id;

  // Convert translation
  ros_pose.pose.position.x = pose.translation()(0);
  ros_pose.pose.position.y = pose.translation()(1);
  ros_pose.pose.position.z = pose.translation()(2);

  // Convert rotation (Sophus::SE3f uses Eigen::Quaternion)
  Eigen::Quaternionf quat(pose.unit_quaternion());
  ros_pose.pose.orientation.x = quat.x();
  ros_pose.pose.orientation.y = quat.y();
  ros_pose.pose.orientation.z = quat.z();
  ros_pose.pose.orientation.w = quat.w();

  return ros_pose;
}

void MonoInertialNode::SyncWithImu()
{
  cv::Mat im;
  double tImg = 0;
  double tImu = 0;
  pid_t pid = getpid();  // Get the PID of the process
  std::thread::id tid = std::this_thread::get_id(); // Get the TID of the current thread

  RCLCPP_INFO(this->get_logger(), "SyncWithImu thread started. PID: %d, TID: %zu", pid, std::hash<std::thread::id>{}(tid));
    
  while(1)
  {
    if (img0Buf.size() == 1 && !imuBuf.empty())
    {
      mBufMutex.lock();
      tImg = this->GetSeconds(img0Buf.front()->header.stamp);
      if (this->GetSeconds(imuBuf.back()->header.stamp) < tImg) {
        this->mBufMutex.unlock();
        continue;
      }
    
      GetImage(img0Buf.front(), &im);
      img0Buf.pop();
      mBufMutex.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mBufMutex.lock();
      if(!imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while (!imuBuf.empty() && this->GetSeconds(imuBuf.front()->header.stamp) <= tImg)
        {
          tImu = this->GetSeconds(imuBuf.front()->header.stamp);
          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tImu));

          imuBuf.pop();
        }
      }
      mBufMutex.unlock();
      
      if(mbClahe)
        mClahe->apply(im,im);

      Sophus::SE3f Tcw = mpSLAM->TrackMonocular(im, tImg, vImuMeas);
      Sophus::SE3f Twc = Tcw.inverse();

      geometry_msgs::msg::PoseStamped ros_pose = ConvertToRosPose(Twc, "world");
      pub_pose_->publish(ros_pose);
    }
    
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}
