#ifndef __MONO_INERTIAL_NODE_HPP__
#define __MONO_INERTIAL_NODE_HPP__

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include <rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include"../../../include/System.h"
#include"../include/ImuTypes.h"

class MonoInertialNode : public rclcpp::Node
{
public:
    MonoInertialNode(ORB_SLAM3::System* pSLAM, const bool bClahe);
    ~MonoInertialNode();

    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    void SyncWithImu();
    void GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg, cv::Mat* output_image);
    double GetSeconds(builtin_interfaces::msg::Time stamp);
    geometry_msgs::msg::PoseStamped ConvertToRosPose(const Sophus::SE3f &pose, const std::string &frame_id);

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
    queue<sensor_msgs::msg::Image::ConstSharedPtr> img0Buf;

    std::mutex mBufMutex;
    ORB_SLAM3::System* mpSLAM;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    std::thread *syncThread_;
    

private:
    std::shared_ptr<rclcpp::CallbackGroup> imu_callback_group_;
    std::shared_ptr<rclcpp::CallbackGroup> image_callback_group_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr sub_img0_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
};

#endif
