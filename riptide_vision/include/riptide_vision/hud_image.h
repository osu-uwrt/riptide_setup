#ifndef HUD_IMAGE_H
#define HUD_IMAGE_H

#include "ros/ros.h"
#include "cmath"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Accel.h"
#include "sensor_msgs/image_encodings.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/Depth.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
using namespace cv;

class HUDImage
{
 private:
  ros::NodeHandle nh;
  ros::Subscriber imu_sub, depth_sub, fwd_img_sub, down_img_sub, darknet_img_sub;
  ros::Subscriber cmd_attitude_sub, cmd_depth_sub, cmd_accel_sub;
  ros::Publisher fwd_img_pub, down_img_pub, darknet_img_pub;

  geometry_msgs::Vector3 euler_rpy, cmd_euler_rpy, linear_accel, cmd_linear_accel;
  double depth, cmd_depth;
  double width, height, topMarger, botMargin;

 public:
  HUDImage();
  void InitMsgs();
  void ForwardImgCB(const sensor_msgs::ImageConstPtr& msg);
  void DownwardImgCB(const sensor_msgs::ImageConstPtr& msg);
  void DarknetImgCB(const sensor_msgs::ImageConstPtr& msg);
  void ImuCB(const riptide_msgs::Imu::ConstPtr& imu_msg);
  void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
  void CmdAttitudeCB(const geometry_msgs::Vector3::ConstPtr& cmd_msg);
  void CmdDepthCB(const riptide_msgs::DepthCommand::ConstPtr& cmd_msg);
  void CmdAccelCB(const geometry_msgs::Accel::ConstPtr& cmd_msg);
  void Loop();
};

#endif
