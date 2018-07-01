#ifndef PS3_DEBUG_IMAGE_H
#define PS3_DEBUG_IMAGE_H

#include "ros/ros.h"
#include "cmath"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Accel.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/Depth.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class PS3DebugImage
{
 private:
  ros::NodeHandle nh;
  ros::Subscriber imu_sub, depth_sub, fwd_img_sub, down_img_sub, darknet_img_sub;
  ros::Subscriber cmd_depth_sub, cmd_attitude_sub, cmd_accel_sub;
  ros::Publisher fwd_img_pub, down_img_pub, darknet_img_pub;

  geometry_msgs::Vector3 euler_rpy, linear_accel, delta_attitude;
  geometry_msgs::Accel cmd_accel;
  double depth, cmd_depth;

 public:
  PS3DebugImage();
  void LoadProperty(std::string name, double &param);
  void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
  void ImuCB(const riptide_msgs::Imu::ConstPtr& imu_msg);
  void AccelCB(const geometry_msgs::Accel::ConstPtr& accel_msg);
  void Loop();
};

#endif
