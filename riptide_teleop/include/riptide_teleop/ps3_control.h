#ifndef PS3_CONTROL_H
#define PS3_CONTROL_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Accel.h"
#include "std_msgs/Float64.h"
#include "riptide_msgs/Depth.h"

class Accel
{
 private:
  ros::NodeHandle nh;
  ros::Publisher angular_pub;
  ros::Publisher linear_x_pub;
  ros::Publisher linear_y_pub;
  ros::Publisher depth_pub;
  ros::Subscriber joy_sub;

  geometry_msgs::Vector3 angular_accel;
  std_msgs::Float64 linear_x_accel;
  std_msgs::Float64 linear_y_accel;
  riptide_msgs::Depth depth_cmd;
  float current_depth_cmd;

  void publish_commands();

 public:
  Accel();
  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
  void loop();
};

#endif
