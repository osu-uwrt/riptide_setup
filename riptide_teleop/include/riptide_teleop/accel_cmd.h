#ifndef ACCEL_CMD_H
#define ACCEL_CMD_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Accel.h"

class Accel
{
 private:
  ros::NodeHandle nh;
  ros::Publisher accels;
  ros::Subscriber js;
  geometry_msgs::Accel accel;

 public:
  Accel();
  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
  void loop();
};

#endif
