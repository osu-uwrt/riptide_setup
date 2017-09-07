#ifndef COMMANDER_H
#define COMMANDER_H

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Joy.h"
#include "riptide_msgs/OdomWithAccel.h"

class Accel
{
 private:
  ros::NodeHandle nh;
  ros::Publisher target_odom;
  ros::Subscriber js;
  riptide_msgs::OdomWithAccel target;

 public:
  Accel();
  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
  void loop();
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "commander");
  Accel accel;
  accel.loop();
}

#endif
