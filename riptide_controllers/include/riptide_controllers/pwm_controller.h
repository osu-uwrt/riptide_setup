#ifndef THRUST_CAL_H
#define THRUST_CAL_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "riptide_msgs/Bat.h"
#include "riptide_msgs/PwmStamped.h"
#include "riptide_msgs/ThrustStamped.h"

class ThrustCal
{
 private:
  ros::NodeHandle nh;
  ros::Subscriber kill_it_with_fire;
  ros::Subscriber thrust, outta_juice;
  ros::Publisher pwm;
  riptide_msgs::PwmStamped us;
  ros::Time alive;
  bool dead, low;
  double min_voltage;
  int counterclockwise(double raw_force, int thruster);
  int clockwise(double raw_force, int thruster);
  float ccw_coeffs[4][2]; //counterclockwise thrust slopes
  float cw_coeffs[4][2]; //clockwise thrust slopes

 public:
  ThrustCal();
  void callback(const riptide_msgs::ThrustStamped::ConstPtr& thrust);
  void killback(const std_msgs::Empty::ConstPtr& thrust);
  void voltsbacken(const riptide_msgs::Bat::ConstPtr& bat_stat);
  void loop();
};

#endif
