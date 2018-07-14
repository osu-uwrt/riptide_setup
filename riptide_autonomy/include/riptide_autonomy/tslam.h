#ifndef TSLAM_H
#define TSLAM_H
//#define DEPTH_OFFSET 0.1 // Save, these were used by arduino
//#define DEPTH_SLOPE 1

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include <cmath>
using namespace std;

class TSlam
{

private:
  ros::NodeHandle nh;
  ros::Subscriber go_sub, attitude_sub, abort_sub;

  int currentTaskHeading;

public:
  TSlam();
  void Go(const std_msgs::Int8::ConstPtr& task);
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& attitude);
  void Abort(const std_msgs::Empty::ConstPtr& data);
};

#endif