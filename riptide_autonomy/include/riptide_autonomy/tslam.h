#ifndef TSLAM_H
#define TSLAM_H

#include "ros/ros.h"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "riptide_msgs/ControlStatus.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_autonomy/be_autonomous.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous;

class TSlam
{

private:
  ros::Subscriber attitude_status_sub, depth_status_sub;

  vector<ros::Subscriber> active_subs;

  double delta_x, delta_y, angle, heading, distance;

  ros::Time acceptable_begin;
  double duration;

  // Create instance to master
  BeAutonomous* master;

public:
  bool enroute;

  TSlam(BeAutonomous* master);
  void Start();
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg);
  void DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr& status_msg);
  void Abort();
};

#endif
