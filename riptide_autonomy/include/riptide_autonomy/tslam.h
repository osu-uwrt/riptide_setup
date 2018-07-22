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
  ros::Timer timer;
  vector<ros::Subscriber> active_subs;

  riptide_msgs::AttitudeCommand attitude_cmd;
  riptide_msgs::DepthCommand depth_cmd;

  double delta_x, delta_y, angle, heading, distance;
  ros::Time acceptable_begin;
  double duration;
  bool clock_is_ticking;
  int validate_id;

  // Create instance to master
  BeAutonomous* master;

public:
  bool enroute;

  TSlam(BeAutonomous* master);
  void Initialize();
  void Start();
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg);
  void DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr& status_msg);
  void BrakeTimer(const ros::TimerEvent& event);
  void Abort();
};

#endif
