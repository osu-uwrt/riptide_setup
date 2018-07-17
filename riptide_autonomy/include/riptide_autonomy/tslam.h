#ifndef TSLAM_H
#define TSLAM_H

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_autonomy/be_autonomous.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous;

class TSlam
{

private:
  ros::Subscriber attitude_status_sub, yolo_sub;

  int execute_id;
  bool arrived;

  double delta_x, delta_y, angle, heading, distance;

  ros::Time acceptable_begin;
  double duration, duration_thresh, x_accel;

  // Create instance to master
  BeAutonomous* master;

public:
  TSlam(BeAutonomous* master);
  void Execute();
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& attitude);
  //void Abort(const std_msgs::Empty::ConstPtr& data);
};

#endif
