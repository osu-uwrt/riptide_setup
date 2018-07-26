#ifndef TSLAM_H
#define TSLAM_H

#include "ros/ros.h"
#include <vector>
#include <yaml-cpp/yaml.h>
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
  ros::Subscriber active_subs[2] = {attitude_status_sub, depth_status_sub};
  ros::Timer timer;

  riptide_msgs::AttitudeCommand attitude_cmd;
  riptide_msgs::DepthCommand depth_cmd;

  // Task Info
  string task_map_file;

  double current_x, current_y, start_x, start_y;
  double eta, x_vel;

  double delta_x, delta_y, angle, search_heading, distance;
  ros::Time error_check_start;
  double error_duration;
  bool clock_is_ticking;
  int validate_id;

  // Create instance to master
  BeAutonomous* master;

public:
  YAML::Node task_map;
  int quadrant;

  TSlam(BeAutonomous* master);
  void Initialize();
  void ReadMap();
  void CalcETA(double Ax, double dist);
  double KeepHeadingInRange(double input);
  void Start();
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg);
  void DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr& status_msg);
  void AbortTSlamTimer(const ros::TimerEvent& event);
  void BrakeTimer(const ros::TimerEvent& event);
  void Abort(bool apply_brake);
};

#endif
