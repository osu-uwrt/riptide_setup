#ifndef CASINO_GATE_H
#define CASINO_GATE_H

#include "ros/ros.h"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "riptide_msgs/ControlStatusLinear.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/Pneumatics.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "riptide_autonomy/be_autonomous.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous;

class CasinoGate
{

private:
  ros::Subscriber task_bbox_sub, alignment_status_sub, attitude_status_sub;
  vector<ros::Subscriber> active_subs;
  ros::Timer timer;

  darknet_ros_msgs::BoundingBoxes task_bboxes;
  riptide_msgs::AlignmentCommand align_cmd;
  riptide_msgs::AttitudeCommand attitude_cmd;

  double duration, gate_heading, pass_thru_duration;
  int detections, attempts, align_id;
  ros::Time acceptable_begin;
  ros::Time detect_start;
  bool clock_is_ticking;
  string object_name;

  // Create instance to master
  BeAutonomous* master;
  bool passed_thru_gate, braked;

public:

  CasinoGate(BeAutonomous* master);
  void Start();
  void IDCasinoGate(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr& status_msg);
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg);
  void PassThruTimer(const ros::TimerEvent& event);
  void Abort();
};

#endif
