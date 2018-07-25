#ifndef SLOTS_H
#define SLOTS_H

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

struct torpedoOffset {
  int y;
  int z;
};

class Slots
{

private:
  ros::Subscriber task_bbox_sub, alignment_status_sub, attitude_status_sub;
  vector<ros::Subscriber> active_subs;

  darknet_ros_msgs::BoundingBoxes task_bboxes;
  riptide_msgs::AlignmentCommand align_cmd;
  riptide_msgs::AttitudeCommand attitude_cmd;
  riptide_msgs::Pneumatics pneumatics_cmd;

  // Mission variables
  int mission_state;
  int active_torpedo;
  int torpedo_count;
  int pneumatics_duration;

  // Alignment variables
  torpedoOffset torpedo_offsets[];
  int bbox_control;
  int bbox_dim;
  int alignment_state;
  bool align_timer_started;
  ros::Time align_start;
  ros::Duration aligned_duration;
  double aligned_duration_thresh;

  // Identification variables
  int fruit_detections;
  int big_red_detections;
  ros::Time id_start;
  ros::Duration id_duration;
  int id_attempts;

  // Reference to master
  BeAutonomous* master;

  void idToAlignment();
  void updateAlignTimer(bool stopTimer=false);
  void hitJackpot();

public:

  Slots(BeAutonomous* master);
  void Initialize();
  void Start();
  void Identify(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr& status_msg);
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg);
  void Abort();
};

#endif
