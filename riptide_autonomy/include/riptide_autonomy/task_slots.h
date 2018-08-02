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
#include "riptide_autonomy/validators.h"

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
  ros::Subscriber *active_subs[3] = {&task_bbox_sub, &alignment_status_sub, &attitude_status_sub};
  ros::Timer timer;

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
  torpedoOffset torpedo_offsets[2];
  int bbox_control;
  double big_red_bbox_height;
  double fruit_bbox_height;
  int alignment_state;
  int normal_heading;

  DetectionValidator *fruitValidator, *bigRedValidator;
  ErrorValidator *xValidator, *yValidator, *zValidator, *yawValidator;

  // Reference to master
  BeAutonomous* master;

  void idToAlignment();

public:

  Slots(BeAutonomous* master);
  void Initialize();
  void Start();
  void IDSlots(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void EndTSlamTimer(const ros::TimerEvent &event);
  void IDToAlignment();
  void AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr& status_msg);
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg);
  void ImuCB(const riptide_msgs::ControlStatusAngular::ConstPtr &msg);
  void BackupTimer(const ros::TimerEvent &event);
  void Abort();
};

#endif
