#ifndef DICE_H
#define DICE_H

#include "ros/ros.h"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "riptide_msgs/ControlStatusLinear.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "riptide_autonomy/be_autonomous.h"
#include "riptide_msgs/validators.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous;

class Dice
{

private:
  ros::Subscriber task_bbox_sub, alignment_status_sub, attitude_status_sub;
  ros::Subscriber *active_subs[3] = {&task_bbox_sub, &alignment_status_sub, &attitude_status_sub};
  ros::Timer timer;

  darknet_ros_msgs::BoundingBoxes task_bboxes;
  riptide_msgs::AlignmentCommand align_cmd;
  riptide_msgs::AttitudeCommand attitude_cmd;

  double detection_duration_black, detection_duration_red, error_duration, pass_thru_duration;

  bool detected_dice1, detected_dice2, detecte_dice5, detected_dice6;
  int completed[2], dice_map[2][2], 

  DetectionValidator *detection1Validator, *detection2Validator, *detection5Validator, *detection6Validator;
  ErrorValidator *xValidator, *yValidator, *zValidator, *yawValidator;

  // Create instance to master
  BeAutonomous *master;

public:
  DIce(BeAutonomous *master);
  void Initialize();
  void Start();
  void IDDice(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg);
  void Align2FirstDice(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void Align2SecondDice(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg);
  void PassThruTimer(const ros::TimerEvent &event);
  void SetEndPos();
  void Abort();
};

#endif
