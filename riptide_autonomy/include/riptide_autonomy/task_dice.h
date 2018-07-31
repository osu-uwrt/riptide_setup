#ifndef DICE_H
#define DICE_H

#include "ros/ros.h"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatus.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "riptide_msgs/ControlStatusLinear.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "riptide_autonomy/be_autonomous.h"
#include "riptide_autonomy/validators.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

#define DICE1 1
#define DICE2 2
#define DICE5 5
#define DICE6 6
#define DICE1_INDEX 0
#define DICE2_INDEX 1
#define DICE5_INDEX 2
#define DICE6_INDEX 3

class BeAutonomous;

class Dice
{

private:
  ros::Subscriber task_bbox_sub, alignment_status_sub, depth_status_sub;
  ros::Subscriber *active_subs[3] = {&task_bbox_sub, &alignment_status_sub, &depth_status_sub};
  ros::Timer timer;

  darknet_ros_msgs::BoundingBoxes task_bboxes;
  riptide_msgs::AlignmentCommand align_cmd;
  riptide_msgs::AttitudeCommand attitude_cmd;
  riptide_msgs::DepthCommand depth_cmd;

  double bump_duration, backup_duration, move_over_duration;
  double dice_bbox_width, upper_dice_zcenter_offset;
  int num_preferred_objects, num_dice_completed;
  string object_name;
  vector<int> preferred_objects;

  bool detected_dice1, detected_dice2, detected_dice5, detected_dice6;
  bool *detected_dice[4] = {&detected_dice1, &detected_dice2, &detected_dice5, &detected_dice6};
  int completed[2], dice_map[2][2], num_dice_detections, diceID[4] = {DICE1, DICE2, DICE5, DICE6};
  int yCenters[4], zCenters[4], yCenterAvg, zCenterAvg, current_dice, current_index;
  bool bumping, backing_up;

  DetectionValidator *detection1Validator, *detection2Validator, *detection5Validator, *detection6Validator;
  DetectionValidator *detectionValidators[4] = {detection1Validator, detection2Validator, detection5Validator, detection6Validator};
  ErrorValidator *xValidator, *yValidator, *zValidator, *yawValidator, *depthValidator;

  // Create instance to master
  BeAutonomous *master;

public:
  Dice(BeAutonomous *master);
  int CvtNum2Index(int dice);
  int CvtIndex2Num(int index);
  bool IsDiceOnTop(int dice);
  bool IsDiceOnLeft(int dice);
  void UpdateCurrentDice();

  void Initialize();
  void Start();
  void IDDiceTask(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg);
  void EndTSlamTimer(const ros::TimerEvent &event);
  void UpdateDiceYCenter(int* value, int max, int min);
  void UpdateDiceZCenter(int* value, int max, int min);
  void MapDiceField(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg);
  
  void Align2FirstDiceYZ(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void Align2FirstDiceBBox(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void FirstDiceBumpTimer(const ros::TimerEvent &event);
  void DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr &status_msg);
  void MoveOverTimer(const ros::TimerEvent &event);
  void IDSecondDice(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg);
  void Align2SecondDiceYZ(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void Align2SecondDiceBBox(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void SecondDiceBumpTimer(const ros::TimerEvent &event);
  
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg);
  
  void SetEndPos();
  void Abort();
};

#endif
