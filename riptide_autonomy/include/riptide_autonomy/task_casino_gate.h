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
#include "riptide_autonomy/validators.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous;

class CasinoGate
{

private:
  ros::Subscriber task_bbox_sub, alignment_status_sub, attitude_status_sub;
  ros::Subscriber *active_subs[3] = {&task_bbox_sub, &alignment_status_sub, &attitude_status_sub};
  ros::Timer timer;

  darknet_ros_msgs::BoundingBoxes task_bboxes;
  riptide_msgs::AlignmentCommand align_cmd;
  riptide_msgs::AttitudeCommand attitude_cmd;

  double gate_heading, end_pos_offset, pass_thru_duration, id_correct_color_duration;
  int left_color, right_color;
  bool passing_on_left, passing_on_right, passed_thru_gate, braked;
  bool detected_black, detected_red;
  string object_name;
  double gate_zcenter_offset, gate_width;
  double incorrect_gate_ycenter_offset, incorrect_gate_width;

  DetectionValidator *detectionBlackValidator, *detectionRedValidator;
  ErrorValidator *xValidator, *yValidator, *zValidator, *yawValidator;

  // Create instance to master
  BeAutonomous *master;

public:
  CasinoGate(BeAutonomous *master);
  void Initialize();
  void Start();
  void IDCasinoGate(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg);
  void EndTSlamTimer(const ros::TimerEvent &event);
  void IDCasinoGateCorrectly(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg);
  void EndSecondIDGateCB(const ros::TimerEvent &event);
  void PositionAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void BBoxAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg);
  void PassThruTimer(const ros::TimerEvent &event);
  void SetEndPos();
  void Abort();
};

#endif
