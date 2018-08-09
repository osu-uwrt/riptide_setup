#ifndef ROULETTE_H
#define ROULETTE_H

#include "ros/ros.h"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "riptide_msgs/ControlStatusLinear.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/Pneumatics.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "riptide_autonomy/be_autonomous.h"
#include "riptide_autonomy/object_describer.h"
#include "riptide_autonomy/validators.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous;
class ObjectDescriber;

class Roulette
{

private:
  ros::Subscriber task_bbox_sub, alignment_status_sub, attitude_status_sub, depth_status_sub;
  ros::Subscriber *active_subs[4] = {&task_bbox_sub, &alignment_status_sub, &attitude_status_sub, &depth_status_sub};
  ros::Timer timer;

  darknet_ros_msgs::BoundingBoxes task_bboxes;
  riptide_msgs::AlignmentCommand align_cmd;
  riptide_msgs::AttitudeCommand attitude_cmd;
  riptide_msgs::Pneumatics pneumatics_cmd;
  riptide_msgs::DepthCommand depth_cmd;

  // Alignment variables
  bool got_heading;
  double green_heading, marker_drop_heading, marker_drop_depth, timeout_duration;
  int num_markers_dropped;

  DetectionValidator *detectionValidator;
  ErrorValidator *xValidator, *yValidator, *zValidator, *yawValidator, *depthValidator;

  // Create instance to master
  BeAutonomous* master;
  ObjectDescriber* od;

public:

  Roulette(BeAutonomous* master);
  void Initialize();
  void Start();
  void IDRoulette(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void EndTSlamTimer(const ros::TimerEvent &event);
  void CenterAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr& status_msg);
  void DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr& status_msg);
  void SetMarkerDropHeading(double heading);
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg);
  void TimeOutTimer(const ros::TimerEvent &event);
  void Abort();
};

#endif
