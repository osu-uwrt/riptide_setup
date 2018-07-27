#ifndef PATH_MARKER_H
#define PATH_MARKER_H

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
#include "riptide_autonomy/object_describer.h"
#include "riptide_autonomy/validators.h"

#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous;
class ObjectDescriber;

class PathMarker
{

private:
  ros::Subscriber task_bbox_sub, alignment_status_sub, attitude_status_sub;
  ros::Subscriber *active_subs[3] = {&task_bbox_sub, &alignment_status_sub, &attitude_status_sub};
  ros::Timer timer;

  enum direction
  {
    right,
    left
  };
  direction pathDirection;

  riptide_msgs::AlignmentCommand align_cmd;
  riptide_msgs::AttitudeCommand attitude_cmd;
  ros::Time detect_start;

  int detections, attempts;
  double path_heading;
  double heading_average = 360, y_average = 100, x_average = 100;

  // Create instance to master
  BeAutonomous *master;
  ObjectDescriber *od;

  DetectionValidator *detectionValidator;
  ErrorValidator *yawValidator, *xValidator, *yValidator;

public:
  PathMarker(BeAutonomous *master);
  void Initialize();
  void Start();
  void IDPathMarker(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg);
  void GotHeading(double heading);
  void FirstAttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg);
  void AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg);
  void SecondAttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg);
  void Success(const ros::TimerEvent &event);
  void Abort();
};

#endif
