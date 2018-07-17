#ifndef ROULETTE_H
#define ROULETTE_H

#include "ros/ros.h"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include "riptide_autonomy/be_autonomous.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous;

class Roulette
{

private:
  ros::Subscriber attitude_status_sub, task_bbox_sub;
  vector<ros::Subscriber> active_subs;

  darknet_ros_msgs::BoundingBoxes task_bboxes;

  double obj_vis_thresh;
  int num_stored_frames;
  vector<int> obj_vis;
  ros::Time acceptable_begin;
  double duration, duration_thresh;

  // Create instance to master
  BeAutonomous* master;

public:
  bool active;

  Roulette(BeAutonomous* master);
  void Execute();
  void TaskBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void Abort();

};

#endif
