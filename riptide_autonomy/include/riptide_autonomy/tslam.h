#ifndef TSLAM_H
#define TSLAM_H

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/ControlStatusAngular.h"
#include <cmath>
using namespace std;
typedef riptide_msgs::Constants rc;

class TSlam
{

private:
  ros::NodeHandle nh;
  ros::Subscriber attitude_sub, yolo_sub;

  // Task Info
  YAML::Node task_map;
  string task_map_file, task_name;
  int competetion_level, task_id, num_tasks, level, quad, last_task, next_task;

  int currentTaskHeading;

public:
  TSlam(ros::NodeHandle& nh_in);
  void SetQuad(int l, int q);
  void SetTask(int l, int n);
  void Execute();
  //void UpdateTaskInfo();
  //void Go(const std_msgs::Int8::ConstPtr& task);
  void AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& attitude);
  //void Abort(const std_msgs::Empty::ConstPtr& data);
};

#endif
