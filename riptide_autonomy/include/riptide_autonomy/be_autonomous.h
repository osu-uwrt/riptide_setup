#ifndef BE_AUTONOMOUS_H
#define BE_AUTONOMOUS_H

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include "cmath"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/TaskInfo.h"
#include "riptide_msgs/MissionState.h"

#include "riptide_msgs/SwitchState.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/DepthCommand.h"

#include "riptide_autonomy/tslam.h"
using namespace std;
typedef riptide_msgs::Constants rc;

class TSlam;

class BeAutonomous
{

public:
  ros::NodeHandle nh;
  ros::Subscriber switch_sub, imu_sub, depth_sub, task_bbox_sub;
  ros::Publisher linear_accel_pub, attitude_pub, depth_pub;
  ros::Publisher task_info_pub, state_mission_pub;

  // Task Info
  YAML::Node tasks, task_map;
  string task_file, task_map_file, task_name;
  int competition_id, mission_id, task_id, last_task_id, alignment_plane, num_tasks;

  // Mission Info
  int execute_id, load_id, last_load_id;
  double load_timer, loader_watchdog;
  bool mission_loaded;

  geometry_msgs::Vector3 euler_rpy, linear_accel;
  double depth;
  vector<darknet_ros_msgs::BoundingBox> task_bboxes;

  // Task Specific Objects
  TSlam* tslam;
  bool tslam_running;
  double current_x, current_y, start_x, start_y;

  //public:
  BeAutonomous();
  template <typename T>
  void LoadParam(string param, T &var);
  void Execute();
  void UpdateTaskInfo();
  void ReadMap();
  double CalcETA(double dist, double Ax);
  void SystemCheck();
  void SwitchCB(const riptide_msgs::SwitchState::ConstPtr& switch_msg);
  void ImuCB(const riptide_msgs::Imu::ConstPtr & imu_msg);
  void DepthCB(const riptide_msgs::Depth::ConstPtr& depth_msg);
  void TaskBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);
  void Loop();
};

#endif
