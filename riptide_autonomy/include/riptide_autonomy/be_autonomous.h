#ifndef BE_AUTONOMOUS_H
#define BE_AUTONOMOUS_H

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include "cmath"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/TaskInfo.h"
#include "riptide_msgs/MissionState.h"

#include "sensor_msgs/image_encodings.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "riptide_msgs/SwitchState.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/AlignmentCommand.h"

#include "riptide_autonomy/tslam.h"
#include "riptide_autonomy/roulette.h"
using namespace std;
typedef riptide_msgs::Constants rc;

class TSlam;
class Roulette;

class BeAutonomous
{

public:
  ros::NodeHandle nh;
  ros::Subscriber switch_sub, imu_sub, depth_sub;
  ros::Publisher linear_accel_pub, attitude_pub, depth_pub, alignment_pub, reset_pub;
  ros::Publisher task_info_pub, state_mission_pub;

  // Task Info
  YAML::Node tasks, task_map;
  string task_file, task_map_file, task_name, object_name;
  double search_depth;
  int competition_id, mission_id, task_id, last_task_id, alignment_plane, num_tasks;
  int frame_width, frame_height;

  // Mission Info
  int execute_id, load_id, last_load_id;
  double load_timer, loader_watchdog;
  bool mission_loaded;

  geometry_msgs::Vector3 euler_rpy, linear_accel;
  double depth;
  vector<darknet_ros_msgs::BoundingBox> task_bboxes;

  // Task Specific Objects
  TSlam* tslam;
  double current_x, current_y, start_x, start_y, eta, time_elapsed;
  ros::Time eta_start, cur_time;

  Roulette* roulette;


  BeAutonomous();
  template <typename T>
  void LoadParam(string param, T &var);
  void Execute();
  void UpdateTaskInfo();
  void ReadMap();
  void CalcETA(double Ax, double dist);
  void SystemCheck();
  void SwitchCB(const riptide_msgs::SwitchState::ConstPtr& switch_msg);
  void ImuCB(const riptide_msgs::Imu::ConstPtr & imu_msg);
  void DepthCB(const riptide_msgs::Depth::ConstPtr& depth_msg);
  void ImageCB(const sensor_msgs::Image::ConstPtr &msg);
  void Loop();
};

#endif
