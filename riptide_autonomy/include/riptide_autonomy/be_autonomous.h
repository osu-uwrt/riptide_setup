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
#include "std_msgs/Int8.h"

#include "riptide_msgs/SwitchState.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ThrustStamped.h"

#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/Pneumatics.h"
#include "riptide_msgs/StatusLight.h"

#include "sensor_msgs/image_encodings.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

#include "riptide_autonomy/tslam.h"
#include "riptide_autonomy/task_casino_gate.h"
#include "riptide_autonomy/task_path_marker.h"
#include "riptide_autonomy/task_dice.h"
#include "riptide_autonomy/task_gold_chip.h"
#include "riptide_autonomy/task_slots.h"
#include "riptide_autonomy/task_roulette.h"
#include "riptide_autonomy/task_gold_chip.h"


using namespace std;
typedef riptide_msgs::Constants rc;

class TSlam;
class CasinoGate;
class PathMarker;
class Dice;
class Slots;
class Roulette;
class GoldChip;

class BeAutonomous
{

public:
  // Subscribers, Publishers, and Timers
  ros::NodeHandle nh;
  ros::Subscriber switch_sub, imu_sub, depth_sub, mission_sub;
  ros::Publisher x_accel_pub, y_accel_pub, z_accel_pub, attitude_pub, depth_pub, alignment_pub, pneumatics_pub;
  ros::Publisher thrust_pub, reset_pub, task_info_pub, state_mission_pub, status_light_pub;
  ros::Timer timer;

  // Mission End/System Check
  int thruster;
  riptide_msgs::ResetControls reset_msg;
  riptide_msgs::ThrustStamped thrust_msg;
  riptide_msgs::AlignmentCommand align_cmd;
  riptide_msgs::AttitudeCommand attitude_cmd;
  riptide_msgs::StatusLight light_msg;

  // Mission Info
  int execute_id, load_id, last_load_id, last_kill_switch_value;
  double load_duration, loader_timer, pre_start_duration, start_timer;
  bool mission_loaded, mission_running, clock_is_ticking;
  ros::Time load_time, pre_start_time;

  // Task Info
  YAML::Node tasks;
  string task_file, task_name, object_name;
  double search_depth, search_accel, brake_duration, detection_duration;
  int num_objects, align_thresh, bbox_thresh, detections_req;
  int competition_id, quadrant, task_id, last_task_id, total_tasks, task_order_index;
  int alignment_plane, frame_width, frame_height, cam_center_x, cam_center_y;
  vector<int> task_order;
  vector<string> object_names;
  bool run_single_task;

  // Vehicle State
  geometry_msgs::Vector3 euler_rpy, linear_accel;
  double depth;

  // Task Parameters
  double depth_thresh, roll_thresh, pitch_thresh, yaw_thresh, error_duration;
  double bbox_surge_duration, bbox_heave_duration;

  // Task Specific Objects
  TSlam* tslam;
  double relative_current_x, relative_current_y;

  CasinoGate* casino_gate;
  PathMarker* path;
  Dice* dice;
  Roulette* roulette;
  Slots* slots;
  GoldChip* gold_chip;

  BeAutonomous();
  template <typename T>
  void LoadParam(string param, T &var);
  void LaunchTSlam();
  void StartTask();
  void EndMission();
  void SendInitMsgs();
  void SendResetMsgs();
  void SystemCheckTimer(const ros::TimerEvent& event);
  void UpdateTaskInfo();
  void SystemCheck(const ros::TimerEvent& event);
  void EndTSlamTimer(const ros::TimerEvent& event);
  void EndTSlam();
  void ResetSwitchPanel();
  void StartMissionCB(const std_msgs::Int8::ConstPtr &msg);
  void SwitchCB(const riptide_msgs::SwitchState::ConstPtr& switch_msg);
  void ImuCB(const riptide_msgs::Imu::ConstPtr & imu_msg);
  void DepthCB(const riptide_msgs::Depth::ConstPtr& depth_msg);
  void ImageCB(const sensor_msgs::Image::ConstPtr &msg);
};

#endif
