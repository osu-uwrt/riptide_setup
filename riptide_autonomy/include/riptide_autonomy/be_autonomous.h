#ifndef BE_AUTONOMOUS
#define BE_AUTONOMOUS_H

#include "ros/ros.h"
#include <yaml-cpp/yaml.h>
#include "cmath"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/SwitchState.h"
#include "riptide_msgs/TaskInfo.h"
#include "riptide_msgs/MissionState.h"
#include "riptide_autonomy/tslam.h"
using namespace std;
typedef riptide_msgs::Constants rc;

class BeAutonomous
{

private:
  ros::NodeHandle nh;
  ros::Subscriber switch_sub;
  ros::Publisher task_info_pub, state_mission_pub;

  // Task Info
  YAML::Node tasks;
  string task_file, task_name;
  int task_id, alignment_plane, num_tasks;

  // Mission Info
  int execute_id, load_id, last_load_id;
  double load_timer, loader_watchdog;
  bool mission_loaded;

  // Task objects
  //TSlam* tslam_ptr;

public:
  BeAutonomous();
  template <typename T>
  void LoadParam(string param, T &var);
  void Execute();
  void UpdateTaskInfo();
  void SystemCheck();
  void SwitchCB(const riptide_msgs::SwitchState::ConstPtr& switch_msg);
  void Loop();
};

#endif
