#ifndef PS3_CONTROLLER_H
#define PS3_CONTROLLER_H

#include "ros/ros.h"
#include "cmath"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "riptide_teleop/ps3_button_mapping.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/Pneumatics.h"
#include "riptide_msgs/Constants.h"
#include "riptide_msgs/LinearCommand.h"
using namespace std;

class PS3Controller
{
 private:
  ros::NodeHandle nh;
  ros::Publisher depth_pub, x_pub, y_pub, moment_pub, reset_pub;
  ros::Publisher plane_pub, pneumatics_pub, roll_pub, pitch_pub, yaw_pub;
  ros::Subscriber joy_sub, depth_sub, imu_sub;

  geometry_msgs::Vector3 delta_attitude, euler_rpy, cmd_euler_rpy;
  riptide_msgs::LinearCommand cmd_x, cmd_y;
  riptide_msgs::DepthCommand cmd_depth;
  riptide_msgs::ResetControls reset_msg;
  riptide_msgs::AttitudeCommand cmd_attitude;
  riptide_msgs::Pneumatics pneumatics_cmd;
  std_msgs::Int8 plane_msg;
  bool isReset, isR2Init, isL2Init, alignment_plane;
  double rt, current_depth, buoyancy_depth_thresh, delta_depth;

  float axes_rear_R2, axes_rear_L2;

  // Max values, and command rates
  double MAX_X_FORCE, MAX_Y_FORCE;
  double CMD_ROLL_RATE, CMD_PITCH_RATE, CMD_YAW_RATE, CMD_DEPTH_RATE;

  // Multiplication Factors (based on command rates)
  double roll_factor, pitch_factor, yaw_factor, depth_factor, boost;

  void InitMsgs();
  double Constrain(double current, double max);
  void DisableControllers();
  void UpdateCommands();
  void PublishCommands();

 public:
  PS3Controller();
  template <typename T>
  void LoadParam(string param, T &var);
  void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
  void ImuCB(const riptide_msgs::Imu::ConstPtr& imu_msg);
  void JoyCB(const sensor_msgs::Joy::ConstPtr& joy);
  void Loop();
};

#endif