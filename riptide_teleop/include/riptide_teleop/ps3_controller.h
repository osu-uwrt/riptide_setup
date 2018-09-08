#ifndef PS3_CONTROLLER_H
#define PS3_CONTROLLER_H

#include "ros/ros.h"
#include "cmath"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_teleop/ps3_button_mapping.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/AttitudeCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/Pneumatics.h"
#include "riptide_msgs/Constants.h"
using namespace std;

class PS3Controller
{
 private:
  ros::NodeHandle nh;
  ros::Publisher attitude_pub, depth_pub, x_accel_pub, y_accel_pub, z_accel_pub, ang_accel_pub, reset_pub;
  ros::Publisher plane_pub, pneumatics_pub;
  ros::Subscriber joy_sub, depth_sub, imu_sub;

  geometry_msgs::Vector3 delta_attitude, euler_rpy, cmd_ang_accel;
  std_msgs::Float64 x_cmd, y_cmd, z_cmd;
  riptide_msgs::DepthCommand cmd_depth;
  riptide_msgs::ResetControls reset_msg;
  riptide_msgs::AttitudeCommand cmd_attitude;
  riptide_msgs::Pneumatics pneumatics_cmd;
  std_msgs::Int8 plane_msg;
  bool isReset, isStarted, isInit, isDepthWorking, isIMUWorking, isR2Init, isL2Init;
  bool isDepthInit, alignment_plane;
  double rt, current_depth, buoyancy_depth_thresh, delta_depth;
  bool publish_pneumatics;

  // Max values, and command rates
  double MAX_ROLL, MAX_PITCH, MAX_DEPTH, MAX_XY_ACCEL, MAX_Z_ACCEL;
  double MAX_ROLL_ACCEL, MAX_PITCH_ACCEL, MAX_YAW_ACCEL;
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
