#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

#include "ros/ros.h"

#include "riptide_msgs/PwmStamped.h"
#include "riptide_msgs/ThrustStamped.h"
#include "riptide_msgs/SwitchState.h"
#include "riptide_msgs/ResetControls.h"
using namespace std;

class PWMController
{
 private:
  ros::NodeHandle nh;
  ros::Subscriber cmd_sub, kill_sub, reset_sub;
  ros::Publisher pwm_pub;
  riptide_msgs::PwmStamped msg;

  float startup_config[2][4], primary_config[2][4]; // Slopes and y-intercepts
  float critical_thrusts[2][2]; // Minimum and startup thrusts
  bool dead, silent, reset_pwm;
  ros::Time last_alive_time;
  ros::Duration alive_timeout;
  bool enable[8];

  void PublishZeroPWM();
  int Thrust2pwm(double raw_force, int thruster);

 public:
  PWMController();
  template <typename T>
  void LoadParam(string param, T &var);
  void ThrustCB(const riptide_msgs::ThrustStamped::ConstPtr &thrust);
  void SwitchCB(const riptide_msgs::SwitchState::ConstPtr &state);
  void ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg);
  void Loop(); //A loop fxn is needed b/c copro can only read msgs so quickly
};

#endif
