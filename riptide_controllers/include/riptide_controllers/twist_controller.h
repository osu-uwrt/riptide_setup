#ifndef DEPTH_CONTROLLER_H
#define DEPTH_CONTROLLER_H
#define MAX_ROLL_TWIST 1
#define MAX_PITCH_TWIST 1
#define MAX_YAW_TWIST 1

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Imu.h"

class TwistController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber cmd_sub;
    ros::Publisher cmd_pub;

    control_toolbox::Pid roll_twist_controller_pid;
    control_toolbox::Pid pitch_twist_controller_pid;
    control_toolbox::Pid yaw_twist_controller_pid;

    geometry_msgs::Vector3 angular_accel_cmd;

    //PID
    double roll_twist_error, pitch_twist_error, yaw_twist_error;
    double roll_twist_error_dot, pitch_twist_error_dot, yaw_twist_error_dot;
    double roll_twist_cmd, pitch_twist_cmd, yaw_twist_cmd;

    geometry_msgs::Vector3 current_twist, last_error;

    bool pid_initialized;

    ros::Time sample_start;
    ros::Duration sample_duration;
    double dt;

    void UpdateError();

  public:
    TwistController();
    void CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd);
    void ImuCB(const riptide_msgs::Imu::ConstPtr &imu);
 };

 #endif