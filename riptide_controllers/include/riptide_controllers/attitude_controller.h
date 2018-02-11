#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H
#define MAX_ROLL 20
#define MAX_PITCH 20

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Imu.h"

class AttitudeController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    ros::Subscriber cmd_sub;
    ros::Publisher cmd_pub;

    control_toolbox::Pid roll_controller_pid;
    control_toolbox::Pid pitch_controller_pid;
    control_toolbox::Pid yaw_controller_pid;

    geometry_msgs::Vector3 accel_cmd;

    //PID
    double roll_error, pitch_error, yaw_error;
    double roll_error_dot, pitch_error_dot, yaw_error_dot;
    double roll_cmd, pitch_cmd, yaw_cmd;

    geometry_msgs::Vector3 current_attitude, last_error;

    bool pid_initialized;

    ros::Time sample_start;
    ros::Duration sample_duration;
    double dt;

    void UpdateError();

  public:
    AttitudeController();
    void CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd);
    void ImuCB(const riptide_msgs::Imu::ConstPtr &imu);
 };

 #endif
