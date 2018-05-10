#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H
#define MAX_ROLL 20
#define MAX_PITCH 20

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/SwitchState.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ControlStatus.h"
#include "riptide_msgs/ControlStatusAngular.h"

class AttitudeController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber imu_sub, cmd_sub, kill_sub, reset_sub;
    ros::Publisher cmd_pub, status_pub;

    control_toolbox::Pid roll_controller_pid;
    control_toolbox::Pid pitch_controller_pid;
    control_toolbox::Pid yaw_controller_pid;

    geometry_msgs::Vector3 ang_accel_cmd;
    riptide_msgs::ControlStatusAngular status_msg;

    //PID
    double roll_error, pitch_error, yaw_error;
    double roll_error_dot, pitch_error_dot, yaw_error_dot;
    double roll_cmd, pitch_cmd, yaw_cmd;

    geometry_msgs::Vector3 current_attitude, last_error;

    bool pid_roll_init, pid_pitch_init, pid_yaw_init;

    /*ros::Time sample_start_roll, sample_start_pitch, sample_start_yaw;
    ros::Duration sample_duration_roll, sample_duration_pitch, sample_duration_yaw;
    double dt_roll, dt_pitch, dt_yaw;*/
    ros::Time sample_start;
    ros::Duration sample_duration;
    double dt;

    void InitPubMsg();
    void UpdateError();
    void ResetController(const riptide_msgs::ResetControls::ConstPtr& reset_msg);
    void ResetRoll();
    void ResetPitch();
    void ResetYaw();

  public:
    AttitudeController();
    void SwitchCB(const riptide_msgs::SwitchState::ConstPtr &state);
    void CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd);
    void ImuCB(const riptide_msgs::Imu::ConstPtr &imu);
 };

 #endif
