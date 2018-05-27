#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_listener.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ControlStatus.h"
#include "riptide_msgs/ControlStatusAngular.h"

class AttitudeController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber imu_sub, cmd_sub, reset_sub;
    ros::Publisher cmd_pub, status_pub;

    control_toolbox::Pid roll_controller_pid;
    control_toolbox::Pid pitch_controller_pid;
    control_toolbox::Pid yaw_controller_pid;

    geometry_msgs::Vector3 ang_accel_cmd;
    riptide_msgs::ControlStatusAngular status_msg;
    double MAX_ROLL_ERROR, MAX_PITCH_ERROR, MAX_YAW_ERROR;

    // IIR Filter variables for D-term
    double PID_IIR_LPF_bandwidth, dt_iir, alpha, imu_filter_rate;

    tf::Matrix3x3 R_b2w, R_w2b;
    tf::Vector3 tf;

    //PID
    double roll_error, pitch_error, yaw_error;
    double roll_error_dot, pitch_error_dot, yaw_error_dot;
    double roll_cmd, pitch_cmd, yaw_cmd, last_roll_cmd, last_pitch_cmd, last_yaw_cmd;

    geometry_msgs::Vector3 current_attitude, last_error, last_error_dot;;

    bool pid_roll_init, pid_pitch_init, pid_yaw_init;

    /*ros::Time sample_start_roll, sample_start_pitch, sample_start_yaw;
    ros::Duration sample_duration_roll, sample_duration_pitch, sample_duration_yaw;
    double dt_roll, dt_pitch, dt_yaw;*/

    ros::Time sample_start;
    ros::Duration sample_duration;
    double dt;

    void InitPubMsg();
    void UpdateError();
    double Constrain(double current, double max);
    double SmoothErrorIIR(double input, double prev);
    void ResetController(const riptide_msgs::ResetControls::ConstPtr& reset_msg);
    void ResetRoll();
    void ResetPitch();
    void ResetYaw();

  public:
    AttitudeController();
    void LoadProperty(std::string name, double &param);
    void CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd);
    void ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg);
    void Loop();
 };

 #endif
