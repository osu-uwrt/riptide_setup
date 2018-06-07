#ifndef DEPTH_CONTROLLER_H
#define DEPTH_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_listener.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ControlStatus.h"

class DepthController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber depth_sub, imu_sub, auto_cmd_sub, man_cmd_sub, reset_sub;
    ros::Publisher cmd_pub, status_pub;

    control_toolbox::Pid depth_controller_pid;
    geometry_msgs::Vector3 accel;
    double output, MAX_DEPTH, MAX_DEPTH_ERROR;

    // IIR Filter variables for D-term
    double PID_IIR_LPF_bandwidth, dt_iir, alpha, sensor_rate;

    riptide_msgs::ControlStatus status_msg;

    tf::Matrix3x3 R_b2w, R_w2b;
    tf::Vector3 tf;

    //PID
    double depth_error, depth_error_dot;
    double current_depth;
    double depth_cmd, last_depth_cmd_absolute;
    double last_error, last_error_dot;
    double dt;

    bool pid_depth_init;

    ros::Time sample_start;
    ros::Duration sample_duration;

    void UpdateError();
    double Constrain(double current, double max);
    double SmoothErrorIIR(double input, double prev);
    void ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg);
    void ResetDepth();

  public:
    DepthController();
    void LoadProperty(std::string name, double &param);
    void ManualCommandCB(const riptide_msgs::DepthCommand::ConstPtr &cmd);
    void AutoCommandCB(const riptide_msgs::DepthCommand::ConstPtr &cmd);
    void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
    void ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg);
    void Loop();
 };

 #endif
