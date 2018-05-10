#ifndef DEPTH_CONTROLLER_H
#define DEPTH_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "std_msgs/Float64.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/SwitchState.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ControlStatus.h"

class DepthController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber depth_sub, cmd_sub, kill_sub, reset_sub;
    ros::Publisher cmd_pub, status_pub;

    control_toolbox::Pid depth_controller_pid;
    std_msgs::Float64 accel;

    riptide_msgs::ControlStatus status_msg;

    //PID
    double depth_error;
    double current_depth;
    double cmd_depth;
    double depth_error_dot;
    double last_error;
    double dt;

    bool pid_depth_init;

    ros::Time sample_start;
    ros::Duration sample_duration;

    void UpdateError();
    void ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg);
    void ResetDepth();

  public:
    DepthController();
    void CommandCB(const riptide_msgs::Depth::ConstPtr &depth);
    void DepthCB(const riptide_msgs::Depth::ConstPtr &cmd);
    void SwitchCB(const riptide_msgs::SwitchState::ConstPtr &state);
 };

 #endif
