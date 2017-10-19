#ifndef DEPTH_CONTROLLER_H
#define DEPTH_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "std_msgs/Float64.h"
#include "riptide_msgs/Depth.h"

class DepthController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber depth_sub;
    ros::Subscriber cmd_sub;
    ros::Publisher cmd_pub;

    control_toolbox::Pid depth_controller_pid;
    std_msgs::Float64 accel;

    //PID
    double depth_error;
    double current_depth;
    double cmd_depth;
    double d_error;
    double last_error;
    double dt;

    bool pid_initialized;

    ros::Time sample_start;
    ros::Duration sample_duration;

    void UpdateError();

  public:
    DepthController();
    void CommandCB(const riptide_msgs::Depth::ConstPtr &depth);
    void DepthCB(const riptide_msgs::Depth::ConstPtr &cmd);
 };

 #endif
