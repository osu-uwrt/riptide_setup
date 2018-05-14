#ifndef ALIGNMENT_CONTROLLER_H
#define ALIGNMENT_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "riptide_msgs/TaskAlignment.h"

class AlignmentController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber task_sub;

    ros::Subscriber cmd_sub;
    ros::Publisher cmd_pub;

    control_toolbox::Pid y_pid;
    std_msgs::Float32 accel;

    //PID
    double y_error;
    double x_error;
    double d_y_error;
    double d_x_error;
    double last_y_error;
    double last_x_error;
    double dt;

    double cmd_width

    bool pid_initialized

    ros::Time sample_start;
    ros::Duration sample_duration;

    void UpdateError();

  public:
    AlignmentController();
    void TaskAlignmentCB(const riptide_msgs::TaskAlignment::ConstPtr &msg);
    void CommandAlignmentCB(const riptide_msgs::CommandAlignment::ConstPtr &msg);
 };

 #endif
