#ifndef ALIGNMENT_CONTROLLER_H
#define ALIGNMENT_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "std_msgs/Float32.h"
#include "riptide_msgs/TaskAlignment.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "geometry_msgs/Vector3.h"
#include <string>

class AlignmentController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber alignment_sub;
    ros::Subscriber command_sub;

    ros::Publisher cmd_pub, x_pub, y_pub, z_pub;

    control_toolbox::Pid x_pid, y_pid, z_pid;
    std_msgs::Float32 surge_cmd, sway_cmd, heave_cmd;

    int alignment_plane, target_bbox_width, task_bbox_width;

    geometry_msgs::Vector3 error, error_dot, last_error, target, task;
    double dt;

    bool pid_initialized;

    std::string topics[2] = {"/task/gate/alignment", "/task/pole/alignment"};
    int current_task_id;

    ros::Time sample_start;
    ros::Duration sample_duration;

    void UpdateError();
    void UpdateTaskID(int id);

  public:
    AlignmentController();
    void AlignmentCB(const riptide_msgs::TaskAlignment::ConstPtr &msg);
    void CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &msg);
 };

 #endif
