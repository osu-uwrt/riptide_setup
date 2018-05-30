#ifndef ALIGNMENT_CONTROLLER_H
#define ALIGNMENT_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "riptide_msgs/TaskAlignment.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ControlStatusLinear.h"
#include "geometry_msgs/Vector3.h"
#include <string>


class AlignmentController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber alignment_sub, command_sub, reset_sub;
    ros::Publisher xy_pub, z_pub, status_pub;

    control_toolbox::Pid x_pid, y_pid, z_pid;
    double heave_cmd;
    geometry_msgs::Vector3 xy_cmd;
    riptide_msgs::DepthCommand depth_cmd;

    riptide_msgs::ControlStatusLinear status_msg;
    double MAX_X_ERROR, MAX_Y_ERROR, MAX_Z_ERROR;

    int alignment_plane, target_bbox_width, task_bbox_width, last_target_bbox_width;

    geometry_msgs::Vector3 error, error_dot, last_error, target, task, last_target;
    double dt;

    bool pid_surge_init, pid_sway_init, pid_heave_init;

    std::string topics[2] = {"/task/gate/alignment", "/task/pole/alignment"};
    int current_task_id;

    ros::Time sample_start;
    ros::Duration sample_duration;

    void InitMsgs();
    void UpdateError();
    void UpdateTaskID(int id);
    double Constrain(double current, double max);
    void ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg);
    void ResetSurge();
    void ResetSway();
    void ResetHeave();

  public:
    AlignmentController();
    void LoadProperty(std::string name, double &param);
    void AlignmentCB(const riptide_msgs::TaskAlignment::ConstPtr &msg);
    void CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &msg);
    void Loop();
 };

 #endif
