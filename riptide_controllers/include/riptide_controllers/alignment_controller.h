#ifndef ALIGNMENT_CONTROLLER_H
#define ALIGNMENT_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "riptide_msgs/Object.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ControlStatusLinear.h"
#include "riptide_msgs/Constants.h"
#include "geometry_msgs/Vector3.h"
using namespace std;

class AlignmentController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber alignment_cmd_sub, object_sub, reset_sub;
    ros::Publisher xy_pub, z_pub, status_pub;

    control_toolbox::Pid x_pid, y_pid, z_pid;
    double heave_cmd;
    geometry_msgs::Vector3 xy_cmd;
    riptide_msgs::DepthCommand depth_cmd;

    riptide_msgs::ControlStatusLinear status_msg;
    double MAX_X_ERROR, MAX_Y_ERROR, MAX_Z_ERROR;

    int alignment_plane, bbox_control, obj_bbox_dim, target_bbox_dim, last_target_bbox_dim;

    geometry_msgs::Vector3 error, error_dot, last_error;
    geometry_msgs::Vector3 obj_pos, target_pos, last_target_pos;
    double dt;

    bool pid_surge_init, pid_sway_init, pid_heave_init;

    ros::Time sample_start;
    ros::Duration sample_duration;

    void InitMsgs();
    void UpdateError();
    double Constrain(double current, double max);
    void ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg);
    void ResetSurge();
    void ResetSway();
    void ResetHeave();

  public:
    AlignmentController();
    template <typename T>
    void LoadParam(string param, T &var);
    void ObjectCB(const riptide_msgs::Object::ConstPtr &obj_msg);
    void CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &cmd);
    void Loop();
 };

 #endif
