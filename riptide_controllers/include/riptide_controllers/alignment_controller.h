#ifndef ALIGNMENT_CONTROLLER_H
#define ALIGNMENT_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "riptide_msgs/Object.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/Depth.h"
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
    ros::Subscriber alignment_cmd_sub, object_sub, depth_sub, reset_sub;
    ros::Publisher xy_pub, depth_pub, status_pub;

    control_toolbox::Pid x_pid, y_pid, z_pid;
    double heave_cmd;
    geometry_msgs::Vector3 xy_cmd;
    riptide_msgs::DepthCommand depth_cmd;

    riptide_msgs::ControlStatusLinear status_msg;
    double current_depth;
    double MAX_X_ERROR, MAX_Y_ERROR, MAX_Z_ERROR;

    int alignment_plane, bbox_control, obj_bbox_dim, target_bbox_dim;

    geometry_msgs::Vector3 error, error_dot, last_error;
    geometry_msgs::Vector3 obj_pos, target_pos;
    double dt;

    bool pid_alignment_reset, pid_alignment_active;
    bool pid_surge_reset, pid_sway_reset, pid_heave_reset;
    bool pid_surge_active, pid_sway_active, pid_heave_active;
    bool reset_linX_sent, reset_linY_sent, reset_linZ_sent; // Send 0 accel
    bool inactive_linX_sent, inactive_linY_sent, inactive_linZ_sent; // Send 0 accel

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
    void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
 };

 #endif
