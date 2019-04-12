#ifndef ALIGNMENT_CONTROLLER_H
#define ALIGNMENT_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "riptide_msgs/Object.h"
#include "riptide_msgs/AlignmentCommand.h"
#include "riptide_msgs/TaskInfo.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/DepthCommand.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ControlStatusLinear.h"
#include "riptide_msgs/Constants.h"
using namespace std;
typedef riptide_msgs::Constants rc;

class AlignmentController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber alignment_cmd_sub, object_sub, depth_sub, task_info_sub;
    ros::Publisher x_pub, y_pub, depth_pub, status_pub;
    ros::Timer timer;

    // IIR Filter variables for error_dot
    double PID_IIR_LPF_bandwidth, dt_iir, alpha, imu_filter_rate;

    double max_zero_detect_duration;

    control_toolbox::Pid x_pid, y_pid, z_pid;
    double cmd_heave;
    std_msgs::Float64 cmd_force_x, cmd_force_y;
    riptide_msgs::DepthCommand depth_cmd;

    riptide_msgs::ControlStatusLinear status_msg;
    double current_depth;
    double MAX_X_ERROR, MAX_Y_ERROR, MAX_Z_ERROR, MAX_BBOX_SURGE_ERROR, MAX_BBOX_DEPTH_ERROR;
    
    int alignment_plane, bbox_control, obj_bbox_dim, target_bbox_dim;

    geometry_msgs::Vector3 error, error_dot, last_error_dot, last_error;
    geometry_msgs::Vector3 obj_pos, target_pos;
    double dt;

    ros::Time sample_start;
    ros::Duration sample_duration;

    // Reset and active variables
    //bool pid_alignment_reset, pid_alignment_active;
    //bool pid_surge_reset, pid_sway_reset, pid_heave_reset;
    bool pid_surge_active, pid_sway_active, pid_heave_active, pid_alignment_active;

    void InitMsgs();
    void UpdateError();
    double Constrain(double current, double max);
    double SmoothErrorIIR(double input, double prev);
    void ResetSurge();
    void ResetSway();
    void ResetHeave();

  public:
    AlignmentController();
    template <typename T>
    void LoadParam(string param, T &var);
    void ObjectCB(const riptide_msgs::Object::ConstPtr &obj_msg);
    void DisableControllerTimer(const ros::TimerEvent& event);
    void CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &cmd);
    void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
    void TaskInfoCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg);
    //void ResetCB(const riptide_msgs::ResetControls::ConstPtr &reset_msg);
 };

 #endif
