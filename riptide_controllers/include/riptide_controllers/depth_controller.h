#ifndef DEPTH_CONTROLLER_H
#define DEPTH_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_listener.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/ResetControls.h"
#include "riptide_msgs/ControlStatus.h"

class DepthController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber depth_sub, imu_sub, cmd_sub, reset_sub;
    ros::Publisher cmd_pub, status_pub;

    control_toolbox::Pid depth_controller_pid;
    geometry_msgs::Vector3 accel;
    double output;

    riptide_msgs::ControlStatus status_msg;

    tf::Matrix3x3 R_b2w, R_w2b;
    tf::Vector3 tf;

    //PID
    double depth_error;
    double current_depth;
    double depth_cmd, prev_depth_cmd;
    double depth_error_dot;
    double last_error;
    double dt;

    bool pid_depth_init;

    ros::Time sample_start;
    ros::Duration sample_duration;

    void UpdateError();
    double ConstrainError(double error, double max);
    void ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg);
    void ResetDepth();

  public:
    DepthController();
    void CommandCB(const riptide_msgs::Depth::ConstPtr &cmd);
    void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
    void ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg);
    void Loop();
 };

 #endif
