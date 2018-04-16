#ifndef ALIGNMENT_CONTROLLER_H
#define ALIGNMENT_CONTROLLER_H

#include "ros/ros.h"
#include "control_toolbox/pid.h"
#include "std_msgs/Float32.h"
#include "riptide_msgs/ObjectData.h"

class AlignmentController
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber object_sub;

    ros::Publisher cmd_pub;

    control_toolbox::Pid y_pid;
    std_msgs::Float32 accel;

    //PID
    double y_error;
    double d_y_error;
    double last_y_error;
    double dt;

    bool pid_initialized;

    ros::Time sample_start;
    ros::Duration sample_duration;

    void UpdateError();

  public:
    AlignmentController();
    void ObjectCB(const riptide_msgs::ObjectData::ConstPtr &msg);
 };

 #endif
