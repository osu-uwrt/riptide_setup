#ifndef COMMAND_COMBINATOR_H
#define COMMAND_COMBINATOR_H

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"

class CommandCombinator
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber linear_x_sub;
    ros::Subscriber linear_y_sub;
    ros::Subscriber linear_z_sub;
    ros::Subscriber angular_sub;

    ros::Publisher cmd_pub;
    geometry_msgs::Accel current_accel;
    void ResetController();

  public:
    CommandCombinator();
    void linearXCB(const std_msgs::Float64::ConstPtr &accel);
    void linearYCB(const std_msgs::Float64::ConstPtr &accel);
    void linearZCB(const std_msgs::Float64::ConstPtr &accel);

    void angularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel);
 };

 #endif
