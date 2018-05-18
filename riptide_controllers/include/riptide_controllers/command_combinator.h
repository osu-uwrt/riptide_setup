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
    ros::Subscriber depth_sub;

    ros::Publisher cmd_pub;
    geometry_msgs::Accel current_accel;
    geometry_msgs::Vector3 linear_accel, depth_accel;
    void ResetController();

  public:
    CommandCombinator();
    void LinearXCB(const std_msgs::Float64::ConstPtr &accel);
    void LinearYCB(const std_msgs::Float64::ConstPtr &accel);
    void LinearZCB(const std_msgs::Float64::ConstPtr &accel);
    void DepthCB(const geometry_msgs::Vector3::ConstPtr &d_accel);
    void AngularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel);
    void CombineLinear();
    double Constrain(double current, double max);
    void Loop();
 };

 #endif
