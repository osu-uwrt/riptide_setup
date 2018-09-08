#ifndef COMMAND_COMBINATOR_H
#define COMMAND_COMBINATOR_H

#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
using namespace std;

class CommandCombinator
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber x_sub, y_sub, z_sub, angular_sub, depth_sub;
    ros::Publisher cmd_pub;

    geometry_msgs::Accel cmd_accel, accel;
    geometry_msgs::Vector3 depth_accel;

    double MAX_X_ACCEL, MAX_Y_ACCEL, MAX_Z_ACCEL; // [m/s^2]
    double MAX_ROLL_ACCEL, MAX_PITCH_ACCEL, MAX_YAW_ACCEL; // [rad/s^2]

  public:
    CommandCombinator();
    template <typename T>
    void LoadParam(string param, T &var);
    void InitMsgs();
    void XCB(const std_msgs::Float64::ConstPtr &x_accel);
    void YCB(const std_msgs::Float64::ConstPtr &y_accel);
    void ZCB(const std_msgs::Float64::ConstPtr &z_accel);
    void DepthCB(const geometry_msgs::Vector3::ConstPtr &d_accel);
    void AngularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel);
    double Constrain(double current, double max);
    void Combine();
 };

 #endif
