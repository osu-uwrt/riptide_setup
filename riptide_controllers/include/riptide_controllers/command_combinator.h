#ifndef COMMAND_COMBINATOR_H
#define COMMAND_COMBINATOR_H

#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
using namespace std;

class CommandCombinator
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber auto_linear_sub, manual_linear_sub;
    ros::Subscriber auto_angular_sub, manual_angular_sub;
    ros::Subscriber depth_sub;
    ros::Publisher cmd_pub;

    geometry_msgs::Accel cmd_accel, auto_accel, manual_accel;
    geometry_msgs::Vector3 depth_accel;

    double MAX_X_ACCEL, MAX_Y_ACCEL, MAX_Z_ACCEL; // [m/s^2]
    double MAX_ROLL_ACCEL, MAX_PITCH_ACCEL, MAX_YAW_ACCEL; // [rad/s^2]

  public:
    CommandCombinator();
    template <typename T>
    void LoadParam(string param, T &var);
    void InitMsgs();
    void AutoLinearCB(const geometry_msgs::Vector3::ConstPtr &lin_accel);
    void ManualLinearCB(const geometry_msgs::Vector3::ConstPtr &lin_accel);
    void DepthCB(const geometry_msgs::Vector3::ConstPtr &d_accel);
    void AutoAngularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel);
    void ManualAngularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel);
    double Constrain(double current, double max);
    void Combine();
    void Loop();
 };

 #endif
