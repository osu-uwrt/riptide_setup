#ifndef COMMAND_COMBINATOR_H
#define COMMAND_COMBINATOR_H

#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include "riptide_msgs/NetLoad.h"
using namespace std;

class CommandCombinator
{
  private:
    // Comms
    ros::NodeHandle nh;
    ros::Subscriber x_sub, y_sub, z_sub, moment_sub, depth_sub;
    ros::Publisher cmd_pub;

    riptide_msgs::NetLoad cmd_load;
    geometry_msgs::Vector3 force_depth, force;

    double MAX_X_FORCE, MAX_Y_FORCE, MAX_Z_FORCE; // [N]
    double MAX_X_MOMENT, MAX_Y_MOMENT, MAX_Z_MOMENT; // [Nm]

  public:
    CommandCombinator();
    template <typename T>
    void LoadParam(string param, T &var);
    void InitMsgs();
    void XCB(const std_msgs::Float64::ConstPtr &force_x);
    void YCB(const std_msgs::Float64::ConstPtr &force_y);
    void ZCB(const std_msgs::Float64::ConstPtr &force_z);
    void DepthCB(const geometry_msgs::Vector3Stamped::ConstPtr &force_msg);
    void MomentCB(const geometry_msgs::Vector3Stamped::ConstPtr &moment_msg);
    double Constrain(double current, double max);
 };

 #endif
