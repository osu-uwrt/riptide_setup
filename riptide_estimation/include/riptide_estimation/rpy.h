#ifndef RPY_H
#define RPY_H

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"

class RPY
{
  private:
    // Comms
    ros::Publisher msg_;
    ros::Subscriber imu_;
    geometry_msgs::Vector3Stamped rpy_msg;

  public:
    RPY(ros::NodeHandle nh);
    void callback(const sensor_msgs::Imu::ConstPtr &imu);
};
#endif
