#include "riptide_estimation/rpy.h"

void RPY::callback(const sensor_msgs::Imu::ConstPtr& imu)
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(imu->orientation, quaternion);
  tf::Matrix3x3 attitude = tf::Matrix3x3(quaternion);
  attitude.getRPY(rpy_msg.vector.x, rpy_msg.vector.y, rpy_msg.vector.z);
  rpy_msg.header.stamp = imu->header.stamp;
  msg_.publish(rpy_msg);
}
RPY::RPY(ros::NodeHandle nh)
{
  imu_ = nh.subscribe<sensor_msgs::Imu>("state/imu", 1, &RPY::callback, this);
  msg_ = nh.advertise<geometry_msgs::Vector3Stamped>("state/rpy", 1);
  rpy_msg.header.frame_id = "base_link";
  ros::spin();
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "orientation");
  ros::NodeHandle nh;
  RPY rpy(nh);
}
