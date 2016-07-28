#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"

ros::Publisher msg_;
ros::Subscriber imu_;
geometry_msgs::Vector3Stamped rpy;

void callback(const sensor_msgs::Imu::ConstPtr& imu)
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(imu->orientation, quaternion);
  tf::Matrix3x3 attitude = tf::Matrix3x3(quaternion);
  attitude.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
  rpy.header.stamp = imu->header.stamp;
  msg_.publish(rpy);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orientation");
  ros::NodeHandle nh;
  imu_ = nh.subscribe<sensor_msgs::Imu>("state/imu", 1, callback);
  msg_ = nh.advertise<geometry_msgs::Vector3Stamped>("state/rpy", 1);
  rpy.header.frame_id = "base_link";
  ros::spin();
}
