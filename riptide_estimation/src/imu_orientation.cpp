
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "geometry_msgs/Vector3Stamped.h"

ros::Publisher rpy;
ros::Subscriber imu;
geometry_msgs::Vector3Stamped angles;

void callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter)
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(filter->orientation, quaternion);
  tf::Matrix3x3 rotation = tf::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);
  tf::Matrix3x3 attitude = rotation * tf::Matrix3x3(quaternion);
  attitude.getRPY(angles.vector.x, angles.vector.y, angles.vector.z);
  angles.header.stamp = filter->header.stamp;
  rpy.publish(angles);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orientation");
  ros::NodeHandle nh;
  imu = nh.subscribe<imu_3dm_gx4::FilterOutput>("imu/filter", 1, callback);
  rpy = nh.advertise<geometry_msgs::Vector3Stamped>("imu/angles", 1);
  angles.header.frame_id = "base_link";
  ros::spin();
}
