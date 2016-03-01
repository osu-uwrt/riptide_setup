#include "geometry_msgs/AccelStamped.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
ros::init(argc,argv, "fake_data");

ros::NodeHandle n;

ros::Publisher fake = n.advertise<geometry_msgs::AccelStamped>("accel_set_pt",10);

ros::Rate loop_rate(1);

while (ros::ok())

{

geometry_msgs::AccelStamped accel_set;

  ros::Time time = ros::Time::now();
  accel_set.header.stamp = time;

accel_set.accel.linear.x=5;
accel_set.accel.linear.y=1;
accel_set.accel.linear.z=3;


fake.publish(accel_set);

ros::spinOnce();

loop_rate.sleep();
}

return 0;
}

