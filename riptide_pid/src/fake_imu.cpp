#include "sensor_msgs/Imu.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
ros::init(argc,argv, "fake_imu");

ros::NodeHandle n;

ros::Publisher fake2 = n.advertise<sensor_msgs::Imu>("imu_fake",10);

ros::Rate loop_rate(100);

int count = 0;

while (ros::ok())

{

sensor_msgs::Imu accel;

accel.linear_acceleration.x=count/3.0;
accel.linear_acceleration.y=count/4.0;
accel.linear_acceleration.z=count/10.0;

count = count+1;

if (count > 10)
{
count = 0;
}

fake2.publish(accel);

ros::spinOnce();

loop_rate.sleep();
}

return 0;
}

