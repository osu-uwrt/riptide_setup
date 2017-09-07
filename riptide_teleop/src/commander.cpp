#include "riptide_teleop/commander.h"

Accel::Accel()
{
  js = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Accel::joy_callback, this);
  target_odom = nh.advertise<riptide_msgs::OdomWithAccel>("target/position", 1);

  target.header.frame_id = "";
  target.pose.position.x = 0;
  target.pose.position.y = 0;
  target.pose.position.z = 0;
  target.twist.linear.x = 0;
  target.twist.linear.y = 0;
  target.twist.linear.z = 0;
  target.twist.angular.x = 0;
  target.twist.angular.y = 0;
  target.twist.angular.z = 0;
  target.accel.angular.x = 0;
  target.accel.angular.y = 0;
  target.accel.angular.z = 0;
  target.accel.linear.z = 0;
}

void Accel::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  target.accel.linear.x = 0.75 * joy->axes[1];
  target.accel.linear.y = joy->axes[0];
  target.pose.position.z += (joy->axes[14] - joy->axes[15]) / 100;

  double roll = 3.14159 / 4 * joy->axes[2] * -1;
  double pitch = 3.14159 / 4 * joy->axes[3];
  double yaw = 3.14159 * (joy->axes[13] - joy->axes[12]);

  target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  target_odom.publish(target);
}

void Accel::loop()
{
  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
