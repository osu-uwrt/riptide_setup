#include "riptide_teleop/accel_cmd.h"

#undef zero

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_accel");
  Accel accel;
  accel.loop();
}

Accel::Accel()
{
  js = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Accel::joy_callback, this);
  accels = nh.advertise<geometry_msgs::Accel>("command/accel", 1);
}

void Accel::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
#ifdef zero
  accel.linear.x = 0;
  accel.linear.y = 0;
  accel.linear.z = 0;

  accel.angular.x = 0;
  accel.angular.y = 0;
  accel.angular.z = 0;
#else
  accel.linear.x = 0.75 * joy->axes[1];
  accel.linear.y = joy->axes[0];
  accel.linear.z = (joy->axes[14] - joy->axes[15]);

  accel.angular.x = 2.0 * 3.14159 * joy->axes[2] * -1;
  accel.angular.y = 1.2 * 3.14159 * joy->axes[3];
  accel.angular.z = 2.0 * 3.14159 * (joy->axes[13] - joy->axes[12]);
#endif
  // accel.linear.x = 5 * joy->axes[1];
  // accel.linear.y = 0;
  // accel.linear.z = 5 * (joy->axes[14] - joy->axes[15]);
  //
  // accel.angular.x = 5 * 2 * 3.14159 * joy->axes[2] * -1;
  // accel.angular.y = 5 * 2 * 3.14159 * joy->axes[3];
  // accel.angular.z = 5 * 2 * 3.14159 * (joy->axes[13] - joy->axes[12]);

  accels.publish(accel);
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
