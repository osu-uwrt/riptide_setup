#include "riptide_teleop/ps3_control.h"

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
  accel.linear.x = 0.75 * joy->axes[1]; // Left joystick vertical
  accel.linear.y = 0.75 * joy->axes[0];  // Left joystick horizontal
  accel.linear.z = 0.25 * (joy->buttons[11] - joy->buttons[10]); // R1 L1

  accel.angular.x =  1.5 * 3.14159 * joy->axes[2] * -1;// Right joystick horizontal
  accel.angular.y = 1.2 * 3.14159 * joy->axes[3]; // Right joystick vertical
  accel.angular.z = 1.2 * 3.14159 * (joy->buttons[8] - joy->buttons[9]); // R2 L2

  accels.publish(accel);
}

// ADD TIMEOUT FOR ZEROS
void Accel::loop()
{
  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    if (accel.linear.x == 0 && accel.linear.y == 0 && accel.linear.z == 0 && accel.angular.x == 0 && accel.angular.y == 0 && accel.angular.z == 0)
    {
      accel.linear.x = 0;
      accel.linear.y = 0;
      accel.linear.z = 0;

      accel.angular.x =  0;
      accel.angular.y = 0;
      accel.angular.z = 0;

      accels.publish(accel);
    }
    rate.sleep();
  }
}
