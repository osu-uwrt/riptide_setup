#include "riptide_teleop/ps3_control.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_accel");
  Accel accel;
  accel.loop();
}

Accel::Accel()
{
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Accel::joy_callback, this);
  angular_pub = nh.advertise<geometry_msgs::Vector3>("command/accel/angular", 1);
  linear_x_pub = nh.advertise<std_msgs::Float64>("command/linear/x", 1);
  linear_y_pub = nh.advertise<std_msgs::Float64>("command/linear/y", 1);
  depth_pub = nh.advertise<riptide_msgs::Depth>("command/depth", 1);
  current_depth_cmd = 0;
}

void Accel::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  linear_x_accel.data = 0.75 * joy->axes[1]; // Left joystick vertical
  linear_y_accel.data = 0.75 * joy->axes[0];  // Left joystick horizontal

  current_depth_cmd = current_depth_cmd + 0.1 * (joy->buttons[11] - joy->buttons[10]); // R1 L1
  if (current_depth_cmd < 0)
    current_depth_cmd = 0;
  depth_cmd.depth = current_depth_cmd;

  angular_accel.x = 1.5 * 3.14159 * joy->axes[2] * -1;// Right joystick horizontal
  angular_accel.y = 1.2 * 3.14159 * joy->axes[3]; // Right joystick vertical
  angular_accel.z = 1.2 * 3.14159 * (joy->buttons[8] - joy->buttons[9]); // R2 L2

  publish_commands();
}

// ADD TIMEOUT FOR ZEROS
void Accel::loop()
{
  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    if (current_depth_cmd == 0 && linear_x_accel.data == 0 && linear_y_accel.data == 0 && angular_accel.x == 0 && angular_accel.y == 0 && angular_accel.z == 0)
    {
      linear_x_accel.data = 0;
      linear_y_accel.data = 0;
      depth_cmd.depth = 0;
      angular_accel.x = 0;
      angular_accel.y = 0;
      angular_accel.z = 0;

      publish_commands();
    }
    rate.sleep();
  }
}

void Accel::publish_commands()
{
    angular_pub.publish(angular_accel);
    linear_x_pub.publish(linear_x_accel);
    linear_y_pub.publish(linear_y_accel);
    depth_pub.publish(depth_cmd);
}
