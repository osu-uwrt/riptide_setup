#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Accel.h"

#define zero

class Accel
{
  private:
    ros::NodeHandle nh;
    ros::Publisher accels;
    ros::Subscriber js;
    geometry_msgs::Accel accel;

  public:
    Accel();
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_accel");
  Accel accel;
  accel.loop();
}

Accel::Accel()
{
  js = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Accel::joy_callback, this);
  accels = nh.advertise<geometry_msgs::Accel>("accel_cmd", 1);
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
  accel.linear.x = 5 * joy->axes[1];
  accel.linear.y = 0;
  accel.linear.z = 5 * (joy->axes[14] - joy->axes[15]);

  accel.angular.x = 5 * 2 * 3.14159 * joy->axes[2] * -1;
  accel.angular.y = 5 * 2 * 3.14159 * joy->axes[3];
  accel.angular.z = 5 * 2 * 3.14159 * (joy->axes[13] - joy->axes[12]);
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
  ros::Rate rate(30);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
