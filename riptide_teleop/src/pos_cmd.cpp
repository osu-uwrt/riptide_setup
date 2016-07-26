#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>
#include <tf/transform_datatypes.h>

class Accel
{
  private:
    ros::NodeHandle nh;
    ros::Publisher target_odom;
    ros::Publisher target_accel;
    ros::Subscriber js;
    nav_msgs::Odometry target;
    geometry_msgs::Accel feed_fwd;

  public:
    Accel();
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_commander");
  Accel accel;
  accel.loop();
}

Accel::Accel()
{
  js = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Accel::joy_callback, this);
  target_odom = nh.advertise<nav_msgs::Odometry>("target/position", 1);
  target_accel = nh.advertise<geometry_msgs::Accel>("target/accel", 1);

  target.header.frame_id = "";
  target.pose.pose.position.x = 0;
  target.pose.pose.position.y = 0;
  target.pose.pose.position.z = 0;
  target.twist.twist.linear.x = 0;
  target.twist.twist.linear.y = 0;
  target.twist.twist.linear.z = 0;
  target.twist.twist.angular.x = 0;
  target.twist.twist.angular.y = 0;
  target.twist.twist.angular.z = 0;
  feed_fwd.angular.x = 0;
  feed_fwd.angular.y = 0;
  feed_fwd.angular.z = 0;
}

void Accel::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  feed_fwd.linear.x = 0.75 * joy->axes[1];
  feed_fwd.linear.y = joy->axes[0];
  feed_fwd.linear.z = (joy->axes[14] - joy->axes[15]);

  double roll = 3.14159 / 4 * joy->axes[2] * -1;
  double pitch = 3.14159 / 4 * joy->axes[3];
  double yaw = 3.14159 * (joy->axes[13] - joy->axes[12]);

  target.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  target_odom.publish(target);
  target_accel.publish(feed_fwd);
}

void Accel::loop()
{
  ros::Rate rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
