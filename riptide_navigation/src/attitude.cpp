#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>
#include <tf/transform_datatypes.h>
#include "imu_3dm_gx4/FilterOutput.h"
#include <sensor_msgs/Imu.h>

control_toolbox::Pid roll, pitch, yaw;
geometry_msgs::Accel cmd;

ros::Time then;
ros::Duration dt;

ros::Subscriber filter_sub;
ros::Subscriber target_sub;
ros::Subscriber feedfwd_sub;
ros::Publisher accel_pub;

double state[6] = {};
double target[6] = {};
double error[6] = {};

void filter_cb(const imu_3dm_gx4::FilterOutput::ConstPtr& new_state)
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(new_state->orientation, quaternion);
  tf::Matrix3x3 attitude = tf::Matrix3x3(quaternion);
  attitude.getRPY(state[0], state[1], state[2]);
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& new_state)
{
  state[3] = new_state->angular_velocity.x;
  state[4] = new_state->angular_velocity.y;
  state[5] = new_state->angular_velocity.z;
}

void target_cb(const nav_msgs::Odometry::ConstPtr& new_target)
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(new_target->pose.pose.orientation, quaternion);
  tf::Matrix3x3 attitude = tf::Matrix3x3(quaternion);
  attitude.getRPY(target[0], target[1], target[2]);
  target[3] = new_target->twist.twist.angular.x;
  target[4] = new_target->twist.twist.angular.y;
  target[5] = new_target->twist.twist.angular.z;
}

void feedfwd_cb(const geometry_msgs::Accel::ConstPtr& feedfwd)
{
  cmd.linear.x = feedfwd->linear.x;
  cmd.linear.y = feedfwd->linear.y;
  cmd.linear.z = feedfwd->linear.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_controller");
  ros::NodeHandle nh;
  ros::NodeHandle r("~roll");
  ros::NodeHandle p("~pitch");
  ros::NodeHandle y("~yaw");

  filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("state/filter", 1, &filter_cb);
  target_sub = nh.subscribe<nav_msgs::Odometry>("target/odom", 1, &target_cb);
  feedfwd_sub = nh.subscribe<geometry_msgs::Accel>("state/imu", 1, &feedfwd_cb);
  accel_pub = nh.advertise<geometry_msgs::Accel>("command/accel", 1);

  roll.init(r);
  pitch.init(p);
  yaw.init(y);

  then = ros::Time::now();
  ros::Rate rate(50.0);

  while(ros::ok())
  {
    ros::spinOnce();

    for(int i = 0; i < 6; i ++)
    {
      error[i] = target[i] - state[i];
    }
    dt = ros::Time::now() - then;

    cmd.angular.x = roll.computeCommand(error[0], error[3], dt);
    cmd.angular.y = pitch.computeCommand(error[1], error[4], dt);
    cmd.angular.z = yaw.computeCommand(error[2], error[5], dt);

    accel_pub.publish(cmd);
    rate.sleep();
  }
}
