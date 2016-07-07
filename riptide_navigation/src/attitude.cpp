#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <imu_3dm_gx4/FilterOutput.h>
#include <sensor_msgs/Imu.h>

control_toolbox::Pid roll, pitch, yaw;

ros::Subscriber state_sub;
ros::Subscriber target_sub;
ros::Publisher accel_pub;

tf::StampedTransform imu2base;

ros::Time then;
ros::Duration dt;

geometry_msgs::Accel cmd;

double state[6] = {};
double target[6] = {};
double error[6] = {};

void state_cb(const sensor_msgs::Imu::ConstPtr& new_state)
{
  // Current orientation
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(new_state->orientation, orientation);
  tf::Matrix3x3 attitude = tf::Matrix3x3(orientation);
  attitude.getRPY(state[0], state[1], state[2]);

  // Current angular velocity
  tf::Vector3 velocity;
  tf::vector3MsgToTF(new_state->angular_velocity, velocity);
  state[3] = velocity.x();
  state[4] = velocity.y();
  state[5] = velocity.z();
}

void target_cb(const nav_msgs::Odometry::ConstPtr& new_target)
{
  // Desired orientation
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(new_target->pose.pose.orientation, orientation);
  tf::Matrix3x3 attitude = tf::Matrix3x3(orientation);
  attitude.getRPY(target[0], target[1], target[2]);

  // Desired angular velocity
  tf::Vector3 velocity;
  tf::vector3MsgToTF(new_target->twist.twist.angular, velocity);
  target[3] = velocity.x();
  target[4] = velocity.y();
  target[5] = velocity.z();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_controller");
  ros::NodeHandle nh;
  ros::NodeHandle r("~roll");
  ros::NodeHandle p("~pitch");
  ros::NodeHandle y("~yaw");

  roll.init(r);
  pitch.init(p);
  yaw.init(y);

  state_sub = nh.subscribe<sensor_msgs::Imu>("state/imu", 1, &state_cb);
  target_sub = nh.subscribe<nav_msgs::Odometry>("target/odom", 1, &target_cb);
  accel_pub = nh.advertise<geometry_msgs::Accel>("command/accel", 1);

  tf::TransformListener listener;
	listener.waitForTransform("/imu", "/base_link", ros::Time(0), ros::Duration(10.0) );
	listener.lookupTransform("/imu", "/base_link", ros::Time(0), imu2base);

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
