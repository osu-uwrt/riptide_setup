#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>
#include <tf/transform_datatypes.h>
#include <imu_3dm_gx4/FilterOutput.h>
#include <sensor_msgs/Imu.h>

control_toolbox::Pid roll, pitch, yaw;

ros::Subscriber state_sub;
ros::Subscriber target_sub;
ros::Publisher accel_pub;

ros::Time then;
ros::Duration dt;

geometry_msgs::Accel cmd;

struct vector
{
		double x;
		double y;
		double z;
};

vector state, state_dot;
vector target, target_dot;
vector error, error_dot;

void state_cb(const sensor_msgs::Imu::ConstPtr& new_state)
{
  // Current orientation
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(new_state->orientation, orientation);
  tf::Matrix3x3 attitude = tf::Matrix3x3(orientation);
  attitude.getRPY(state.x, state.y, state.z);

  // Current angular velocity
  tf::Vector3 velocity;
  tf::vector3MsgToTF(new_state->angular_velocity, velocity);
  state_dot.x = velocity.x();
  state_dot.y = velocity.y();
  state_dot.z = velocity.z();
}

void target_cb(const nav_msgs::Odometry::ConstPtr& new_target)
{
  // Desired orientation
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(new_target->pose.pose.orientation, orientation);
  tf::Matrix3x3 attitude = tf::Matrix3x3(orientation);
  attitude.getRPY(target.x, target.y, target.z);

  // Desired angular velocity
  tf::Vector3 velocity;
  tf::vector3MsgToTF(new_target->twist.twist.angular, velocity);
  target_dot.x = velocity.x();
  target_dot.y = velocity.y();
  target_dot.z = velocity.z();
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
  target_sub = nh.subscribe<nav_msgs::Odometry>("target/attitude", 1, &target_cb);
  accel_pub = nh.advertise<geometry_msgs::Accel>("command/accel", 1);

  then = ros::Time::now();
  ros::Rate rate(50.0);

  while(ros::ok())
  {
    ros::spinOnce();

    error.x = target.x - state.x;
    error.y = target.y - state.y;
    error.z = target.z - state.z;

    error_dot.x = target_dot.x - state_dot.x;
    error_dot.y = target_dot.y - state_dot.y;
    error_dot.z = target_dot.z - state_dot.z;

    dt = ros::Time::now() - then;
    then = ros::Time::now();

    cmd.angular.x = roll.computeCommand(error.x, error_dot.x, dt);
    cmd.angular.y = pitch.computeCommand(error.y, error_dot.y, dt);
    cmd.angular.z = yaw.computeCommand(error.z, error_dot.z, dt);

    accel_pub.publish(cmd);
    rate.sleep();
  }
}
