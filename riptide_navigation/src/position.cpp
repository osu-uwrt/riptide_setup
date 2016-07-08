#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Accel.h>
#include <riptide_msgs/Depth.h>

control_toolbox::Pid surge, sway, heave;

ros::Subscriber state_sub;
ros::Subscriber target_sub;
ros::Publisher att_pub;

ros::Time then;
ros::Duration dt;

nav_msgs::Odometry att;

struct vector
{
		double x;
		double y;
		double z;
};

vector state, state_dot;
vector target, target_dot;
vector error, error_dot;

void depth_cb(const riptide_msgs::Depth::ConstPtr& new_state)
{
  // Current linear displacement
  state_dot.x = 0;
  state_dot.y = 0;
  state_dot.z = new_state->depth;

  // Current linear velocity
  state_dot.x = 0;
  state_dot.y = 0;
  state_dot.z = 0;
}

void target_cb(const nav_msgs::Odometry::ConstPtr& new_target)
{
  // Desired linear displacement
  target.x = new_target->pose.pose.position.x;
  target.y = new_target->pose.pose.position.y;
  target.z = new_target->pose.pose.position.z;

  // Desired angular velocity
  tf::Vector3 velocity;
  tf::vector3MsgToTF(new_target->twist.twist.linear, velocity);
  target_dot.x = velocity.x();
  target_dot.y = velocity.y();
  target_dot.z = velocity.z();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_controller");
  ros::NodeHandle nh;
  ros::NodeHandle r("~surge");
  ros::NodeHandle p("~sway");
  ros::NodeHandle y("~heave");

  surge.init(r);
  sway.init(p);
  heave.init(y);

  state_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1, &depth_cb);
  target_sub = nh.subscribe<nav_msgs::Odometry>("target/position", 1, &target_cb);
  att_pub = nh.advertise<nav_msgs::Odometry>("target/attitude", 1);

  then = ros::Time::now();
  ros::Rate rate(25.0);

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

    // att.angular.x = surge.computeCommand(error.x, error_dot.x, dt);
    // att.angular.y = sway.computeCommand(error.y, error_dot.y, dt);
    // att.angular.z = heave.computeCommand(error.z, error_dot.z, dt);

    att_pub.publish(att);
    rate.sleep();
  }
}
