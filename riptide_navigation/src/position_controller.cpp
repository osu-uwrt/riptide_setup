#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <tf/transform_datatypes.h>
#include <riptide_msgs/OdomWithAccel.h>
#include <riptide_msgs/Depth.h>

control_toolbox::Pid surge, sway, heave;

ros::Subscriber state_sub;
ros::Subscriber target_sub;
ros::Publisher attitude_pub;

riptide_msgs::OdomWithAccel attitude;

struct vector
{
    double x;
    double y;
    double z;
};

vector state, state_dot;
vector target, target_dot;
vector error, error_dot;
vector feed_fwd;

ros::Time then;
ros::Duration dt;

void depth_cb(const riptide_msgs::Depth::ConstPtr& new_state)
{
	// Update current depth
  state.z = (-1 * new_state->depth) - 0.25;
}

void target_cb(const riptide_msgs::OdomWithAccel::ConstPtr& new_target)
{
  // Pass angular values through
  attitude.pose.position = new_target->pose.position;
  attitude.twist.angular = new_target->twist.angular;
  attitude.accel.angular = new_target->accel.angular;

  // Desired linear displacement
  target.x = new_target->pose.position.x;
  target.y = new_target->pose.position.y;
  target.z = new_target->pose.position.z;

  // Desired linear velocity
	target_dot.x = new_target->twist.linear.x;
	target_dot.y = new_target->twist.linear.y;
	target_dot.z = new_target->twist.linear.z;

  // Desired linear acceleration
	feed_fwd.x = new_target->accel.linear.x;
	feed_fwd.y = new_target->accel.linear.y;
	feed_fwd.z = new_target->accel.linear.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_controller");
  ros::NodeHandle nh;
  ros::NodeHandle sg("~surge");
  ros::NodeHandle sy("~sway");
  ros::NodeHandle hv("~heave");

  surge.init(sg);
  sway.init(sy);
  heave.init(hv);

  state_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1, &depth_cb);
  target_sub = nh.subscribe<riptide_msgs::OdomWithAccel>("target/position", 1, &target_cb);
  attitude_pub = nh.advertise<riptide_msgs::OdomWithAccel>("target/attitude", 1);

	// Linear displacement
	state.x = 0;
	state.y = 0;
	state.z = 0;

	// Linear velocity
	state_dot.x = 0;
	state_dot.y = 0;
	state_dot.z = 0;

	// Linear displacement
	target.x = 0;
	target.y = 0;
	target.z = 0;

	// Linear velocity
	target_dot.x = 0;
	target_dot.y = 0;
	target_dot.z = 0;

	// Linear acceleration
	feed_fwd.x = 0;
	feed_fwd.y = 0;
	feed_fwd.z = 0;

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

    attitude.accel.linear.x = feed_fwd.x + surge.computeCommand(error.x, error_dot.x, dt);
    attitude.accel.linear.y = feed_fwd.y + sway.computeCommand(error.y, error_dot.y, dt);
    attitude.accel.linear.z = feed_fwd.z + heave.computeCommand(error.z, error_dot.z, dt);

    attitude_pub.publish(attitude);
    rate.sleep();
  }
}
