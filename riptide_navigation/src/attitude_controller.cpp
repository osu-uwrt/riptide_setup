#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <tf/transform_datatypes.h>
#include <riptide_msgs/OdomWithAccel.h>
#include <sensor_msgs/Imu.h>

control_toolbox::Pid roll, pitch, yaw;

ros::Subscriber state_sub;
ros::Subscriber target_sub;
ros::Publisher command;

geometry_msgs::Accel accel;

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

void target_cb(const riptide_msgs::OdomWithAccel::ConstPtr& new_target)
{
	// Pass linear values through
  accel.linear = new_target->accel.linear;

  // Desired angular displacement
  tf::Quaternion new_orientation;
  tf::quaternionMsgToTF(new_target->pose.orientation, new_orientation);
  tf::Matrix3x3 new_attitude = tf::Matrix3x3(new_orientation);
  new_attitude.getRPY(target.x, target.y, target.z);

  // Desired angular velocity
  target_dot.x = new_target->twist.angular.x;
  target_dot.y = new_target->twist.angular.y;
  target_dot.z = new_target->twist.angular.z;

	// Desired angular acceleration
  feed_fwd.x = new_target->accel.angular.x;
  feed_fwd.y = new_target->accel.angular.y;
  feed_fwd.z = new_target->accel.angular.z;
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
  target_sub = nh.subscribe<riptide_msgs::OdomWithAccel>("target/attitude", 1, &target_cb);
  command = nh.advertise<geometry_msgs::Accel>("command/accel", 1);

	// Angular displacement
	state.x = 0;
	state.y = 0;
	state.z = 0;

	// Angular velocity
	state_dot.x = 0;
	state_dot.y = 0;
	state_dot.z = 0;

	// Angular displacement
	target.x = 0;
	target.y = 0;
	target.z = 0;

	// Angular velocity
	target_dot.x = 0;
	target_dot.y = 0;
	target_dot.z = 0;

	// Angular acceleration
	feed_fwd.x = 0;
	feed_fwd.y = 0;
	feed_fwd.z = 0;

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

    accel.angular.x = feed_fwd.x + roll.computeCommand(error.x, error_dot.x, dt);
    accel.angular.y = feed_fwd.y + pitch.computeCommand(error.y, error_dot.y, dt);
    accel.angular.z = feed_fwd.z + yaw.computeCommand(error.z, error_dot.z, dt);

    command.publish(accel);
    rate.sleep();
  }
}
