#include "riptide_autonomy/tslam.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "tslam");
  TSlam ts;
  ros::spin();
}

TSlam::TSlam() : nh("tslam") { // NOTE: there is no namespace declared in nh()
	go_sub = nh.subscribe<std_msgs::Int8>("/command/tslam/go", 1, &TSlam::Go, this);
	abort_sub = nh.subscribe<std_msgs::Empty>("/command/tslam/abort", 1, &TSlam::Abort, this);
}

void TSlam::Go(const std_msgs::Int8::ConstPtr& task)
{
	// Calculate heading
	currentTaskHeading = 0;
	ros::Publisher attitude_pub = nh.advertise<geometry_msgs::Vector3>("/command/attitude", 1);
	geometry_msgs::Vector3 msg;
	msg.x = 0;
	msg.y = 0;
	msg.z = currentTaskHeading;
	attitude_pub.publish(msg);
	attitude_pub.shutdown();

	// Watch to see when the controller gets there
	attitude_sub = nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::AttitudeStatusCB, this);
}

void TSlam::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg)
{
	double error = abs(status_msg->yaw.error);

	// Once we are at heading
	if (error < 5)
	{
		attitude_sub.shutdown();

		// Drive forward
		ros::Publisher accel_pub = nh.advertise<geometry_msgs::Vector3>("/command/accel_linear", 1);
		geometry_msgs::Vector3 msg;
		msg.x = 1;
		msg.y = 0;
		msg.z = 0;
		accel_pub.publish(msg);
		accel_pub.shutdown();
	}
}

void TSlam::Abort(const std_msgs::Empty::ConstPtr& data)
{
	// Stop accelerating
	ros::Publisher accel_pub = nh.advertise<geometry_msgs::Vector3>("/command/accel_linear", 1);
	geometry_msgs::Vector3 msg;
	msg.x = 0;
	msg.y = 0;
	msg.z = 0;
	accel_pub.publish(msg);
	accel_pub.shutdown();

	// Unsubscribe
	attitude_sub.shutdown();
}
