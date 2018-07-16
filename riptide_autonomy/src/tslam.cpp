#include "riptide_autonomy/tslam.h"

int main() {

}

TSlam::TSlam(ros::NodeHandle& nh_in) {
  //nh(nh_in);
  //nh = nh_in;
  //ros::Publisher accel_pub = nh_in.advertise<geometry_msgs::Vector3>("/command/accel_linear", 1);
  level = rc::ROUND_SEMIS;
  quad = 0;
  last_task = -1;
  next_task = rc::TASK_CASINO_GATE;

  //go_sub = nh.subscribe<std_msgs::Int8>("/command/tslam/go", 1, &TSlam::Go, this);
	//abort_sub = nh.subscribe<std_msgs::Empty>("/command/tslam/abort", 1, &TSlam::Abort, this);

  competetion_level = level;

  // Just initialize task_map_file so it will compile
  task_map_file = "../osu-uwrt/riptide_software/src/riptide_autonomy/cfg/task_map_semis.yaml";
  if(competetion_level == rc::ROUND_SEMIS)
    task_map_file = "../osu-uwrt/riptide_software/src/riptide_autonomy/cfg/task_map_semis.yaml";
  else
    task_map_file = "../osu-uwrt/riptide_software/src/riptide_autonomy/cfg/task_map_finals.yaml";

  task_id = rc::TASK_CASINO_GATE;
  task_map = YAML::LoadFile(task_map_file);

  // Verify number of objects and thresholds match
  num_tasks = (int)task_map["task_map"]["map"].size();
}

void TSlam::SetQuad(int l, int q) {
  level = l;
  quad = q;
}

void TSlam::SetTask(int l, int n) {
  last_task = l;
  next_task = n;
}

void TSlam::Execute() {

}

/*void TSlam::UpdateTaskInfo() {
  task_name = tasks["tasks"][task_id]["name"].as<string>();
  num_objects = (int)tasks["tasks"][task_id]["objects"].size();

  alignment_plane = tasks["tasks"][task_id]["plane"].as<int>();
  if(alignment_plane != rc::PLANE_YZ && alignment_plane != rc::PLANE_XY)
    alignment_plane = rc::PLANE_YZ; // Default to YZ-plane (fwd cam)

  object_names.clear();
  thresholds.clear();
  for(int i=0; i < num_objects; i++) {
    object_names.push_back(tasks["tasks"][task_id]["objects"][i].as<string>());
    thresholds.push_back(tasks["tasks"][task_id]["thresholds"][i].as<double>());
  }
}*/

/*void TSlam::Go(const std_msgs::Int8::ConstPtr& task)
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
}*/

void TSlam::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg)
{
	double error = abs(status_msg->yaw.error);

	// Once we are at heading
	if (error < 5)
	{
		attitude_sub.shutdown();

		// Drive forward
		//ros::Publisher accel_pub = nh->advertise<geometry_msgs::Vector3>("/command/accel_linear", 1);
		geometry_msgs::Vector3 msg;
		msg.x = 1;
		msg.y = 0;
		msg.z = 0;
		//accel_pub.publish(msg);
		//accel_pub.shutdown();
	}
}

/*void TSlam::Abort(const std_msgs::Empty::ConstPtr& data)
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
}*/
