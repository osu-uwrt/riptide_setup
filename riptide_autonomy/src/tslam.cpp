#include "riptide_autonomy/tslam.h"

TSlam::TSlam(BeAutonomous* master) {
  this->master = master;
  duration_thresh = 3.0;
  x_accel = 1.0;
  //go_sub = nh.subscribe<std_msgs::Int8>("/command/tslam/go", 1, &TSlam::Go, this);
	//abort_sub = nh.subscribe<std_msgs::Empty>("/command/tslam/abort", 1, &TSlam::Abort, this);
}

void TSlam::Execute() {
  // Calculate heading to point towards next task
  delta_x = master->start_x - master->current_x;
  delta_y = master->start_y - master->current_y;
  angle = atan2(delta_y, delta_x);
  heading = angle - 90;
  if(heading <= -180)
    heading += 360;

  riptide_msgs::AttitudeCommand attitude_cmd;
  attitude_cmd.roll_active = true;
  attitude_cmd.pitch_active = true;
  attitude_cmd.yaw_active = true;
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;
  attitude_cmd.euler_rpy.z = 0;
  master->attitude_pub.publish(attitude_cmd);

  // Calculate distance and estimated ETA
  distance = sqrt(delta_x*delta_x + delta_y*delta_y);

  attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::AttitudeStatusCB, this);
}

void TSlam::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg) {
  if(master->tslam_running) {
    double error = abs(status_msg->yaw.error);

  	// Once we are at heading
  	if (error < 5)
  	{
      if(duration == 0)
        acceptable_begin = ros::Time::now();
      else
        duration += (ros::Time::now().toSec() - acceptable_begin.toSec());

      if(duration >= duration_thresh) {
        attitude_status_sub.shutdown();

    		// Drive forward
    		geometry_msgs::Vector3 msg;
    		msg.x = x_accel;
    		msg.y = 0;
    		msg.z = 0;
    		master->linear_accel_pub.publish(msg);
      }
  	}
    else {
      duration = 0;
    }
  }
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
