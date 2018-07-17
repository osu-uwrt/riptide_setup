#include "riptide_autonomy/tslam.h"

TSlam::TSlam(BeAutonomous* master) {
  this->master = master;
  duration_thresh = 3.0;
  x_accel = 1.0;
  active_subs.clear();
  enroute = false;
  depth_stable = false;
}

void TSlam::Execute() {
  // Calculate heading to point towards next task
  delta_x = master->start_x - master->current_x;
  delta_y = master->start_y - master->current_y;
  angle = atan2(delta_y, delta_x);
  heading = angle - 90;
  if(heading <= -180)
    heading += 360;

  // Publish attitude command
  riptide_msgs::AttitudeCommand attitude_cmd;
  attitude_cmd.roll_active = true;
  attitude_cmd.pitch_active = true;
  attitude_cmd.yaw_active = true;
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;
  attitude_cmd.euler_rpy.z = heading;
  master->attitude_pub.publish(attitude_cmd);

  // Publish depth command
  riptide_msgs::DepthCommand depth_cmd;
  depth_cmd.active = true;
  depth_cmd.depth = master->search_depth;
  master->depth_pub.publish(depth_cmd);

  // Calculate distance and ETA
  distance = sqrt(delta_x*delta_x + delta_y*delta_y);
  master->CalcETA(x_accel, distance);

  attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::AttitudeStatusCB, this);
  depth_status_sub = master->nh.subscribe<riptide_msgs::ControlStatus>("/status/controls/depth", 1, &TSlam::DepthStatusCB, this);
  active_subs.push_back(attitude_status_sub);
  active_subs.push_back(depth_status_sub);
}

void TSlam::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg) {
  double yaw_error = abs(status_msg->yaw.error);

	// Once we are at heading
	if(yaw_error < 5 && depth_stable)
	{
    if(duration == 0)
      acceptable_begin = ros::Time::now();
    else
      duration += (ros::Time::now().toSec() - acceptable_begin.toSec());

    if(duration >= duration_thresh) {
      attitude_status_sub.shutdown();
      active_subs.clear();

  		// Drive forward
  		geometry_msgs::Vector3 msg;
  		msg.x = x_accel;
  		msg.y = 0;
  		msg.z = 0;
  		master->linear_accel_pub.publish(msg);
      master->eta_start = ros::Time::now();
      enroute = true;
    }
	}
  else duration = 0;
}

void TSlam::DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr& status_msg) {
  if(!depth_stable && abs(status_msg->error) < 0.1) {
    if(duration == 0)
      acceptable_begin = ros::Time::now();
    else
      duration += (ros::Time::now().toSec() - acceptable_begin.toSec());

    if(duration >= duration_thresh) {
      depth_status_sub.shutdown();
      active_subs.erase(active_subs.begin()+1);
      depth_stable = true;
      duration = 0;
    }
  }
  else duration = 0;
}

// Shutdown all active subscribers
void TSlam::Abort() {
  enroute = false;

  if(active_subs.size() > 0) {
    for(int i=0; i<active_subs.size(); i++) {
      active_subs.at(i).shutdown();
    }
    active_subs.clear();
  }
  geometry_msgs::Vector3 msg;
  msg.x = 0;
  msg.y = 0;
  msg.z = 0;
  master->linear_accel_pub.publish(msg);
}
