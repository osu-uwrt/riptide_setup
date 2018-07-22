#include "riptide_autonomy/tslam.h"

#define VALIDATE_PITCH 0
#define VALIDATE_YAW 1

TSlam::TSlam(BeAutonomous* master) {
  this->master = master;
  TSlam::Initialize();
}
void TSlam::Initialize() {
  duration = 0;
  delta_x = 0;
  delta_y = 0;
  angle = 0;
  heading = 0;

  active_subs.clear();
  enroute = false;
  clock_is_ticking = false;
  validate_id = VALIDATE_PITCH;
}

void TSlam::Start() {
  // Calculate heading to point towards next task
  delta_x = master->start_x - master->current_x;
  delta_y = master->start_y - master->current_y;
  ROS_INFO("Cur X: %f", master->current_x);
  ROS_INFO("Cur Y: %f", master->current_y);
  angle = atan2(delta_y, delta_x);
  heading = angle - 90;
  if(heading <= -180)
    heading += 360;

  // Calculate distance and ETA
  distance = sqrt(delta_x*delta_x + delta_y*delta_y);
  master->CalcETA(master->search_accel, distance);
  ROS_INFO("TSlam: eta of %f sec to task %s", master->eta, master->task_name.c_str());

  // Publish attitude command
  attitude_cmd.roll_active = true;
  attitude_cmd.pitch_active = true;
  attitude_cmd.yaw_active = false; // Don't go to heading yet
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;
  attitude_cmd.euler_rpy.z = heading;
  master->attitude_pub.publish(attitude_cmd);
  ROS_INFO("TSlam: Published attitude cmd");

  attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::AttitudeStatusCB, this);
  active_subs.push_back(attitude_status_sub);
  ROS_INFO("TSlam: Checking pitch error");
}

void TSlam::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg) {
  if(validate_id == VALIDATE_PITCH) { // Must first validate pitch so vehicle can submerge properly
  	if(abs(status_msg->pitch.error) < master->pitch_thresh)
  	{
      if(!clock_is_ticking) {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if(duration >= master->error_duration_thresh) {
        attitude_status_sub.shutdown();
        active_subs.erase(active_subs.end());
        duration = 0;
        clock_is_ticking = false;

        // Publish depth command
        depth_cmd.active = true;
        depth_cmd.depth = master->search_depth;
        master->depth_pub.publish(depth_cmd);
        ROS_INFO("TSlam: Pitch good. Published depth cmd");
        depth_status_sub = master->nh.subscribe<riptide_msgs::ControlStatus>("/status/controls/depth", 1, &TSlam::DepthStatusCB, this);
        active_subs.push_back(depth_status_sub);
      }
  	}
    else {
      duration = 0;
      clock_is_ticking = false;
    }
  }
  else if(validate_id == VALIDATE_YAW) { // Validate heading after depth is reached
  	if(abs(status_msg->yaw.error) < master->yaw_thresh)
  	{
      if(!clock_is_ticking) {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if(duration >= master->error_duration_thresh) {
        attitude_status_sub.shutdown();
        active_subs.clear();
        duration = 0;
        clock_is_ticking = false;

    		// Drive forward
    		geometry_msgs::Vector3 msg;
    		msg.x = master->search_accel;
    		msg.y = 0;
    		msg.z = 0;
    		master->linear_accel_pub.publish(msg);
        enroute = true;
        master->eta_start = ros::Time::now();

        double tslam_duration = 1.5*master->eta;
        master->timer = master->nh.createTimer(ros::Duration(tslam_duration), &BeAutonomous::EndTSlamTimer, master, true);
        ROS_INFO("TSlam: Reached heading, now moving forward. Abort timer initiated. ETA: %f", tslam_duration);
      }
  	}
    else {
      duration = 0;
      clock_is_ticking = false;
    }
  }
}

void TSlam::DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr& status_msg) {
  if(abs(status_msg->error) < master->depth_thresh) {
    if(!clock_is_ticking) {
      acceptable_begin = ros::Time::now();
      clock_is_ticking = true;
    }
    else
      duration = ros::Time::now().toSec() - acceptable_begin.toSec();

    if(duration >= master->error_duration_thresh) {
      depth_status_sub.shutdown();
      active_subs.erase(active_subs.end());
      duration = 0;
      clock_is_ticking = false;
      validate_id = VALIDATE_YAW;

      attitude_cmd.yaw_active = true;
      master->attitude_pub.publish(attitude_cmd);
      attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::AttitudeStatusCB, this);
      active_subs.push_back(attitude_status_sub);
      ROS_INFO("TSlam: Reached search depth, now checking heading error");
    }
  }
  else {
    duration = 0;
    clock_is_ticking = false;
  }
}

void TSlam::BrakeTimer(const ros::TimerEvent& event) {
  geometry_msgs::Vector3 msg;
  msg.x = 0;
  msg.y = 0;
  msg.z = 0;
  master->linear_accel_pub.publish(msg);
  ROS_INFO("TSlam: Thruster brake applied");
}

// Shutdown all active subscribers
void TSlam::Abort() {
  TSlam::Initialize();

  if(active_subs.size() > 0) {
    for(int i=0; i<active_subs.size(); i++) {
      active_subs.at(i).shutdown();
    }
    active_subs.clear();
  }
  geometry_msgs::Vector3 msg;
  msg.x = -(master->search_accel);
  msg.y = 0;
  msg.z = 0;
  master->linear_accel_pub.publish(msg);
  timer = master->nh.createTimer(ros::Duration(0.25), &TSlam::BrakeTimer, this, true);
  ROS_INFO("TSlam: Aborting. Braking now.");
}
