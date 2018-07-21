#include "riptide_autonomy/casino_gate.h"

#define ALIGN_YZ 0
#define ALIGN_BBOX_WIDTH 1

CasinoGate::CasinoGate(BeAutonomous* master) {
  this->master = master;
  active_subs.clear();
  detections = 0;
  attempts = 0;
  task_completed = false;
  align_id = ALIGN_YZ;
  clock_is_ticking = false;
  gate_heading = 0;
}

void CasinoGate::Start() {
  object_name = (master->color == rc::COLOR_BLACK)?object_names.at(0):object_names.at(1); // Black side if statement true, Red otherwise
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = object_name; // Casino_Gate Black/Red
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)master->frame_width*0.7;
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = (int)(master->frame_height/4);
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("CasinoGate: alignment command published (but disabled)");

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &CasinoGate::IDCasinoGate, this);
  active_subs.push_back(task_bbox_sub);
}

// ID the CasinoGate task
void CasinoGate::IDCasinoGate(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg) {
  // Get number of objects and make sure you have 'x' many within 't' seconds
  // Simply entering this callback signifies the object was detected (unless it was a false-positive)
  detections++;
  if(detections == 1) {
    detect_start = ros::Time::now();
    attempts++;
  }
  else {
    duration = ros::Time::now().toSec() - detect_start.toSec();
  }

  if(duration > master->detection_duration_thresh) {
    if(detections >= master->detections_req) {
      task_bbox_sub.shutdown();
      active_subs.erase(active_subs.end());
      master->tslam->Abort();
      clock_is_ticking = false;
      duration = 0;

      // Activate YZ alignment controllers
      // Set points already specified in initial alignment command
      align_cmd.surge_active = false;
      align_cmd.sway_active = true;
      align_cmd.heave_active = true;
      master->alignment_pub.publish(align_cmd);
      alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &CasinoGate::AlignmentStatusCB, this);
      active_subs.push_back(alignment_status_sub);
      ROS_INFO("CasinoGate: Identified CasinoGate. Now aligning wit YZ controllers");
    }
    else /*if(attempts < 3)*/{
      ROS_INFO("CasinoGate: Attempt %i to ID CasinoGate", attempts);
      ROS_INFO("CasinoGate: %i detections in %f sec", detections, duration);
      ROS_INFO("CasinoGate: Beginning attempt %i", attempts+1);
      detections = 0;
      duration = 0;
    }
    /*else {
      ROS_INFO("CasinoGate: More than 3 attempts used to ID CasinoGate");
      ROS_INFO("CasinoGate: Aborting");
      CasinoGate::Abort();
    }*/
  }
}

// A. Make sure the vehicle is aligned with the YZ controllers
// B. Make sure the vehicle is aligned with the X controller
void CasinoGate::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr& status_msg) {
  if(align_id == ALIGN_YZ) { // Perform (A) - YZ alignment
    if(abs(status_msg->y.error) < master->align_thresh && abs(status_msg->z.error) < master->align_thresh)
  	{
      if(!clock_is_ticking) {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if(duration >= master->error_duration_thresh) { // Roulete should be in the camera center
        duration = 0;
        clock_is_ticking = false;
        //align_id = ALIGN_BBOX_WIDTH; // Verify if bbox alignment will work

        // Activate X alignment controller
        align_cmd.surge_active = true;
        master->alignment_pub.publish(align_cmd);
        ROS_INFO("CasinoGate: Activating X controller based on bbox");
      }
  	}
    else {
      duration = 0;
      clock_is_ticking = false;
    }
  }
  else if(align_id == ALIGN_BBOX_WIDTH) { // Perform (B) - X alignment
    if(abs(status_msg->x.error) < master->bbox_thresh)
  	{
      if(!clock_is_ticking) {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if(duration >= master->bbox_duration_thresh) { // Roulete should be off-center
        // Calculate heading for gate based on quadrant
        gate_heading = master->task_map["task_map"][master->quadrant]["map"][master->task_id]["gate_heading"].as<double>();

        // Publish attitude command
        attitude_cmd.roll_active = true;
        attitude_cmd.pitch_active = true;
        attitude_cmd.yaw_active = true;
        attitude_cmd.euler_rpy.x = 0;
        attitude_cmd.euler_rpy.y = 0;
        attitude_cmd.euler_rpy.z = gate_heading;
        master->attitude_pub.publish(attitude_cmd);
        ROS_INFO("CasinoGate: Published gate heading");

        attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &CasinoGate::AttitudeStatusCB, this);
        active_subs.push_back(attitude_status_sub);
      }
  	}
    else {
      duration = 0;
      clock_is_ticking = false;
    }
  }
}

// Make sure the robot is at the correct heading based on the quadrant
void CasinoGate::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg) {
	// Alignment is good, now verify heading error
	if(abs(status_msg->yaw.error) < master->yaw_thresh)
	{
    if(!clock_is_ticking) {
      acceptable_begin = ros::Time::now();
      clock_is_ticking = true;
    }
    else
      duration = ros::Time::now().toSec() - acceptable_begin.toSec();

    if(duration >= master->error_duration_thresh) {
      // Shutdown alignment callback
      alignment_status_sub.shutdown();
      active_subs.erase(active_subs.begin());
      duration = 0;
      clock_is_ticking = false;

  		// Disable alignment controller so vehicle can move forward
      align_cmd.surge_active = false;
      align_cmd.sway_active = false;
      align_cmd.heave_active = false;
      master->alignment_pub.publish(align_cmd);
      ROS_INFO("CasinoGate: Aligned to CasinoGate with linear and attitude controllers");

      // Publish forward accel and call it a success after a few seconds
      ROS_INFO("CasinoGate: Now going to pass thru gate");
      geometry_msgs::Vector3 linear_accel_cmd;
      linear_accel_cmd.x = master->search_accel;
      linear_accel_cmd.y = 0;
      linear_accel_cmd.z = 0;
      master->linear_accel_pub.publish(linear_accel_cmd);
      master->timer = master->nh.createTimer(ros::Duration(1.2*master->eta), &BeAutonomous::EndTSlamTimer, master, true);
      ROS_INFO("CasinoGate: Aligned to gate, now moving forward. Abort timer initiated. ETA: %f", );
    }
	}
  else {
    duration = 0;
    clock_is_ticking = false;
  }
}

// Shutdown all active subscribers
void CasinoGate::Abort() {
  attempts = 0;
  clock_is_ticking = false;

  if(active_subs.size() > 0) {
    for(int i=0; i<active_subs.size(); i++) {
      active_subs.at(i).shutdown();
    }
    active_subs.clear();
  }
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("CasinoGate: Aborting");

  if(task_completed)
    master->StartTask();
}
