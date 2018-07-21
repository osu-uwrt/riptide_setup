#include "riptide_autonomy/roulette.h"

#define ALIGN_CENTER 0
#define ALIGN_BBOX_WIDTH 1
#define ALIGN_OFFSET 2

Roulette::Roulette(BeAutonomous* master) {
  this->master = master;
  od = new ObjectDescriber(master);
  active_subs.clear();
  detections = 0;
  attempts = 0;
  align_id = ALIGN_CENTER;
  num_markers_dropped = 0;
  drop_duration = 0;
  drop_duration_thresh = 0;
  clock_is_ticking = false;
  drop_clock_is_ticking = false;
}

void Roulette::Start() {
  drop_duration_thresh = master->tasks["tasks"][master->task_id]["drop_duration_thresh"].as<double>();
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = master->object_names.at(0); // Roulette
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)master->frame_height*0.7;
  align_cmd.bbox_control = rc::CONTROL_BBOX_HEIGHT;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Roulette: alignment command published (but disabled)");

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Roulette::IDRoulette, this);
  active_subs.push_back(task_bbox_sub);
}

// ID the roulette task
void Roulette::IDRoulette(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg) {
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

      // Send alignment command to put in center of frame (activate controllers)
      // Set points already specified in initial alignment command
      align_cmd.surge_active = true;
      align_cmd.sway_active = true;
      align_cmd.heave_active = true;
      master->alignment_pub.publish(align_cmd);
      alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Roulette::AlignmentStatusCB, this);
      active_subs.push_back(alignment_status_sub);
      ROS_INFO("Roulette: Identified roulette. Now aligning to center");
    }
    else /*if(attempts < 3)*/{
      ROS_INFO("Roulette: Attempt %i to ID roulette", attempts);
      ROS_INFO("Roulette: %i detections in %f sec", detections, duration);
      ROS_INFO("Roulette: Beginning attempt %i", attempts+1);
      detections = 0;
      duration = 0;
    }/*
    else {
      ROS_INFO("Roulette: More than 3 attempts used to ID roulette");
      ROS_INFO("Roulette: Aborting");
      Roulette::Abort();
    }*/
  }
}

// A. Make sure the vehicle is aligned to the center of the roulette wheel
// B. Make sure the vehicle is aligned to the ofset position so it can drop two markers
void Roulette::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr& status_msg) {
  if(align_id == ALIGN_CENTER) { // Perform (A)
    if(abs(status_msg->x.error) < master->align_thresh && abs(status_msg->y.error) < master->align_thresh)
  	{
      if(!clock_is_ticking) {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if(duration >= master->error_duration_thresh) { // Roulete should be in the camera center
        alignment_status_sub.shutdown();
        active_subs.erase(active_subs.end());
        duration = 0;
        clock_is_ticking = false;

    		// Calculate heading for roulette wheel
        od->GetRouletteHeading(&Roulette::SetMarkerDropHeading, this);
      }
  	}
    else {
      duration = 0;
      clock_is_ticking = false;
    }
  }
  else if(align_id == ALIGN_BBOX_WIDTH) { // Align with bbox
    if(abs(status_msg->z.error) < master->bbox_thresh)
  	{
      if(!clock_is_ticking) {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if(duration >= master->bb_duration_thresh) { // Roulete should be in the camera center
        alignment_status_sub.shutdown();
        active_subs.erase(active_subs.end());
        duration = 0;
        clock_is_ticking = false;

    		// Calculate heading for roulette wheel
        od->GetRouletteHeading(&Roulette::SetMarkerDropHeading, this);
      }
  	}
    else {
      duration = 0;
      clock_is_ticking = false;
    }
  }
  else if(align_id == ALIGN_OFFSET) { // Perform (B)
    if(abs(status_msg->x.error) < master->align_thresh && abs(status_msg->y.error) < master->align_thresh)
  	{
      if(!clock_is_ticking) {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if(duration >= master->error_duration_thresh) { // Roulete should be off-center
        if(num_markers_dropped < 2) {
          pneumatics_cmd.header.stamp = ros::Time::now();
          pneumatics_cmd.torpedo_stbd = false;
          pneumatics_cmd.torpedo_port = false;
          pneumatics_cmd.markerdropper = true;
          pneumatics_cmd.manipulator = false;
          pneumatics_cmd.duration = 300; // [ms]

          if(!drop_clock_is_ticking || drop_duration > drop_duration_thresh) {
            drop_time = ros::Time::now();
            drop_clock_is_ticking = true;
            master->pneumatics_pub.publish(pneumatics_cmd);
            num_markers_dropped++;
          }
          else {
            drop_duration = ros::Time::now().toSec() - drop_time.toSec();
          }
        }
        else {
          pneumatics_cmd.markerdropper = false;;
          master->pneumatics_pub.publish(pneumatics_cmd);
          alignment_status_sub.shutdown();
          active_subs.erase(active_subs.end());
          duration = 0;
          clock_is_ticking = false;
          drop_clock_is_ticking = false;
          ROS_INFO("Roulette is DONE!!!");
          Roulette::Abort();
          master->StartTask();
        }
      }
  	}
    else {
      duration = 0;
      clock_is_ticking = false;
      drop_clock_is_ticking = false;
    }
  }
}

// When the green_heading has been found, set the robot to a heading normal to the green section
void Roulette::SetMarkerDropHeading(double heading) {
  double offset = heading - 90;
  green_heading = master->euler_rpy.z + offset;
  if(green_heading >= 180)
    green_heading -= 360;
  if(green_heading < -180)
    green_heading += 360;

  // Calculate heading to drop markers
  if(offset <= 90)
    marker_drop_heading = master->euler_rpy.z + 90;
  if(offset > 90)
    marker_drop_heading = master->euler_rpy.z - 90;

  if(marker_drop_heading >= 180)
    marker_drop_heading -= 360;
  if(marker_drop_heading < -180)
    marker_drop_heading += 360;

  ROS_INFO("Roulette: Green heading is %f", green_heading);
  ROS_INFO("Roulette: Marker Drop Heading is %f", marker_drop_heading);

  // Publish attitude command
  attitude_cmd.roll_active = true;
  attitude_cmd.pitch_active = true;
  attitude_cmd.yaw_active = true;
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;
  attitude_cmd.euler_rpy.z = marker_drop_heading;
  master->attitude_pub.publish(attitude_cmd);
  ROS_INFO("Roulette: Published marker drop attitude");

  attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &Roulette::AttitudeStatusCB, this);
  active_subs.push_back(attitude_status_sub);
}

// Make sure the robot goes to the marker drop heading
void Roulette::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr& status_msg) {
	// Depth is good, now verify heading error

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

  		// Now align to a bit left of the roulette center
      align_cmd.target_pos.x = 0;
      align_cmd.target_pos.y = -(master->frame_width/6);
      align_cmd.target_pos.z = 0;
      master->alignment_pub.publish(align_cmd);
      align_id = ALIGN_OFFSET;
      alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Roulette::AlignmentStatusCB, this);
      active_subs.push_back(alignment_status_sub);
      ROS_INFO("Roulette: Identified roulette. Now aligning to off-center");
    }
	}
  else {
    duration = 0;
    clock_is_ticking = false;
  }
}

// Shutdown all active subscribers
void Roulette::Abort() {
  attempts = 0;
  num_markers_dropped = 0;
  align_id = ALIGN_CENTER;
  duration = 0;
  drop_duration = 0;
  clock_is_ticking = false;
  drop_clock_is_ticking = false;

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
  ROS_INFO("Roulette: Aborting");
}
