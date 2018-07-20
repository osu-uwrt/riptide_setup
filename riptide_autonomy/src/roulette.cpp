#include "riptide_autonomy/roulette.h"



Roulette::Roulette(BeAutonomous* master) {
  this->master = master;
  active_subs.clear();
  detections = 0;
  active = false;
}

/*void Roulette::GotHeading(double a) {
  ROS_INFO("Heading done");
}*/

void Roulette::Execute() {
  active = true;
  riptide_msgs::AlignmentCommand align_cmd;
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = "Roulette";
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)master->frame_width*0.8;
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  master->alignment_pub.publish(align_cmd);

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Roulette::LocateRoulette, this);
  active_subs.push_back(task_bbox_sub);
}

void Roulette::LocateRoulette(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg) {
  // Get number of objects and make sure you have 'x' many within 't' seconds
  // Simply entering this callback signifies the object was detected
  detections++;
  if(detections == 1) {
    detect_start = ros::Time::now();
  }
  else {
    duration = ros::Time::now().toSec() - detect_start.toSec();
  }


  if(duration > master->detection_duration_thresh) {
    if(detections >= master->detections_req) {

    }
    else {
      detections = 0;
    }
  }

}

// Shutdown all active subscribers
void Roulette::Abort() {
  active = false;

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
