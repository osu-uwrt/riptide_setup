#include "riptide_autonomy/roulette.h"

Roulette::Roulette(BeAutonomous* master) {
  this->master = master;
  active_subs.clear();
  num_stored_frames = 10;
  obj_vis_thresh = 0.8;
  obj_vis.clear();
  active = false;
}

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

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Roulette::TaskBBoxCB, this);
  active_subs.push_back(task_bbox_sub);
}

void Roulette::TaskBBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg) {
  // Get number of objects and make sure you have 'x' many within 't' seconds

  for(int i=0; i<task_bboxes.bounding_boxes.size(); i++) {
    //obj_vis =
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


/*void Roulette::Go(const std_msgs::Int8::ConstPtr& task)
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
	attitude_sub = nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &Roulette::AttitudeStatusCB, this);
}*/



/*void Roulette::Abort(const std_msgs::Empty::ConstPtr& data)
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
