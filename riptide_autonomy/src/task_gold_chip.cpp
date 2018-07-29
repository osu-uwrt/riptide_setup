// Gold chip, what it does:
#include "riptide_autonomy/task_gold_chip.h"

#define AST_CENTER 0
#define AST_BBOX 1

#define BURN_BABY_BURN 0
#define BACK_OFF_MAN 1

GoldChip::GoldChip(BeAutonomous* master) {
  this->master = master;
  GoldChip::Initialize();
}

void GoldChip::Initialize() {
  active_subs.clear();

  mission_state = -1;
  delete chip_detector;
}

void GoldChip::Start() {
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = master->object_names.at(0);
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)(master->frame_height*0.7);
  align_cmd.bbox_control = rc::CONTROL_BBOX_HEIGHT;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("GoldChip: alignment command published (but disabled)");

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &GoldChip::Identify, this);
  active_subs.push_back(task_bbox_sub);
  chip_detector = new DetectionValidator(master->detections_req, master->detection_duration_thresh);
  x_validator = new ErrorValidator(master->align_thresh, master->error_duration_thresh);
  y_validator = new ErrorValidator(master->align_thresh, master->error_duration_thresh);
  bbox_validator = new ErrorValidator(master->bbox_thresh, master->error_duration_thresh);
}

void GoldChip::idToAlignment() {
  task_bbox_sub.shutdown();
  active_subs.erase(active_subs.end());

  align_cmd.surge_active = false;
  align_cmd.sway_active = true;
  align_cmd.heave_active = true;
  alignment_state = AST_CENTER;

  // Take control
  master->tslam->Abort(true);
  master->alignment_pub.publish(align_cmd);
  alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &GoldChip::AlignmentStatusCB, this);
  active_subs.push_back(alignment_status_sub);
}

void GoldChip::Identify(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg) {
  int attempts = chip_detector.GetAttempts();
  if (chip_detector->GetDetections() == 0) {
    ROS_INFO("GoldChip: Beginning chip identificaion. Attempt %d", attempts);
  }

  if (chip_detector->Validate()) {
    ROS_INFO("GoldChip: Detection complete. Identified Gold Chip after after %d attempts. Aligning to Gold Chip.", attempts);
    chip_detector->Reset();
    GoldChip::idToAlignment();
  }
}

void GoldChip::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr& status_msg) {
  if (alignment_state == AST_CENTER) {
    if (x_validator->Validate(status_msg->x.error) && y_validator->Validate(status_msg->y.error)) {
      x_validator->Reset();
      y_validator->Reset();

      // Unsure if this is how we lock in depth, copied from roulette
      align_cmd.heave_active = false;
      align_cmd.surge_active = true;
      master->alignment_pub.publish(align_cmd);
      alignment_state = AST_BBOX;
      ROS_INFO("GoldChip: Aligned to target. Depth locked in. Approaching the Gold Chip.");
    }
  } else if (alignment_state == AST_BBOX) {
    if (bbox_validator->Validate(status_msg->z.error)) {
      ROS_INFO("GoldChip: Gold Chip within reach. Beginning push maneuver.");
      GoldChip::StrikeGold();
    }
  }
}

void GoldChip::StrikeGold() {
  mission_state = BURN_BABY_BURN;
  align_cmd.surge_active = false;
  master->alignment_pub.publish(align_cmd);
}

// Shutdown all active subscribers
void GoldChip::Abort() {
  GoldChip::Initialize();

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
  ROS_INFO("GoldChip: Aborting");
}
