// Slots, what it does:
// 1: Identify either the Fruit or Big Red Slot
// 2: If Big Red is found:
//     2a. Align the active torpedo YZ to Big Red
//     2b. Align to Big Red in X axis
//     2c. Fire active torpedo
//     2d. If torpedoes remain:
//          2da. Toggle the active torpedo, return to 2a
//         Otherwise:
//          2db. END.
//    If Fruits are the only thing found:
//      2e. Case unimplemented. END.
//    If nothing is found:
//      2f. Return to step 1

#include "riptide_autonomy/task_slots.h"

// State number is based on index in the "object names" list
// -1 mission state indicates uninitialized
#define MST_FRUIT 0
#define MST_BIG_RED 1
#define MST_NO_ID 2

#define AST_CENTER 0
#define AST_BBOX 1

#define FRUIT_STR "Slots_Fruit"
#define BIG_RED_STR "Slots_Big_Red"

#define PORT_TORPEDO -1
#define STBD_TORPEDO 1

Slots::Slots(BeAutonomous* master) {
  this->master = master;
  Slots::Initialize();
}

void Slots::Initialize() {
  active_subs.clear();

  mission_state = -1;
  alignment_state = -1;
  active_torpedo = PORT_TORPEDO;
  torpedo_count = 2;
}

void Slots::Start() {
  torpedo_offsets[PORT_TORPEDO].y = master->tasks["tasks"][master->task_id]["torpedo_offset"]["port"]["y"].as<int>();
  torpedo_offsets[PORT_TORPEDO].z = master->tasks["tasks"][master->task_id]["torpedo_offset"]["port"]["z"].as<int>();
  torpedo_offsets[STBD_TORPEDO].y = master->tasks["tasks"][master->task_id]["torpedo_offset"]["stbd"]["y"].as<int>();
  torpedo_offsets[STBD_TORPEDO].z = master->tasks["tasks"][master->task_id]["torpedo_offset"]["stbd"]["z"].as<int>();
  pneumatics_duration = master->tasks["tasks"][master->task_id]["pneumatics_duration"].as<double>();
  bbox_control = master->tasks["tasks"][master->task_id]["bbox_control"].as<int>();
  bbox_dim = master->tasks["tasks"][master->task_id]["bbox_dim"].as<int>();
  aligned_duration_thresh = master->tasks["tasks"][master->task_id]["aligned_duration_thresh"].as<double>();

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
  ROS_INFO("Roulette: alignment command published (but disabled)");

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Slots::Identify, this);
  active_subs.push_back(task_bbox_sub);
}

// Requires mission_state be either MST_FRUIT or MST_BIG_RED
// Only MST_BIG_RED is implemented for now
void Slots::idToAlignment() {
  fruit_detections = 0;
  big_red_detections = 0;
  id_attempts = 0;
  id_duration = ros::Duration(0);
  task_bbox_sub.shutdown();
  active_subs.erase(active_subs.end());

  align_cmd.surge_active = false;
  align_cmd.sway_active = true;
  align_cmd.heave_active = true;
  align_cmd.object_name = master->object_names.at(mission_state);
  align_cmd.bbox_dim = bbox_dim;
  align_cmd.bbox_control = bbox_control;
  if (mission_state == MST_BIG_RED) {
    align_cmd.target_pos.y = torpedo_offsets[active_torpedo].y;
    align_cmd.target_pos.z = torpedo_offsets[active_torpedo].z;
    alignment_state = AST_CENTER;
    // Take control
    master->tslam->Abort(true);
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Slots::AlignmentStatusCB, this);
    active_subs.push_back(alignment_status_sub);
  } else {
    Slots::Abort();
  }
}

void Slots::Identify(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg) {
  // If this is our first detection, start the identificaion timer
  if (fruit_detections + big_red_detections == 0) {
    id_start = ros::Time::now();
    id_attempts++;
    ROS_INFO("Slots: Beginning slots detection. Attempt %d", id_attempts);
  } else {
    id_duration = ros::Time::now() - id_start;
  }

  // Process the bbox_msg and determine what we're seeing
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++) {
    if (bbox_msg->bounding_boxes[i].Class == FRUIT_STR) {
      fruit_detections++;
    } else if (bbox_msg->bounding_boxes[i].Class == BIG_RED_STR) {
      big_red_detections++;
    }
  }

  // Figure out our state based on what we saw in the past X seconds
  // Prioritize identification of Big Red
  if (id_duration.toSec() >= master->detection_duration_thresh) {
    if (big_red_detections >= master->detections_req) {
      mission_state = MST_BIG_RED;
      ROS_INFO("Slots: Detection complete. Identified Big Red after after %d attempts. Aligning to Big Red.", id_attempts);
      Slots::idToAlignment();
    } else if (fruit_detections >= master->detections_req) {
      mission_state = MST_FRUIT;
      ROS_INFO("Slots: Detection complete. Identified Fruit after %d attempts. Aligning to Fruit.", id_attempts);
      Slots::idToAlignment();
    } else {
      mission_state = MST_NO_ID;
      ROS_INFO("Slots: Detection complete. No identification after %d attempts.", id_attempts);
    }
  }
}

void Slots::updateAlignTimer(bool stopTimer) {
  if (!stopTimer) {
    if (!align_timer_started) {
      align_timer_started = true;
      align_start = ros::Time::now();
      ROS_INFO("Slots: Alignment timer started.");
    } else {
      aligned_duration = ros::Time::now() - align_start;
    }
  }
  else {
    aligned_duration = ros::Duration(0);
    align_timer_started = false;
  }
}

void Slots::hitJackpot() {
  if (torpedo_count > 0) {
    pneumatics_cmd.header.stamp = ros::Time::now();
    pneumatics_cmd.torpedo_port = active_torpedo == PORT_TORPEDO;
    pneumatics_cmd.torpedo_port = active_torpedo == STBD_TORPEDO;
    pneumatics_cmd.markerdropper = false;
    pneumatics_cmd.manipulator = false;
    pneumatics_cmd.duration = pneumatics_duration;
    master->pneumatics_pub.publish(pneumatics_cmd);
    ROS_INFO("Slots: Torpedo %d fired. Toggling active. %d torpedoes remaining.", active_torpedo, torpedo_count);
    active_torpedo *= -1; // Toggle active torpedo
  } else {
    ROS_INFO("Slots: CRITICAL! Tried to fire with no torpedoes.");
  }
}

void Slots::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr& status_msg) {
  if (alignment_state == AST_CENTER) {
    if (abs(status_msg->y.error) < master->align_thresh && abs(status_msg->z.error) < master->align_thresh) {
      Slots::updateAlignTimer();
      if (aligned_duration.toSec() >= aligned_duration_thresh) {
        ROS_INFO("Slots: Aligned for %f seconds. Approaching target.", aligned_duration.toSec());
        // Change alignment state, stop timer, activate surge alignment.
        if (mission_state == MST_BIG_RED) {
          alignment_state = AST_BBOX;
          Slots::updateAlignTimer(true);
          align_cmd.surge_active = true;
          master->alignment_pub.publish(align_cmd);
        }
      }
    } else {
      Slots::updateAlignTimer(true);
      ROS_INFO("Slots: Alignment lost. Timer stopped.");
    }
  } else if (alignment_state == AST_BBOX) {
    if (abs(status_msg->x.error) < master->bbox_thresh) {
      Slots::updateAlignTimer();
      if (aligned_duration.toSec() >= aligned_duration_thresh) {
        if (mission_state == MST_BIG_RED) {
          ROS_INFO("Slots: Bounding Box aligned for %f seconds. Attempting to fire.", aligned_duration.toSec());
          Slots::updateAlignTimer(true);
          Slots::hitJackpot();
          // Update target alignment (next torpedo)
          if (torpedo_count > 0) {
            alignment_state = AST_CENTER;
            align_cmd.target_pos.y = torpedo_offsets[active_torpedo].y;
            align_cmd.target_pos.z = torpedo_offsets[active_torpedo].z;
            master->alignment_pub.publish(align_cmd);
          } else {
            ROS_INFO("Slots: Out of torpedoes. Let's go home, boys.");
            Slots::Abort();
          }
        }
      }
    }
  }
}

// Shutdown all active subscribers
void Slots::Abort() {
  Slots::Initialize();

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
  ROS_INFO("Slots: Aborting");
}
