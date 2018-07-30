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

#define PORT_TORPEDO 0
#define STBD_TORPEDO 1

Slots::Slots(BeAutonomous *master)
{
  this->master = master;
  Slots::Initialize();
}

void Slots::Initialize()
{
  mission_state = -1;
  alignment_state = -1;
  active_torpedo = PORT_TORPEDO;
  torpedo_count = 2;

  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void Slots::Start()
{
  torpedo_offsets[PORT_TORPEDO].y = master->tasks["tasks"][master->task_id]["torpedo_offset"]["port"]["y"].as<int>();
  torpedo_offsets[PORT_TORPEDO].z = master->tasks["tasks"][master->task_id]["torpedo_offset"]["port"]["z"].as<int>();
  torpedo_offsets[STBD_TORPEDO].y = master->tasks["tasks"][master->task_id]["torpedo_offset"]["stbd"]["y"].as<int>();
  torpedo_offsets[STBD_TORPEDO].z = master->tasks["tasks"][master->task_id]["torpedo_offset"]["stbd"]["z"].as<int>();
  pneumatics_duration = master->tasks["tasks"][master->task_id]["pneumatics_duration"].as<double>();
  big_red_bbox_height = master->tasks["tasks"][master->task_id]["big_red_bbox_height"].as<double>();
  fruit_bbox_height = master->tasks["tasks"][master->task_id]["fruit_bbox_height"].as<double>();


  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = master->object_names.at(0);
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)(master->frame_height * big_red_bbox_height);
  align_cmd.bbox_control = rc::CONTROL_BBOX_HEIGHT;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Slots: alignment command published (but disabled)");

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Slots::Identify, this);

  fruitValidator = new DetectionValidator(master->detections_req, master->detection_duration);
  bigRedValidator = new DetectionValidator(master->detections_req, master->detection_duration);
  xValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  yValidator = new ErrorValidator(master->bbox_thresh, master->bbox_surge_duration);
  zValidator = new ErrorValidator(master->align_thresh, master->error_duration);

  fruitValidator->Reset();
  bigRedValidator->Reset();
  xValidator->Reset();
  yValidator->Reset();
  zValidator->Reset();
}

// Requires mission_state be either MST_FRUIT or MST_BIG_RED
// Only MST_BIG_RED is implemented for now

void Slots::Identify(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Process the bbox_msg and determine what we're seeing
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
  {
    if (bbox_msg->bounding_boxes[i].Class == FRUIT_STR)
      fruitValidator->Validate();
    else if (bbox_msg->bounding_boxes[i].Class == BIG_RED_STR)
      bigRedValidator->Validate();
  }

  if (bigRedValidator->IsValid())
  {
    mission_state = MST_BIG_RED;
    ROS_INFO("Slots: Detection complete. Identified Big Red. Aligning to Big Red.");
    Slots::idToAlignment();
  }
  else if (fruitValidator->IsValid())
  {
    mission_state = MST_FRUIT;
    ROS_INFO("Slots: Detection complete. Identified Fruit. Aligning to Fruit.");
    Slots::idToAlignment();
  }
}

void Slots::idToAlignment()
{
  task_bbox_sub.shutdown();
  bigRedValidator->Reset();
  fruitValidator->Reset();

  align_cmd.surge_active = false;
  align_cmd.sway_active = true;
  align_cmd.heave_active = true;
  align_cmd.object_name = master->object_names.at(mission_state);

  if (mission_state == MST_BIG_RED)
  {
    align_cmd.bbox_dim = big_red_bbox_height;
    align_cmd.target_pos.y = torpedo_offsets[active_torpedo].y;
    align_cmd.target_pos.z = torpedo_offsets[active_torpedo].z;
    ROS_INFO("Target y: %f", align_cmd.target_pos.y);
    ROS_INFO("Target z: %f", align_cmd.target_pos.z);
    alignment_state = AST_CENTER;
    // Take control
    master->tslam->Abort(false);
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Slots::AlignmentStatusCB, this);
  }
  else
  {

    // TODO: Write fruit code
    Slots::Abort();
  }
}

void Slots::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (alignment_state == AST_CENTER)
  {
    if (yValidator->Validate(status_msg->y.error) && zValidator->Validate(status_msg->z.error))
    {
      yValidator->Reset();
      zValidator->Reset();
      ROS_INFO("Slots: Aligned. Approaching target.");
      // Change alignment state, stop timer, activate surge alignment.
      if (mission_state == MST_BIG_RED)
      {
        alignment_state = AST_BBOX;
        align_cmd.surge_active = true;
        master->alignment_pub.publish(align_cmd);
      }
    }
  }
  else if (alignment_state == AST_BBOX)
  {
    if (xValidator->Validate(status_msg->x.error))
    {
      xValidator->Reset();
      if (mission_state == MST_BIG_RED)
      {
        ROS_INFO("Slots: Bounding Box aligned. Attempting to fire.");
        Slots::hitJackpot();
        // Update target alignment (next torpedo)
        if (torpedo_count > 0)
        {
          ROS_INFO("Slots: Aligning to next torpedo");
          alignment_state = AST_CENTER;
          align_cmd.target_pos.y = torpedo_offsets[active_torpedo].y;
          align_cmd.target_pos.z = torpedo_offsets[active_torpedo].z;
          master->alignment_pub.publish(align_cmd);
        }
        else
        {
          ROS_INFO("Slots: Out of torpedoes. Let's go home, boys.");
          Slots::Abort();
          master->tslam->SetEndPos();
          master->StartTask();
        }
      }
    }
  }
}

void Slots::hitJackpot()
{
  if (torpedo_count > 0)
  {
    pneumatics_cmd.header.stamp = ros::Time::now();
    pneumatics_cmd.torpedo_port = active_torpedo == PORT_TORPEDO;
    pneumatics_cmd.torpedo_stbd = active_torpedo == STBD_TORPEDO;
    pneumatics_cmd.markerdropper = false;
    pneumatics_cmd.manipulator = false;
    pneumatics_cmd.duration = pneumatics_duration;
    master->pneumatics_pub.publish(pneumatics_cmd);
    torpedo_count--;
    ROS_INFO("Slots: Torpedo %d fired. Toggling active. %d torpedoes remaining.", active_torpedo, torpedo_count);
    if (active_torpedo == PORT_TORPEDO)
      active_torpedo = STBD_TORPEDO;
    else
      active_torpedo = PORT_TORPEDO;
  }
  else
  {
    ROS_INFO("Slots: CRITICAL! Tried to fire with no torpedoes.");
  }
}

// Shutdown all active subscribers
void Slots::Abort()
{
  Slots::Initialize();

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Slots: Aborting");
}
