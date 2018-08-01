#include "riptide_autonomy/task_dice.h"

#define PI 3.141592653

/* Dice - Order of Execution:
1. Start. Detect a single dice
2. Detect at least 3 dice so we can map their relative positions
3. 

*/

Dice::Dice(BeAutonomous *master)
{
  this->master = master;
  Dice::Initialize();
  ROS_INFO("Dice: Initialized");
}

int Dice::CvtNum2Index(int dice)
{
  switch (dice)
  {
  case DICE1:
    return DICE1_INDEX;
  case DICE2:
    return DICE2_INDEX;
  case DICE5:
    return DICE5_INDEX;
  case DICE6:
    return DICE6_INDEX;
  default:
    ROS_INFO("Dice: Dice number %i does not exist. Setting to DICE1_INDEX", dice);
    return DICE1_INDEX;
  }
}

int Dice::CvtIndex2Num(int index)
{
  switch (index)
  {
  case DICE1_INDEX:
    return DICE1;
  case DICE2_INDEX:
    return DICE2;
  case DICE5_INDEX:
    return DICE5;
  case DICE6_INDEX:
    return DICE6;
  default:
    ROS_INFO("Dice: Dice index %i does not exist. Setting to DICE1", index);
    return DICE1;
  }
}

// Determine if dice block is in top row
bool Dice::IsDiceOnTop(int dice)
{
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      if (detected_dice[CvtNum2Index(dice)])
      {
        if (dice_map[i][j] == dice)
        {
          if (i == 0)
            return true;
          else
            return false;
        }
      }
      else
        return false;
    }
  }
}

// Determine if dice block is in left column
bool Dice::IsDiceOnLeft(int dice)
{
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      if (detected_dice[CvtNum2Index(dice)])
      {
        if (dice_map[i][j] == dice)
        {
          if (j == 0)
            return true;
          else
            return false;
        }
      }
      else
        return false;
    }
  }
}

// Use next dice block in preferred_objects list
void Dice::UpdateCurrentDice()
{
  for (int i = 0; i < num_preferred_objects; i++)
  {
    if (current_dice == preferred_objects.at(i))
    {
      current_dice = preferred_objects.at((i + 1) % num_preferred_objects);
      object_name = master->object_names.at(CvtNum2Index(current_dice));
    }
  }
}

void Dice::Initialize()
{
  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void Dice::Start()
{
  detection1Validator = new DetectionValidator(master->detections_req, master->detection_duration);
  detection2Validator = new DetectionValidator(master->detections_req, master->detection_duration);
  detection5Validator = new DetectionValidator(master->detections_req, master->detection_duration);
  detection6Validator = new DetectionValidator(master->detections_req, master->detection_duration);
  xValidator = new ErrorValidator(master->bbox_thresh, master->bbox_surge_duration);
  yValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  zValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  yawValidator = new ErrorValidator(master->yaw_thresh, master->error_duration);
  depthValidator = new ErrorValidator(master->depth_thresh, master->error_duration);

  // Add preferred objects to list
  num_preferred_objects = (int)master->tasks["tasks"][master->task_id]["object_preferences"].size();
  preferred_objects.clear();
  for (int i = 0; i < num_preferred_objects; i++)
  {
    preferred_objects.push_back(master->tasks["tasks"][master->task_id]["preferred_objects"][i].as<double>());
  }
  current_dice = preferred_objects.at(0);
  object_name = master->object_names.at(CvtNum2Index(current_dice));

  bump_duration = master->tasks["tasks"][master->task_id]["bump_duration"].as<double>();
  backup_duration = master->tasks["tasks"][master->task_id]["backup_duration"].as<double>();
  dice_bbox_width = master->tasks["tasks"][master->task_id]["dice_bbox_width"].as<double>();
  upper_dice_zcenter_offset = master->tasks["tasks"][master->task_id]["upper_dice_zcenter_offset"].as<double>();
  ROS_INFO("Dice: Loaded variables from tasks yaml");

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = object_name; // Dice1, Dice2, Dice5, or Dice6
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)(master->frame_width * dice_bbox_width);
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0; // May have to be updated later
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Dice: Alignment command published (but disabled)");

  for (int i = 0; i < 4; i++)
  {
    *detected_dice[i] = false; // Order is always Dice1, Dice2, Dice5, Dice6
    yCenters[i] = 420;
    zCenters[i] = 420;
  }

  completed[0] = 0;
  completed[1] = 0;
  dice_map[0][0] = 420;
  dice_map[0][1] = 420;
  dice_map[1][0] = 420;
  dice_map[1][1] = 420;
  yCenterAvg = 0;
  zCenterAvg = 0;
  num_dice_detections = 0;
  bumping = true;
  backing_up = false;

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::IDDiceTask, this);
  ROS_INFO("Dice: Subscribed to /task/bboxes");
}

// ID the Dice task and set update target y-pos based on the detected object(s)
void Dice::IDDiceTask(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Figure out which side(s) of the gate we can detect
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
  {
    if (bbox_msg->bounding_boxes.at(i).Class == "Dice1")
    {
      if (detection1Validator->Validate())
        detected_dice1 = true;
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice2")
    {
      if (detection2Validator->Validate())
        detected_dice2 = true;
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice5")
    {
      if (detection5Validator->Validate())
        detected_dice5 = true;
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice6")
    {
      if (detection6Validator->Validate())
        detected_dice6 = true;
    }
  }

  if (detected_dice1 || detected_dice2 || detected_dice5 || detected_dice6)
  {
    task_bbox_sub.shutdown();
    master->tslam->Abort(false);

    // Wait for TSlam to finish braking before proceeding
    timer = master->nh.createTimer(ros::Duration(master->brake_duration), &Dice::EndTSlamTimer, this, true);
    ROS_INFO("Dice; Found Dice field. Awaiting TSlam to end.");

    /*task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::MapDiceField, this);
    ROS_INFO("Dice: Found some crap floating around in TRANSDEC.");*/
  }
}

// Put rest of IDDiceTask code here
void Dice::EndTSlamTimer(const ros::TimerEvent &event)
{
  // Now try to detect at least 3 dice
  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::MapDiceField, this);
  ROS_INFO("Dice: Nvm, It's just some crap floating around in TRANSDEC.");
  ROS_INFO("Dice: Now trying to detect at least 3 dice.");
}

// Perform running avg of dice y-center positions (center of camera is (0,0))
// Use vehicle frame axes
void Dice::UpdateDiceYCenter(int *value, int max, int min)
{
  int center = master->cam_center_x - (max + min) / 2;
  if (*value == 420)
    *value = center;
  else
    *value = (int)((*value + center) / 2);
}

// Perform running avg of dice z-center positions (center of camera is (0,0))
// Use vehicle frame axes
void Dice::UpdateDiceZCenter(int *value, int max, int min)
{
  int center = master->cam_center_y - (max + min) / 2;
  if (*value == 420)
    *value = center;
  else
    *value = (int)((*value + center) / 2);
}

void Dice::MapDiceField(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Figure out which side(s) of the gate we can detect
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
  {
    if (bbox_msg->bounding_boxes.at(i).Class == "Dice1")
    {
      if (detection1Validator->Validate())
        detected_dice1 = true;
      Dice::UpdateDiceYCenter(&yCenters[0], bbox_msg->bounding_boxes.at(i).xmax, bbox_msg->bounding_boxes.at(i).xmin);
      Dice::UpdateDiceZCenter(&zCenters[0], bbox_msg->bounding_boxes.at(i).ymax, bbox_msg->bounding_boxes.at(i).ymin);
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice2")
    {
      if (detection2Validator->Validate())
        detected_dice2 = true;
      Dice::UpdateDiceYCenter(&yCenters[1], bbox_msg->bounding_boxes.at(i).xmax, bbox_msg->bounding_boxes.at(i).xmin);
      Dice::UpdateDiceZCenter(&zCenters[1], bbox_msg->bounding_boxes.at(i).ymax, bbox_msg->bounding_boxes.at(i).ymin);
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice5")
    {
      if (detection5Validator->Validate())
        detected_dice5 = true;
      Dice::UpdateDiceYCenter(&yCenters[2], bbox_msg->bounding_boxes.at(i).xmax, bbox_msg->bounding_boxes.at(i).xmin);
      Dice::UpdateDiceZCenter(&zCenters[2], bbox_msg->bounding_boxes.at(i).ymax, bbox_msg->bounding_boxes.at(i).ymin);
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice6")
    {
      if (detection6Validator->Validate())
        detected_dice6 = true;
      Dice::UpdateDiceYCenter(&yCenters[3], bbox_msg->bounding_boxes.at(i).xmax, bbox_msg->bounding_boxes.at(i).xmin);
      Dice::UpdateDiceZCenter(&zCenters[3], bbox_msg->bounding_boxes.at(i).ymax, bbox_msg->bounding_boxes.at(i).ymin);
    }
  }

  num_dice_detections = (int)detected_dice1 + (int)detected_dice2 + (int)detected_dice5 + (int)detected_dice6;

  if (num_dice_detections >= 3)
  {
    task_bbox_sub.shutdown();
    detection1Validator->Reset();
    detection2Validator->Reset();
    detection5Validator->Reset();
    detection6Validator->Reset();

    // Calculate y and z center avg positions
    for (int i = 0; i < 4; i++)
    {
      if (detected_dice[i])
      {
        yCenterAvg += (yCenters[i] / num_dice_detections);
        zCenterAvg += (zCenters[i] / num_dice_detections);
      }
    }

    // Determine where each dice block is in the map
    // Notation: [0][0] is top left, and [1][1] is bottom right
    for (int i = 0; i < num_dice_detections; i++)
    {
      if (detected_dice[i])
      {
        if (yCenters[i] >= yCenterAvg && zCenters[i] >= zCenterAvg)
          dice_map[0][0] = diceID[i];
        else if (yCenters[i] >= yCenterAvg && zCenters[i] < zCenterAvg)
          dice_map[1][0] = diceID[i];
        else if (yCenters[i] < yCenterAvg && zCenters[i] >= zCenterAvg)
          dice_map[0][1] = diceID[i];
        else if (yCenters[i] < yCenterAvg && zCenters[i] < zCenterAvg)
          dice_map[1][1] = diceID[i];
      }
    }

    // If only 3 dice detected, add the fourh dice block to the map array
    if (num_dice_detections == 3)
    {
      int index = 0;
      for (int i = 0; i < 2; i++)
      {
        for (int j = 0; j < 2; j++)
        {
          if (dice_map[i][j] == 0)
            dice_map[i][j] = diceID[index];
          index++;
        }
      }
    }

    // It is probably safe to assume we generally only detect the 2, 5, and 6
    if (!detected_dice[CvtNum2Index(current_dice)])
    {
      Dice::UpdateCurrentDice(); // Adjust current_dice and name
      align_cmd.object_name = object_name;
    }

    if (IsDiceOnTop(current_dice))
      align_cmd.target_pos.z = -(int)(master->frame_height * upper_dice_zcenter_offset); // KEEP the negative sign

    // Reset detected_dice array
    for (int i = 0; i < 4; i++)
      *detected_dice[i] = false;

    // Activate sway and heave controllers
    align_cmd.surge_active = false;
    align_cmd.sway_active = true;
    align_cmd.heave_active = true;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Dice::Align2FirstDiceYZ, this);
    ROS_INFO("Dice: Dice map complete. Aligning %s to center of frame", object_name.c_str());
  }
}

// Make sure the dice is aligned properly
void Dice::Align2FirstDiceYZ(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (yValidator->Validate(status_msg->y.error) && zValidator->Validate(status_msg->z.error))
  {
    yValidator->Reset();
    zValidator->Reset();
    alignment_status_sub.shutdown();

    // Activate X alignment controller
    align_cmd.surge_active = true;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Dice::Align2FirstDiceBBox, this);
    ROS_INFO("Dice: Aligned to %s. Now aligning on bbox width", object_name.c_str());
  }
}

// Set the bbox to desired width
void Dice::Align2FirstDiceBBox(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (xValidator->Validate(status_msg->x.error))
  {
    xValidator->Reset();
    alignment_status_sub.shutdown();

    // Turn off alignment controller
    align_cmd.surge_active = false;
    align_cmd.sway_active = false;
    align_cmd.heave_active = false;
    master->alignment_pub.publish(align_cmd);

    std_msgs::Float64 msg;
    msg.data = master->search_accel;
    master->x_accel_pub.publish(msg);
    ROS_INFO("Dice: Fully aligned to %s. Ready to play craps", object_name.c_str());

    timer = master->nh.createTimer(ros::Duration(bump_duration), &Dice::FirstDiceBumpTimer, this, true);
    ROS_INFO("Dice: About to bump %s. Timer initiated", object_name.c_str());
  }
}

// Timer to stop bumping the dice and backup, then to set the search depth
void Dice::FirstDiceBumpTimer(const ros::TimerEvent &event)
{
  if (bumping)
  {
    bumping = false;
    backing_up = true;
    completed[num_dice_completed++] = current_dice;

    // Now back up
    std_msgs::Float64 msg;
    msg.data = -master->search_accel;
    master->x_accel_pub.publish(msg);

    timer = master->nh.createTimer(ros::Duration(backup_duration), &Dice::FirstDiceBumpTimer, this, true);
    ROS_INFO("Dice: Bumped %s. Now backing up.", object_name.c_str());
  }
  else if (backing_up)
  {
    bumping = true;
    backing_up = false;

    // Now stop
    std_msgs::Float64 msg;
    msg.data = 0;
    master->x_accel_pub.publish(msg);

    // Publish search depth command
    depth_cmd.active = true;
    depth_cmd.depth = master->search_depth;
    master->depth_pub.publish(depth_cmd);
    depth_status_sub = master->nh.subscribe<riptide_msgs::ControlStatus>("/status/controls/depth", 1, &Dice::DepthStatusCB, this);
    ROS_INFO("Dice: Backed up from %s. Now setting search depth.", object_name.c_str());
  }
}

void Dice::DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr &status_msg)
{
  if (depthValidator->Validate(status_msg->error))
  {
    depthValidator->Reset();
    depth_status_sub.shutdown();

    // Update dice number and save previous dice number
    bool prev_dice = current_dice;
    Dice::UpdateCurrentDice();

    // Set new alignment command for second dice block
    align_cmd.surge_active = false;
    align_cmd.sway_active = false;
    align_cmd.heave_active = false;
    align_cmd.object_name = object_name; // Dice1, Dice2, Dice5, or Dice6
    align_cmd.alignment_plane = master->alignment_plane;
    align_cmd.bbox_dim = (int)(master->frame_width * dice_bbox_width);
    align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
    align_cmd.target_pos.x = 0;
    align_cmd.target_pos.y = 0;
    align_cmd.target_pos.z = 0;

    if (IsDiceOnTop(current_dice))
      align_cmd.target_pos.z = -(int)(master->frame_height * upper_dice_zcenter_offset); // KEEP the negative sign

    master->alignment_pub.publish(align_cmd);

    // Move robot left or right a bit if new dice block is not in view
    // Move either left or right to put second dice closer to camera center
    std_msgs::Float64 msg;
    msg.data = 0;
    if (IsDiceOnLeft(current_dice) && !IsDiceOnLeft(prev_dice)) // Second dice is on the left
      msg.data = master->search_accel;
    else if (IsDiceOnLeft(prev_dice) && !IsDiceOnLeft(current_dice)) // Second dice is on the right
      msg.data = -(master->search_accel);

    if (msg.data != 0) // Need to move over
    {
      master->y_accel_pub.publish(msg);
      timer = master->nh.createTimer(ros::Duration(move_over_duration), &Dice::MoveOverTimer, this, true);
      ROS_INFO("Dice: Moving over to see second dice block, %s.", object_name.c_str());
    }
    else // No need to move over. So start detecting second dice
    {
      task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::IDSecondDice, this);
      ROS_INFO("Dice: No need to move over. Subscribed to /task/bboxes. Searching for second dice.");
    }
  }
}

void Dice::MoveOverTimer(const ros::TimerEvent &event)
{
  std_msgs::Float64 msg;
  msg.data = 0;
  master->y_accel_pub.publish(msg);

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::IDSecondDice, this);
  ROS_INFO("Dice: Moved over. Subscribed to /task/bboxes. Searching for second dice");
}

// Make sure we can detect the second dice
// TODO: add timeout in case we can't detect the second dice
void Dice::IDSecondDice(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Try to detect the second dice
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
  {
    if (bbox_msg->bounding_boxes.at(i).Class == object_name)
    {
      if (detectionValidators[CvtNum2Index(current_dice)]->Validate())
      {
        *detected_dice[CvtNum2Index(current_dice)] = true;
        task_bbox_sub.shutdown();

        //Active sway and heave controllers
        align_cmd.surge_active = false;
        align_cmd.sway_active = true;
        align_cmd.heave_active = true;
        master->alignment_pub.publish(align_cmd);
        alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Dice::Align2SecondDiceYZ, this);
        ROS_INFO("Dice: Found second dice. No aligning to center");
      }
    }
  }
}

// Make sure the dice is aligned properly
void Dice::Align2SecondDiceYZ(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (yValidator->Validate(status_msg->y.error) && zValidator->Validate(status_msg->z.error))
  {
    yValidator->Reset();
    zValidator->Reset();
    alignment_status_sub.shutdown();

    // Activate X alignment controller
    align_cmd.surge_active = true;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Dice::Align2SecondDiceBBox, this);
    ROS_INFO("Dice: Aligned to %s. Now aligning on bbox width", object_name.c_str());
  }
}

// Set the bbox to desired width
void Dice::Align2SecondDiceBBox(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (xValidator->Validate(status_msg->x.error))
  {
    xValidator->Reset();
    alignment_status_sub.shutdown();

    // Turn off alignment controller
    align_cmd.surge_active = false;
    align_cmd.sway_active = false;
    align_cmd.heave_active = false;
    master->alignment_pub.publish(align_cmd);

    std_msgs::Float64 msg;
    msg.data = master->search_accel;
    master->x_accel_pub.publish(msg);
    ROS_INFO("Dice: Fully aligned to %s. Ready to finish playing craps", object_name.c_str());

    timer = master->nh.createTimer(ros::Duration(bump_duration), &Dice::SecondDiceBumpTimer, this, true);
    ROS_INFO("Dice: About to bump %s. Timer initiated", object_name.c_str());
  }
}

// Timer to stop bumping the dice and backup, then to set the search depth
void Dice::SecondDiceBumpTimer(const ros::TimerEvent &event)
{
  if (bumping)
  {
    bumping = false;
    backing_up = true;
    completed[num_dice_completed++] = current_dice;

    // Now back up
    std_msgs::Float64 msg;
    msg.data = -master->search_accel;
    master->x_accel_pub.publish(msg);

    timer = master->nh.createTimer(ros::Duration(backup_duration), &Dice::SecondDiceBumpTimer, this, true);
    ROS_INFO("Dice: Bumped %s. Now backing up.", object_name.c_str());
  }
  else if (backing_up)
  {
    bumping = true;
    backing_up = false;

    // Now stop
    std_msgs::Float64 msg;
    msg.data = 0;
    master->x_accel_pub.publish(msg);

    master->tslam->SetEndPos();
    master->StartTask();

    ROS_INFO("Dice: Backed up from %s. Finished playing craps.", object_name.c_str());
    ROS_INFO("Dice: The bank of benji may or may not be bankrupt by the end of this run.");
  }
}

void Dice::Abort()
{
  Dice::Initialize();
  timer.stop();
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Dice: Aborting");
}