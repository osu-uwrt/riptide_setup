#include "riptide_autonomy/dice.h"

#define PI 3.141592653

/* Dice - Order of Execution:
1. Start. Set alignment command.

*/

Dice::Dice(BeAutonomous *master)
{
  this->master = master;
  Dice::Initialize();
  ROS_INFO("Dice: Initialized");
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

  // Add preferred objects to list
  num_preferred_objects = (int)master->tasks["tasks"][master->task_id]["object_preferences"].size();
  preferred_objects.clear();
  for (int i = 0; i < num_preferred_objects; i++)
  {
    preferred_objects.push_back(master->tasks["tasks"][master->task_id]["preferred_objects"][i].as<string>());
  }
  object_name = preferred_objects.at(0);

  bump_duration = master->tasks["tasks"][master->task_id]["bump_duration"].as<double>();
  dice_bbox_width = master->tasks["tasks"][master->task_id]["dice_bbox_width"].as<double>();
  upper_dice_zcenter_offset = master->tasks["tasks"][master->task_id]["upper_dice_zcenter_offset"].as<double>();

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = object_name; // Casino_Gate Black/Red
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)master->frame_width * dice_bbox_width;
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Dice: Alignment command published (but disabled)");

  detected_dice1 = false;
  detected_dice2 = false;
  detected_dice5 = false;
  detected_dice6 = false;
  completed[0] = 0;
  completed[1] = 0;
  dice_map[0][0] = 420;
  dice_map[0][1] = 420;
  dice_map[1][0] = 420;
  dice_map[1][1] = 420;
  xCenters[0] = 420;
  xCenters[1] = 420;
  xCenters[2] = 420;
  xCenters[3] = 420;
  yCenters[0] = 420;
  yCenters[1] = 420;
  yCenters[2] = 420;
  yCenters[3] = 420;

  num_dice_detections = 0;

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::IDDice, this);
  ROS_INFO("Dice: Subscribed to /task/bboxes");
}

// ID the Dice task and set update target y-pos based on the detected object(s)
void Dice::IDDice(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Figure out which side(s) of the gate we can detect
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
  {
    if (bbox_msg->bounding_boxes.at(i).Class == "Dice1")
    {
      detected_dice1 = detection1Validator->Validate();
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice2")
    {
      detected_dice2 = detection1Validator->Validate();
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice5")
    {
      detected_dice5 = detection1Validator->Validate();
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice6")
    {
      detected_dice6 = detection1Validator->Validate();
    }
  }

  if (detected_dice1 || detected_dice2 || detected_dice5 || detected_dice6)
  {
    task_bbox_sub.shutdown();
    master->tslam->Abort(false);
    task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::MapDiceField, this);
  }
}

void Dice::MapDiceField(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Figure out which side(s) of the gate we can detect
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
  {
    if (bbox_msg->bounding_boxes.at(i).Class == "Dice1")
    {
      detected_dice1 = detection1Validator->Validate();
      Dice::UpdateDiceCenter(yCenter[0], bbox_msg->bounding_boxes.at(i).xmax, bbox_msg->bounding_boxes.at(i).xmin);
      Dice::UpdateDiceCenter(zCenter[0], bbox_msg->bounding_boxes.at(i).ymax, bbox_msg->bounding_boxes.at(i).ymin);
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice2")
    {
      detected_dice2 = detection1Validator->Validate();
      Dice::UpdateDiceCenter(yCenter[1], bbox_msg->bounding_boxes.at(i).xmax, bbox_msg->bounding_boxes.at(i).xmin);
      Dice::UpdateDiceCenter(zCenter[1], bbox_msg->bounding_boxes.at(i).ymax, bbox_msg->bounding_boxes.at(i).ymin);
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice5")
    {
      detected_dice5 = detection1Validator->Validate();
      Dice::UpdateDiceCenter(yCenter[2], bbox_msg->bounding_boxes.at(i).xmax, bbox_msg->bounding_boxes.at(i).xmin);
      Dice::UpdateDiceCenter(zCenter[2], bbox_msg->bounding_boxes.at(i).ymax, bbox_msg->bounding_boxes.at(i).ymin);
    }
    else if (bbox_msg->bounding_boxes.at(i).Class == "Dice6")
    {
      detected_dice6 = detection1Validator->Validate();
      Dice::UpdateDiceCenter(yCenter[3], bbox_msg->bounding_boxes.at(i).xmax, bbox_msg->bounding_boxes.at(i).xmin);
      Dice::UpdateDiceCenter(zCenter[3], bbox_msg->bounding_boxes.at(i).ymax, bbox_msg->bounding_boxes.at(i).ymin);
    }
  }

  num_dice_detections = (int)detected_dice1 + (int)detected_dice2 + (int)detected_dice5 + (int)detected_dice6;

  if (sum_detections >= 3)
  {
    task_bbox_sub.shutdown();
    master->tslam->Abort(false);

    int x_avg, y_avg;
    if (!detected_dice1)
    {
      x_avg = (xCenter[1] + xCenter[2] + xCenter[3]) / 3;
      y_avg = (yCenter[1] + yCenter[2] + yCenter[3]) / 3;
    }
    else if (!detected_dice2)
    {
      x_avg = (xCenter[0] + xCenter[2] + xCenter[3]) / 3;
      y_avg = (yCenter[0] + yCenter[2] + yCenter[3]) / 3;
    }
    else if (!detected_dice5)
    {
      x_avg = (xCenter[0] + xCenter[1] + xCenter[3]) / 3;
      y_avg = (yCenter[0] + yCenter[1] + yCenter[3]) / 3;
    }
    else if (!detected_dice6)
    {
      x_avg = (xCenter[0] + xCenter[1] + xCenter[2]) / 3;
      y_avg = (yCenter[0] + yCenter[1] + yCenter[2]) / 3;
    }
  }
}

// Perform running avg of dice x-center positions
void Dice::UpdateDiceXCenter(int *value, max, min)
{
  int center = master->cam_center_y - (max + min) / 2;
  if (*value == 420)
    *value = center;
  else
    *value = (int)((*value + center) / 2);
}

// Perform running avg of dice x-center positions
void Dice::UpdateDiceYCenter(int *value, max, min)
{
  int center = master->cam_center_x - (max + min) / 2;
  if (*value == 420)
    *value = center;
  else
    *value = (int)((*value + center) / 2);
}

Dice::Dice(BeAutonomous *master)
{
  this->master = master;
  Dice::Initialize();
  ROS_INFO("Dice: Initialized");
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

  // Add preferred objects to list
  num_preferred_objects = (int)master->tasks["tasks"][master->task_id]["object_preferences"].size();
  preferred_objects.clear();
  for (int i = 0; i < num_preferred_objects; i++)
  {
    preferred_objects.push_back(master->tasks["tasks"][master->task_id]["preferred_objects"][i].as<string>());
  }
  object_name = preferred_objects.at(0);

  bump_duration = master->tasks["tasks"][master->task_id]["bump_duration"].as<double>();
  dice_bbox_width = master->tasks["tasks"][master->task_id]["dice_bbox_width"].as<double>();
  upper_dice_zcenter_offset = master->tasks["tasks"][master->task_id]["upper_dice_zcenter_offset"].as<double>();

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = object_name; // Casino_Gate Black/Red
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)master->frame_width * dice_bbox_width;
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Dice: Alignment command published (but disabled)");

  detected_dice1 = false;
  detected_dice2 = false;
  detected_dice5 = false;
  detected_dice6 = false;
  completed[0] = 0;
  completed[1] = 0;
  dice_map[0][0] = 0;
  dice_map[0][1] = 0;
  dice_map[1][0] = 0;
  dice_map[1][1] = 0;

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::IDDice, this);
  ROS_INFO("Dice: Subscribed to /task/bboxes");
}

// ID the Dice task and set update target y-pos based on the detected object(s)
// TODO: Add a timeout? Maybe not since this has to be completed before we attempt anything else
void Dice::IDDice(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Figure out which side(s) of the gate we can detect
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
  {
    if (bbox_msg->bounding_boxes.at(i).Class == "Casino_Gate_Black")
    {
      detections_black++;
      detected_black = ValidateDetections(&detections_black, &detection_duration_black, master->detections_req, master->detection_duration, &detect_black_start, &attempts_black);
      if (!detected_black)
        ROS_INFO("DiceBlack: %i Attemps - %i detections in %f sec", attempts_black, detections_black, detection_duration_black);
    }
    else
    {
      detections_red++;
      detected_red = ValidateDetections(&detections_red, &detection_duration_red, master->detections_req, master->detection_duration, &detect_red_start, &attempts_red);
      if (!detected_red)
        ROS_INFO("DiceBlack: %i Attemps - %i detections in %f sec", attempts_red, detections_red, detection_duration_red);
    }
  }

  // Set the side we are passing on and update the target y-pos in the frame
  if (detected_black || detected_red)
  {
    task_bbox_sub.shutdown();
    master->tslam->Abort(true);

    if (master->color == left_color) // Must pass on LEFT side
    {
      passing_on_left = true;
      if (detected_black && detected_red) // Both detected - align object to center of frame
      {
        ROS_INFO("Dice: Detected both sides. Aligning left side in center");
      }
      else if ((detected_red && left_color == rc::COLOR_BLACK) || (detected_black && left_color == rc::COLOR_RED))
      {
        // Detected the right side of the gate, so put target y-pos on right side of frame using the right side
        align_cmd.object_name = master->object_names.at((master->color + 1) % 2); // Switch to detected object
        align_cmd.target_pos.y = -(int)(master->frame_height / 3);
        ROS_INFO("Dice: Detected right side. Aligning to left side");
      }
      else
        ROS_INFO("Dice: Detected left side. Aligning to center");
    }
    else // Must pass on RIGHT side
    {
      passing_on_right = true;
      if (detected_black && detected_red) // Both detected - align object to center of frame
      {
        ROS_INFO("Dice: Detected both sides. Aligning right side in center");
      }
      else if ((detected_black && left_color == rc::COLOR_BLACK) || (detected_black && left_color == rc::COLOR_RED))
      {
        // Detected the left side of the gate, so put target y-pos on left side of frame using the left side
        align_cmd.object_name = master->object_names.at((master->color + 1) % 2); // Switch to detected object
        align_cmd.target_pos.y = (int)(master->frame_height / 3);
        ROS_INFO("Dice: Detected left side. Aligning to right side");
      }
      else
        ROS_INFO("Dice: Detected right side. Aligning to center");
    }

    align_cmd.surge_active = false;
    align_cmd.sway_active = true;
    align_cmd.heave_active = true;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Dice::AlignmentStatusCB, this);
    ROS_INFO("Dice: Aligning to %s. Checking sway/heave error", object_name.c_str());
  }
}

// A. Make sure the vehicle is aligned with the YZ controllers
// B. Make sure the vehicle is aligned with the X controller
void Dice::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (align_id == ALIGN_YZ) // Perform (A) - YZ alignment
  {
    if (ValidateError2(status_msg->y.error, status_msg->z.error, &error_duration, master->align_thresh, master->error_durationk_is_ticking, &error_check_start))
    {
      align_id = ALIGN_BBOX_WIDTH; // Verify if bbox alignment will work
      // Activate X alignment controller
      align_cmd.surge_active = true;
      master->alignment_pub.publish(align_cmd);
      ROS_INFO("Dice: Color in correct spot. Aligning on bbox width");
    }
  }
  else if (align_id == ALIGN_BBOX_WIDTH) // Perform (B) - X alignment
  {
    if (ValidateError(status_msg->x.error, &error_duration, master->bbox_thresh, master->bbox_surge_duration, &clock_is_ticking, &error_check_start))
    {
      alignment_status_sub.shutdown();

      // Publish attitude command
      attitude_cmd.roll_active = true;
      attitude_cmd.pitch_active = true;
      attitude_cmd.yaw_active = true;
      attitude_cmd.euler_rpy.x = 0;
      attitude_cmd.euler_rpy.y = 0;
      attitude_cmd.euler_rpy.z = gate_heading;
      master->attitude_pub.publish(attitude_cmd);

      attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &Dice::AttitudeStatusCB, this);
      ROS_INFO("Dice: Published gate heading. Checking heading.");
    }
  }
}

// Make sure the robot is at the correct heading based on the quadrant
void Dice::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
{
  if (ValidateError(status_msg->yaw.error, &error_duration, master->yaw_thresh, master->error_durationk_is_ticking, &error_check_start))
  {
    attitude_status_sub.shutdown();

    // Disable alignment controller so vehicle can move forward
    align_cmd.surge_active = false;
    align_cmd.sway_active = false;
    align_cmd.heave_active = false;
    master->alignment_pub.publish(align_cmd);
    ROS_INFO("Dice: Aligned to Dice with linear and attitude controllers");

    // Publish forward accel and call it a success after a few seconds
    ROS_INFO("Dice: Now going to pass thru gate");
    std_msgs::Float64 msg;
    msg.data = master->search_accel;
    master->x_accel_pub.publish(msg);

    pass_thru_duration = master->tasks["tasks"][master->task_id]["pass_thru_duration"].as<double>();
    timer = master->nh.createTimer(ros::Duration(pass_thru_duration), &Dice::PassThruTimer, this, true);
    ROS_INFO("Dice: Don't move gate, I'm coming for ya. Abort timer initiated. ETA: %f", pass_thru_duration);
  }

  // Alignment is good, now verify heading error
  if (abs(status_msg->yaw.error) < master->yaw_thresh)
  {
    if (!clock_is_ticking)
    {
      error_check_start = ros::Time::now();
      clock_is_ticking = true;
    }
    else
      error_duration = ros::Time::now().toSec() - error_check_start.toSec();

    if (error_duration >= master->error_duration{
      // Shutdown alignment callback
      attitude_status_sub.shutdown();
      error_duration = 0;
      clock_is_ticking = false;

      // Disable alignment controller so vehicle can move forward
      align_cmd.surge_active = false;
      align_cmd.sway_active = false;
      align_cmd.heave_active = false;
      master->alignment_pub.publish(align_cmd);
      ROS_INFO("Dice: Aligned to Dice with linear and attitude controllers");

      // Publish forward accel and call it a success after a few seconds
      ROS_INFO("Dice: Now going to pass thru gate");
      std_msgs::Float64 msg;
      msg.data = master->search_accel;
      master->x_accel_pub.publish(msg);

      pass_thru_duration = master->tasks["tasks"][master->task_id]["pass_thru_duration"].as<double>();
      timer = master->nh.createTimer(ros::Duration(pass_thru_duration), &Dice::PassThruTimer, this, true);
      ROS_INFO("Dice: Don't move gate, I'm coming for ya. Abort timer initiated. ETA: %f", pass_thru_duration);
    }
  }
  else
  {
    error_duration = 0;
    clock_is_ticking = false;
  }
}

void Dice::PassThruTimer(const ros::TimerEvent &event)
{
  geometry_msgs::Vector3 msg;
  msg.x = 0;
  msg.y = 0;
  msg.z = 0;
  if (!passed_thru_gate)
  {
    passed_thru_gate = true;
    msg.data = -(master->search_accel);
    master->x_accel_pub.publish(msg);
    timer = master->nh.createTimer(ros::Duration(0.25), &Dice::PassThruTimer, this, true);
    ROS_INFO("Dice: Passed thru gate...I think. Timer set for braking.");
  }
  else if (passed_thru_gate && !braked)
  {
    braked = true;
    master->x_accel_pub.publish(msg);
    ROS_INFO("Dice: Completed. Thruster brake applied.");
    Dice::SetEndPos();
    Dice::Abort();
    master->StartTask();
  }

  Dice::Dice(BeAutonomous * master)
  {
    this->master = master;
    Dice::Initialize();
    ROS_INFO("Dice: Initialized");
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

    // Add preferred objects to list
    num_preferred_objects = (int)master->tasks["tasks"][master->task_id]["object_preferences"].size();
    preferred_objects.clear();
    for (int i = 0; i < num_preferred_objects; i++)
    {
      preferred_objects.push_back(master->tasks["tasks"][master->task_id]["preferred_objects"][i].as<string>());
    }
    object_name = preferred_objects.at(0);

    bump_duration = master->tasks["tasks"][master->task_id]["bump_duration"].as<double>();
    dice_bbox_width = master->tasks["tasks"][master->task_id]["dice_bbox_width"].as<double>();
    upper_dice_zcenter_offset = master->tasks["tasks"][master->task_id]["upper_dice_zcenter_offset"].as<double>();

    align_cmd.surge_active = false;
    align_cmd.sway_active = false;
    align_cmd.heave_active = false;
    align_cmd.object_name = object_name; // Casino_Gate Black/Red
    align_cmd.alignment_plane = master->alignment_plane;
    align_cmd.bbox_dim = (int)master->frame_width * dice_bbox_width;
    align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
    align_cmd.target_pos.x = 0;
    align_cmd.target_pos.y = 0;
    align_cmd.target_pos.z = 0;
    master->alignment_pub.publish(align_cmd);
    ROS_INFO("Dice: Alignment command published (but disabled)");

    detected_dice1 = false;
    detected_dice2 = false;
    detected_dice5 = false;
    detected_dice6 = false;
    completed[0] = 0;
    completed[1] = 0;
    dice_map[0][0] = 0;
    dice_map[0][1] = 0;
    dice_map[1][0] = 0;
    dice_map[1][1] = 0;

    task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::IDDice, this);
    ROS_INFO("Dice: Subscribed to /task/bboxes");
  }

  // ID the Dice task and set update target y-pos based on the detected object(s)
  // TODO: Add a timeout? Maybe not since this has to be completed before we attempt anything else
  void Dice::IDDice(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
  {
    // Figure out which side(s) of the gate we can detect
    for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
    {
      if (bbox_msg->bounding_boxes.at(i).Class == "Casino_Gate_Black")
      {
        detections_black++;
        detected_black = ValidateDetections(&detections_black, &detection_duration_black, master->detections_req, master->detection_duration, &detect_black_start, &attempts_black);
        if (!detected_black)
          ROS_INFO("DiceBlack: %i Attemps - %i detections in %f sec", attempts_black, detections_black, detection_duration_black);
      }
      else
      {
        detections_red++;
        detected_red = ValidateDetections(&detections_red, &detection_duration_red, master->detections_req, master->detection_duration, &detect_red_start, &attempts_red);
        if (!detected_red)
          ROS_INFO("DiceBlack: %i Attemps - %i detections in %f sec", attempts_red, detections_red, detection_duration_red);
      }
    }

    // Set the side we are passing on and update the target y-pos in the frame
    if (detected_black || detected_red)
    {
      task_bbox_sub.shutdown();
      master->tslam->Abort(true);

      if (master->color == left_color) // Must pass on LEFT side
      {
        passing_on_left = true;
        if (detected_black && detected_red) // Both detected - align object to center of frame
        {
          ROS_INFO("Dice: Detected both sides. Aligning left side in center");
        }
        else if ((detected_red && left_color == rc::COLOR_BLACK) || (detected_black && left_color == rc::COLOR_RED))
        {
          // Detected the right side of the gate, so put target y-pos on right side of frame using the right side
          align_cmd.object_name = master->object_names.at((master->color + 1) % 2); // Switch to detected object
          align_cmd.target_pos.y = -(int)(master->frame_height / 3);
          ROS_INFO("Dice: Detected right side. Aligning to left side");
        }
        else
          ROS_INFO("Dice: Detected left side. Aligning to center");
      }
      else // Must pass on RIGHT side
      {
        passing_on_right = true;
        if (detected_black && detected_red) // Both detected - align object to center of frame
        {
          ROS_INFO("Dice: Detected both sides. Aligning right side in center");
        }
        else if ((detected_black && left_color == rc::COLOR_BLACK) || (detected_black && left_color == rc::COLOR_RED))
        {
          // Detected the left side of the gate, so put target y-pos on left side of frame using the left side
          align_cmd.object_name = master->object_names.at((master->color + 1) % 2); // Switch to detected object
          align_cmd.target_pos.y = (int)(master->frame_height / 3);
          ROS_INFO("Dice: Detected left side. Aligning to right side");
        }
        else
          ROS_INFO("Dice: Detected right side. Aligning to center");
      }

      align_cmd.surge_active = false;
      align_cmd.sway_active = true;
      align_cmd.heave_active = true;
      master->alignment_pub.publish(align_cmd);
      alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Dice::AlignmentStatusCB, this);
      ROS_INFO("Dice: Aligning to %s. Checking sway/heave error", object_name.c_str());
    }
  }

  // A. Make sure the vehicle is aligned with the YZ controllers
  // B. Make sure the vehicle is aligned with the X controller
  void Dice::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
  {
    if (align_id == ALIGN_YZ) // Perform (A) - YZ alignment
    {
      if (ValidateError2(status_msg->y.error, status_msg->z.error, &error_duration, master->align_thresh, master->error_durationk_is_ticking, &error_check_start))
      {
        align_id = ALIGN_BBOX_WIDTH; // Verify if bbox alignment will work
        // Activate X alignment controller
        align_cmd.surge_active = true;
        master->alignment_pub.publish(align_cmd);
        ROS_INFO("Dice: Color in correct spot. Aligning on bbox width");
      }
    }
    else if (align_id == ALIGN_BBOX_WIDTH) // Perform (B) - X alignment
    {
      if (ValidateError(status_msg->x.error, &error_duration, master->bbox_thresh, master->bbox_surge_duration, &clock_is_ticking, &error_check_start))
      {
        alignment_status_sub.shutdown();

        // Publish attitude command
        attitude_cmd.roll_active = true;
        attitude_cmd.pitch_active = true;
        attitude_cmd.yaw_active = true;
        attitude_cmd.euler_rpy.x = 0;
        attitude_cmd.euler_rpy.y = 0;
        attitude_cmd.euler_rpy.z = gate_heading;
        master->attitude_pub.publish(attitude_cmd);

        attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &Dice::AttitudeStatusCB, this);
        ROS_INFO("Dice: Published gate heading. Checking heading.");
      }
    }
  }

  // Make sure the robot is at the correct heading based on the quadrant
  void Dice::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
  {
    if (ValidateError(status_msg->yaw.error, &error_duration, master->yaw_thresh, master->error_durationk_is_ticking, &error_check_start))
    {
      attitude_status_sub.shutdown();

      // Disable alignment controller so vehicle can move forward
      align_cmd.surge_active = false;
      align_cmd.sway_active = false;
      align_cmd.heave_active = false;
      master->alignment_pub.publish(align_cmd);
      ROS_INFO("Dice: Aligned to Dice with linear and attitude controllers");

      // Publish forward accel and call it a success after a few seconds
      ROS_INFO("Dice: Now going to pass thru gate");
      std_msgs::Float64 msg;
      msg.data = master->search_accel;
      master->x_accel_pub.publish(msg);

      pass_thru_duration = master->tasks["tasks"][master->task_id]["pass_thru_duration"].as<double>();
      timer = master->nh.createTimer(ros::Duration(pass_thru_duration), &Dice::PassThruTimer, this, true);
      ROS_INFO("Dice: Don't move gate, I'm coming for ya. Abort timer initiated. ETA: %f", pass_thru_duration);
    }

    // Alignment is good, now verify heading error
    if (abs(status_msg->yaw.error) < master->yaw_thresh)
    {
      if (!clock_is_ticking)
      {
        error_check_start = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        error_duration = ros::Time::now().toSec() - error_check_start.toSec();

    if (error_duration >= master->error_duration{
        // Shutdown alignment callback
        attitude_status_sub.shutdown();
        error_duration = 0;
        clock_is_ticking = false;

        // Disable alignment controller so vehicle can move forward
        align_cmd.surge_active = false;
        align_cmd.sway_active = false;
        align_cmd.heave_active = false;
        master->alignment_pub.publish(align_cmd);
        ROS_INFO("Dice: Aligned to Dice with linear and attitude controllers");

        // Publish forward accel and call it a success after a few seconds
        ROS_INFO("Dice: Now going to pass thru gate");
        std_msgs::Float64 msg;
        msg.data = master->search_accel;
        master->x_accel_pub.publish(msg);

        pass_thru_duration = master->tasks["tasks"][master->task_id]["pass_thru_duration"].as<double>();
        timer = master->nh.createTimer(ros::Duration(pass_thru_duration), &Dice::PassThruTimer, this, true);
        ROS_INFO("Dice: Don't move gate, I'm coming for ya. Abort timer initiated. ETA: %f", pass_thru_duration);
    }
    }
    else
    {
      error_duration = 0;
      clock_is_ticking = false;
    }
  }

  void Dice::PassThruTimer(const ros::TimerEvent &event)
  {
    geometry_msgs::Vector3 msg;
    msg.x = 0;
    msg.y = 0;
    msg.z = 0;
    if (!passed_thru_gate)
    {
      passed_thru_gate = true;
      msg.data = -(master->search_accel);
      master->x_accel_pub.publish(msg);
      timer = master->nh.createTimer(ros::Duration(0.25), &Dice::PassThruTimer, this, true);
      ROS_INFO("Dice: Passed thru gate...I think. Timer set for braking.");
    }
    else if (passed_thru_gate && !braked)
    {
      braked = true;
      master->x_accel_pub.publish(msg);
      ROS_INFO("Dice: Completed. Thruster brake applied.");
      Dice::SetEndPos();
      Dice::Abort();
      master->StartTask();
    }
  }

  // Calculate end position based on whether vehicle passed thru on left or right side of gate
  void Dice::SetEndPos()
  {
    double alpha = 0;

    if (passing_on_left)
      alpha = gate_heading + 90;
    else if (passing_on_right)
      alpha = gate_heading - 90;

    double end_mid_x = master->tslam->task_map["task_map"][master->task_id]["end_mid_x"][master->tslam->quadrant].as<double>();
    double end_mid_y = master->tslam->task_map["task_map"][master->task_id]["end_mid_y"][master->tslam->quadrant].as<double>();

    double current_x = end_mid_x - end_pos_offset * sin(alpha * PI / 180);
    double current_y = end_mid_y + end_pos_offset * cos(alpha * PI / 180);

    master->tslam->SetPos(current_x, current_y);
  }

  // Shutdown all active subscribers
  void Dice::Abort()
  {
    Dice::Initialize();
    timer.stop();
    align_cmd.surge_active = false;
    align_cmd.sway_active = false;
    align_cmd.heave_active = false;
    master->alignment_pub.publish(align_cmd);
    ROS_INFO("Dice: Aborting");

    Dice::Dice(BeAutonomous * master)
    {
      this->master = master;
      Dice::Initialize();
      ROS_INFO("Dice: Initialized");
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

      // Add preferred objects to list
      num_preferred_objects = (int)master->tasks["tasks"][master->task_id]["object_preferences"].size();
      preferred_objects.clear();
      for (int i = 0; i < num_preferred_objects; i++)
      {
        preferred_objects.push_back(master->tasks["tasks"][master->task_id]["preferred_objects"][i].as<string>());
      }
      object_name = preferred_objects.at(0);

      bump_duration = master->tasks["tasks"][master->task_id]["bump_duration"].as<double>();
      dice_bbox_width = master->tasks["tasks"][master->task_id]["dice_bbox_width"].as<double>();
      upper_dice_zcenter_offset = master->tasks["tasks"][master->task_id]["upper_dice_zcenter_offset"].as<double>();

      align_cmd.surge_active = false;
      align_cmd.sway_active = false;
      align_cmd.heave_active = false;
      align_cmd.object_name = object_name; // Casino_Gate Black/Red
      align_cmd.alignment_plane = master->alignment_plane;
      align_cmd.bbox_dim = (int)master->frame_width * dice_bbox_width;
      align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
      align_cmd.target_pos.x = 0;
      align_cmd.target_pos.y = 0;
      align_cmd.target_pos.z = 0;
      master->alignment_pub.publish(align_cmd);
      ROS_INFO("Dice: Alignment command published (but disabled)");

      detected_dice1 = false;
      detected_dice2 = false;
      detected_dice5 = false;
      detected_dice6 = false;
      completed[0] = 0;
      completed[1] = 0;
      dice_map[0][0] = 0;
      dice_map[0][1] = 0;
      dice_map[1][0] = 0;
      dice_map[1][1] = 0;

      task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::IDDice, this);
      ROS_INFO("Dice: Subscribed to /task/bboxes");
    }

    // ID the Dice task and set update target y-pos based on the detected object(s)
    // TODO: Add a timeout? Maybe not since this has to be completed before we attempt anything else
    void Dice::IDDice(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
    {
      // Figure out which side(s) of the gate we can detect
      for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
      {
        if (bbox_msg->bounding_boxes.at(i).Class == "Casino_Gate_Black")
        {
          detections_black++;
          detected_black = ValidateDetections(&detections_black, &detection_duration_black, master->detections_req, master->detection_duration, &detect_black_start, &attempts_black);
          if (!detected_black)
            ROS_INFO("DiceBlack: %i Attemps - %i detections in %f sec", attempts_black, detections_black, detection_duration_black);
        }
        else
        {
          detections_red++;
          detected_red = ValidateDetections(&detections_red, &detection_duration_red, master->detections_req, master->detection_duration, &detect_red_start, &attempts_red);
          if (!detected_red)
            ROS_INFO("DiceBlack: %i Attemps - %i detections in %f sec", attempts_red, detections_red, detection_duration_red);
        }
      }

      // Set the side we are passing on and update the target y-pos in the frame
      if (detected_black || detected_red)
      {
        task_bbox_sub.shutdown();
        master->tslam->Abort(true);

        if (master->color == left_color) // Must pass on LEFT side
        {
          passing_on_left = true;
          if (detected_black && detected_red) // Both detected - align object to center of frame
          {
            ROS_INFO("Dice: Detected both sides. Aligning left side in center");
          }
          else if ((detected_red && left_color == rc::COLOR_BLACK) || (detected_black && left_color == rc::COLOR_RED))
          {
            // Detected the right side of the gate, so put target y-pos on right side of frame using the right side
            align_cmd.object_name = master->object_names.at((master->color + 1) % 2); // Switch to detected object
            align_cmd.target_pos.y = -(int)(master->frame_height / 3);
            ROS_INFO("Dice: Detected right side. Aligning to left side");
          }
          else
            ROS_INFO("Dice: Detected left side. Aligning to center");
        }
        else // Must pass on RIGHT side
        {
          passing_on_right = true;
          if (detected_black && detected_red) // Both detected - align object to center of frame
          {
            ROS_INFO("Dice: Detected both sides. Aligning right side in center");
          }
          else if ((detected_black && left_color == rc::COLOR_BLACK) || (detected_black && left_color == rc::COLOR_RED))
          {
            // Detected the left side of the gate, so put target y-pos on left side of frame using the left side
            align_cmd.object_name = master->object_names.at((master->color + 1) % 2); // Switch to detected object
            align_cmd.target_pos.y = (int)(master->frame_height / 3);
            ROS_INFO("Dice: Detected left side. Aligning to right side");
          }
          else
            ROS_INFO("Dice: Detected right side. Aligning to center");
        }

        align_cmd.surge_active = false;
        align_cmd.sway_active = true;
        align_cmd.heave_active = true;
        master->alignment_pub.publish(align_cmd);
        alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Dice::AlignmentStatusCB, this);
        ROS_INFO("Dice: Aligning to %s. Checking sway/heave error", object_name.c_str());
      }
    }

    // A. Make sure the vehicle is aligned with the YZ controllers
    // B. Make sure the vehicle is aligned with the X controller
    void Dice::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
    {
      if (align_id == ALIGN_YZ) // Perform (A) - YZ alignment
      {
        if (ValidateError2(status_msg->y.error, status_msg->z.error, &error_duration, master->align_thresh, master->error_durationk_is_ticking, &error_check_start))
        {
          align_id = ALIGN_BBOX_WIDTH; // Verify if bbox alignment will work
          // Activate X alignment controller
          align_cmd.surge_active = true;
          master->alignment_pub.publish(align_cmd);
          ROS_INFO("Dice: Color in correct spot. Aligning on bbox width");
        }
      }
      else if (align_id == ALIGN_BBOX_WIDTH) // Perform (B) - X alignment
      {
        if (ValidateError(status_msg->x.error, &error_duration, master->bbox_thresh, master->bbox_surge_duration, &clock_is_ticking, &error_check_start))
        {
          alignment_status_sub.shutdown();

          // Publish attitude command
          attitude_cmd.roll_active = true;
          attitude_cmd.pitch_active = true;
          attitude_cmd.yaw_active = true;
          attitude_cmd.euler_rpy.x = 0;
          attitude_cmd.euler_rpy.y = 0;
          attitude_cmd.euler_rpy.z = gate_heading;
          master->attitude_pub.publish(attitude_cmd);

          attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &Dice::AttitudeStatusCB, this);
          ROS_INFO("Dice: Published gate heading. Checking heading.");
        }
      }
    }

    // Make sure the robot is at the correct heading based on the quadrant
    void Dice::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
    {
      if (ValidateError(status_msg->yaw.error, &error_duration, master->yaw_thresh, master->error_durationk_is_ticking, &error_check_start))
      {
        attitude_status_sub.shutdown();

        // Disable alignment controller so vehicle can move forward
        align_cmd.surge_active = false;
        align_cmd.sway_active = false;
        align_cmd.heave_active = false;
        master->alignment_pub.publish(align_cmd);
        ROS_INFO("Dice: Aligned to Dice with linear and attitude controllers");

        // Publish forward accel and call it a success after a few seconds
        ROS_INFO("Dice: Now going to pass thru gate");
        std_msgs::Float64 msg;
        msg.data = master->search_accel;
        master->x_accel_pub.publish(msg);

        pass_thru_duration = master->tasks["tasks"][master->task_id]["pass_thru_duration"].as<double>();
        timer = master->nh.createTimer(ros::Duration(pass_thru_duration), &Dice::PassThruTimer, this, true);
        ROS_INFO("Dice: Don't move gate, I'm coming for ya. Abort timer initiated. ETA: %f", pass_thru_duration);
      }

      // Alignment is good, now verify heading error
      if (abs(status_msg->yaw.error) < master->yaw_thresh)
      {
        if (!clock_is_ticking)
        {
          error_check_start = ros::Time::now();
          clock_is_ticking = true;
        }
        else
          error_duration = ros::Time::now().toSec() - error_check_start.toSec();

    if (error_duration >= master->error_duration{
          // Shutdown alignment callback
          attitude_status_sub.shutdown();
          error_duration = 0;
          clock_is_ticking = false;

          // Disable alignment controller so vehicle can move forward
          align_cmd.surge_active = false;
          align_cmd.sway_active = false;
          align_cmd.heave_active = false;
          master->alignment_pub.publish(align_cmd);
          ROS_INFO("Dice: Aligned to Dice with linear and attitude controllers");

          // Publish forward accel and call it a success after a few seconds
          ROS_INFO("Dice: Now going to pass thru gate");
          std_msgs::Float64 msg;
          msg.data = master->search_accel;
          master->x_accel_pub.publish(msg);

          pass_thru_duration = master->tasks["tasks"][master->task_id]["pass_thru_duration"].as<double>();
          timer = master->nh.createTimer(ros::Duration(pass_thru_duration), &Dice::PassThruTimer, this, true);
          ROS_INFO("Dice: Don't move gate, I'm coming for ya. Abort timer initiated. ETA: %f", pass_thru_duration);
    }
      }
      else
      {
        error_duration = 0;
        clock_is_ticking = false;
      }
    }

    void Dice::PassThruTimer(const ros::TimerEvent &event)
    {
      geometry_msgs::Vector3 msg;
      msg.x = 0;
      msg.y = 0;
      msg.z = 0;
      if (!passed_thru_gate)
      {
        passed_thru_gate = true;
        msg.data = -(master->search_accel);
        master->x_accel_pub.publish(msg);
        timer = master->nh.createTimer(ros::Duration(0.25), &Dice::PassThruTimer, this, true);
        ROS_INFO("Dice: Passed thru gate...I think. Timer set for braking.");
      }
      else if (passed_thru_gate && !braked)
      {
        braked = true;
        master->x_accel_pub.publish(msg);
        ROS_INFO("Dice: Completed. Thruster brake applied.");
        Dice::SetEndPos();
        Dice::Abort();
        master->StartTask();
      }
    }

    // Calculate end position based on whether vehicle passed thru on left or right side of gate
    void Dice::SetEndPos()
    {
      double alpha = 0;

      if (passing_on_left)
        alpha = gate_heading + 90;
      else if (passing_on_right)
        alpha = gate_heading - 90;

      double end_mid_x = master->tslam->task_map["task_map"][master->task_id]["end_mid_x"][master->tslam->quadrant].as<double>();
      double end_mid_y = master->tslam->task_map["task_map"][master->task_id]["end_mid_y"][master->tslam->quadrant].as<double>();

      double current_x = end_mid_x - end_pos_offset * sin(alpha * PI / 180);
      double current_y = end_mid_y + end_pos_offset * cos(alpha * PI / 180);

      master->tslam->SetPos(current_x, current_y);
    }

    // Shutdown all active subscribers
    void Dice::Abort()
    {
      Dice::Initialize();
      timer.stop();
      align_cmd.surge_active = false;
      align_cmd.sway_active = false;
      align_cmd.heave_active = false;
      master->alignment_pub.publish(align_cmd);
      ROS_INFO("Dice: Aborting");
    