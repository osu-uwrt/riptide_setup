#include "riptide_autonomy/dice.h"

#define PI 3.141592653
#define ALIGN_YZ 0
#define ALIGN_BBOX_WIDTH 1

/* Casino_Gate - Order of Execution:
1. Start. Set alignment command.
2. ID any side(s) of the gate. Compare which side is where, and update alignment command accordingly.
3. Align required side to center of frame with the bbox towards the top and of certain width.
4. Lock in suitable depth and maintain.
5. Drive forward for specified duration and call it a success
6. Abort. Start next task.
*/

Dice::Dice(BeAutonomous *master)
{
  this->master = master;
  Dice::Initialize();
  ROS_INFO("Dice: Initialized");
}

void Dice::Initialize()
{
  detections_black = 0;
  detections_red = 0;
  attempts_black = 0;
  attempts_red = 0;
  gate_heading = 0;
  align_id = ALIGN_YZ;

  detection_duration_black = 0;
  detection_duration_red = 0;
  error_duration = 0;
  clock_is_ticking = false;
  braked = false;
  passing_on_right = false;
  passing_on_left = false;

  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void Dice::Start()
{
  object_name = (master->color == rc::COLOR_BLACK) ? master->object_names.at(0) : master->object_names.at(1); // Black side if statement true, Red otherwise
  gate_heading = master->tslam->task_map["task_map"][master->tslam->quadrant]["map"][master->task_id]["gate_heading"].as<double>();
  end_pos_offset = master->tasks["tasks"][master->task_id]["end_pos_offset"].as<double>();
  left_color = master->tslam->task_map["task_map"][master->tslam->quadrant]["map"]["left_colot"].as<int>();

  // Set to black and just hope it's right if it doesn't load
  if (left_color != rc::COLOR_BLACK && left_color != rc::COLOR_RED)
    left_color = rc::COLOR_BLACK;

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = object_name; // Casino_Gate Black/Red
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)master->frame_width * 0.6;
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = (int)(master->frame_height / 4);
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Dice: alignment command published (but disabled)");

  detected_black = false;
  detected_red = false;
  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Dice::IDDice, this);
  ROS_INFO("Dice: subscribed to /task/bboxes");
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
      detected_black = ValidateDetections(&detections_black, &detection_duration_black, master->detections_req, master->detection_duration_thresh, &detect_black_start, &attempts_black);
      if (!detected_black)
        ROS_INFO("DiceBlack: %i Attemps - %i detections in %f sec", attempts_black, detections_black, detection_duration_black);
    }
    else
    {
      detections_red++;
      detected_red = ValidateDetections(&detections_red, &detection_duration_red, master->detections_req, master->detection_duration_thresh, &detect_red_start, &attempts_red);
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
    if (ValidateError2(status_msg->y.error, status_msg->z.error, &error_duration, master->align_thresh, master->error_duration_thresh, &clock_is_ticking, &error_check_start))
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
    if (ValidateError(status_msg->x.error, &error_duration, master->bbox_thresh, master->bbox_surge_duration_thresh, &clock_is_ticking, &error_check_start))
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
  if (ValidateError(status_msg->yaw.error, &error_duration, master->yaw_thresh, master->error_duration_thresh, &clock_is_ticking, &error_check_start))
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

    if (error_duration >= master->error_duration_thresh)
    {
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
}
