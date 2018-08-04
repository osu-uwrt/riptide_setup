#include "riptide_autonomy/task_casino_gate.h"

#define PI 3.141592653

/* Casino_Gate - Order of Execution:
1. Start. Set alignment command.
2. ID the black side of the gate.
3. Align required side to center of frame with the bbox towards the top and of certain width.
4. Lock in suitable depth and maintain.
5. Drive forward for specified duration and call it a success
6. Abort. Start next task.
*/

CasinoGate::CasinoGate(BeAutonomous *master)
{
  this->master = master;
  CasinoGate::Initialize();
  ROS_INFO("CasinoGate: Initialized");
}

void CasinoGate::Initialize()
{
  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void CasinoGate::Start()
{
  detectionValidator = new DetectionValidator(master->detections_req, master->detection_duration);
  xValidator = new ErrorValidator(master->bbox_thresh, master->bbox_surge_duration);
  yValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  zValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  yawValidator = new ErrorValidator(master->yaw_thresh, master->error_duration);
  depthValidator = new ErrorValidator(master->depth_thresh, master->error_duration);

  // Alignment parameters
  object_name = master->object_names.at(0);
  gate_zcenter_offset = master->tasks["tasks"][master->task_id]["gate_zcenter_offset"].as<double>();
  gate_ycenter_offset = master->tasks["tasks"][master->task_id]["gate_ycenter_offset"].as<double>();
  gate_width = master->tasks["tasks"][master->task_id]["gate_width"].as<double>();
  pass_thru_duration = master->tasks["tasks"][master->task_id]["pass_thru_duration"].as<double>();
  heading_offset = master->tasks["tasks"][master->task_id]["heading_offset"].as<double>();
  black_side = master->black_side;
  ROS_INFO("CasinoGate: Loaded variables from tasks yaml");
  ROS_INFO("CasinoGate: Black side (0->left, 1->right): %i", black_side);

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = object_name; // GFlap
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)master->frame_width * gate_width;
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0; //(int)(master->frame_height * gate_zcenter_offset);
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("CasinoGate: Alignment command published (but disabled)");

  braked = false;
  passed_thru_gate = false;
  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &CasinoGate::IDCasinoGate, this);
  ROS_INFO("CasinoGate: Looking for %s. Subscribed to /task/bboxes", object_name.c_str());
}

// ID the CasinoGate task and set update target y-pos based on the detected object(s)
// TODO: Add a timeout? Maybe not since this has to be completed before we attempt anything else
void CasinoGate::IDCasinoGate(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  if (detectionValidator->Validate())
  {
    task_bbox_sub.shutdown();
    detectionValidator->Reset();
    master->tslam->Abort(true);

    // Wait for TSlam to finish braking before proceeding
    timer = master->nh.createTimer(ros::Duration(master->brake_duration), &CasinoGate::EndTSlamTimer, this, true);
    ROS_INFO("CasinoGate: Identified %s. Awaiting TSlam to end.", object_name.c_str());
  }
}

// Put rest of IDCasinoGate code here
void CasinoGate::EndTSlamTimer(const ros::TimerEvent &event)
{
  // Align gate to correct y-position
  align_cmd.sway_active = true;
  align_cmd.surge_active = true; // Do BOTH surge and sway
  master->alignment_pub.publish(align_cmd);
  alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &CasinoGate::CenterAlignmentStatusCB, this);
  ROS_INFO("CasinoGate: Starting y-alignment. Checking sway/surge error");
}

// Make sure the color is aligned correctly (centered AND correct bbox width)
void CasinoGate::CenterAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (yValidator->Validate(status_msg->y.error) && xValidator->Validate(status_msg->x.error))
  {
    yValidator->Reset();
    xValidator->Reset();
    alignment_status_sub.shutdown();

    // Add bbox code here
    // Stop aligning on bbox
    align_cmd.surge_active = false;
    align_cmd.sway_active = false;
    master->alignment_pub.publish(align_cmd);

    // Set new attitude
    if (black_side == rc::LEFT) // ADD offset if black is on left
      pass_thru_heading = master->euler_rpy.z + heading_offset; // ADD
    else // SUBTRACT offset if black is on right
      pass_thru_heading = master->euler_rpy.z - heading_offset; // SUBTRACT
    
    pass_thru_heading = master->tslam->KeepHeadingInRange(pass_thru_heading);
    attitude_cmd.roll_active = true;
    attitude_cmd.pitch_active = true;
    attitude_cmd.yaw_active = true;
    attitude_cmd.euler_rpy.x = 0;
    attitude_cmd.euler_rpy.y = 0;
    attitude_cmd.euler_rpy.z = pass_thru_heading;
    master->attitude_pub.publish(attitude_cmd);
    ROS_INFO("CasinoGate: Published pass thru heading. Checking heading error.");

    double pass_thru_depth = master->search_depth + depth_offset;
    depth_cmd.active = true;
    depth_cmd.depth = pass_thru_depth;
    master->depth_pub.publish(depth_cmd);
    ROS_INFO("CasinoGate: pass thru depth: %f", pass_thru_depth);

    attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &CasinoGate::YawAttitudeStatusCB, this);
    ROS_INFO("CasinoGate: Published new depth. Will check after heading is good.");

    /*// Activate X alignment controller
    align_cmd.surge_active = true;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &CasinoGate::BBoxAlignmentStatusCB, this);
    ROS_INFO("CasinoGate: %s in center of frame. Aligning on bbox width", object_name.c_str());*/
  }
}

// Set the bbox to desired width
void CasinoGate::BBoxAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  ROS_INFO("CasinoGate: GFlap bbox error: %f", status_msg->x.error);
  if (xValidator->Validate(status_msg->x.error))
  {
    xValidator->Reset();
    alignment_status_sub.shutdown();

    // Stop aligning on bbox
    align_cmd.surge_active = false;
    align_cmd.sway_active = false;
    master->alignment_pub.publish(align_cmd);

    // Set new attitude
    if (black_side == rc::LEFT) // ADD offset if black is on left
      pass_thru_heading = master->euler_rpy.z + heading_offset; // ADD
    else // SUBTRACT offset if black is on right
      pass_thru_heading = master->euler_rpy.z - heading_offset; // SUBTRACT
    
    pass_thru_heading = master->tslam->KeepHeadingInRange(pass_thru_heading);
    attitude_cmd.roll_active = true;
    attitude_cmd.pitch_active = true;
    attitude_cmd.yaw_active = true;
    attitude_cmd.euler_rpy.x = 0;
    attitude_cmd.euler_rpy.y = 0;
    attitude_cmd.euler_rpy.z = pass_thru_heading;
    master->attitude_pub.publish(attitude_cmd);
    ROS_INFO("CasinoGate: Published pass thru heading. Checking heading error.");

    double pass_thru_depth = master->search_depth + depth_offset;
    depth_cmd.active = true;
    depth_cmd.depth = pass_thru_depth;
    master->depth_pub.publish(depth_cmd);
    ROS_INFO("CasinoGate: pass thru depth: %f", pass_thru_depth);

    attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &CasinoGate::YawAttitudeStatusCB, this);
    ROS_INFO("CasinoGate: Published new depth. Will check after heading is good.");
  }
}

void CasinoGate::YawAttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
{
  if (yawValidator->Validate(status_msg->yaw.error))
  {
    yawValidator->Reset();
    attitude_status_sub.shutdown();

    depth_status_sub = master->nh.subscribe<riptide_msgs::ControlStatus>("/status/controls/depth", 1, &CasinoGate::DepthStatusCB, this);
    ROS_INFO("CasinoGate: Heading good. Now checking depth.");
  }
}

void CasinoGate::DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr &status_msg)
{
  if (depthValidator->Validate(status_msg->error))
  {
    depthValidator->Reset();
    depth_status_sub.shutdown();

    std_msgs::Float64 accel_cmd;
    accel_cmd.data = master->search_accel;
    master->x_accel_pub.publish(accel_cmd);

    timer = master->nh.createTimer(ros::Duration(pass_thru_duration), &CasinoGate::PassThruTimer, this, true);
    ROS_INFO("CasinoGate: Fully aligned. Don't move gate, I'm coming for ya. Abort timer initiated. ETA: %f", pass_thru_duration);
  }
}

void CasinoGate::PassThruTimer(const ros::TimerEvent &event)
{
  std_msgs::Float64 msg;
  if (!passed_thru_gate)
  {
    passed_thru_gate = true;
    msg.data = -(master->search_accel);
    master->x_accel_pub.publish(msg);
    timer = master->nh.createTimer(ros::Duration(master->brake_duration), &CasinoGate::PassThruTimer, this, true);
    ROS_INFO("CasinoGate: Passed thru gate...I think. Timer set for braking.");
  }
  else if (passed_thru_gate && !braked)
  {
    msg.data = 0;
    braked = true;
    master->x_accel_pub.publish(msg);
    ROS_INFO("CasinoGate: Completed. Thruster brake applied.");
    master->tslam->SetEndPos();
    CasinoGate::Abort();
    master->LaunchTSlam();
  }
}

// Shutdown all active subscribers
void CasinoGate::Abort()
{
  CasinoGate::Initialize();
  timer.stop();
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("CasinoGate: Aborting");
}
