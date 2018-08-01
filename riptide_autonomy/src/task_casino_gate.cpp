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
  detectionBlackValidator = new DetectionValidator(master->detections_req, master->detection_duration);
  xValidator = new ErrorValidator(master->bbox_thresh, master->bbox_surge_duration);
  yValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  zValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  yawValidator = new ErrorValidator(master->yaw_thresh, master->error_duration);

  // Alignment parameters
  object_name = master->object_names.at(0);
  gate_zcenter_offset = master->tasks["tasks"][master->task_id]["gate_zcenter_offset"].as<double>();
  gate_width = master->tasks["tasks"][master->task_id]["gate_width"].as<double>();
  pass_thru_duration = master->tasks["tasks"][master->task_id]["pass_thru_duration"].as<double>();
  ROS_INFO("CasinoGate: Loaded variables from tasks yaml");

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = object_name; // Casino_Gate Black
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)master->frame_width * gate_width;
  align_cmd.bbox_control = rc::CONTROL_BBOX_WIDTH;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = (int)(master->frame_height * gate_zcenter_offset);
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("CasinoGate: Alignment command published (but disabled)");

  braked = false;
  passed_thru_gate = false;
  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &CasinoGate::IDCasinoGate, this);
  ROS_INFO("CasinoGate: Only looking for %s. Subscribed to /task/bboxes", object_name.c_str());
}

// ID the CasinoGate task and set update target y-pos based on the detected object(s)
// TODO: Add a timeout? Maybe not since this has to be completed before we attempt anything else
void CasinoGate::IDCasinoGate(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Figure out which side(s) of the gate we can detect initially
  for (int i = 0; i < bbox_msg->bounding_boxes.size(); i++)
  {
    if (bbox_msg->bounding_boxes.at(i).Class == "Casino_Gate_Black")
    {
      if (detectionBlackValidator->Validate())
      {
        task_bbox_sub.shutdown();
        detectionBlackValidator->Reset();
        master->tslam->Abort(true);

        // Wait for TSlam to finish braking before proceeding
        timer = master->nh.createTimer(ros::Duration(master->brake_duration), &CasinoGate::EndTSlamTimer, this, true);
        ROS_INFO("CasinoGate: Identified %s. Awaiting TSlam to end.", object_name.c_str());
      }
    }
  }
}

// Put rest of IDCasinoGate code here
void CasinoGate::EndTSlamTimer(const ros::TimerEvent &event)
{
  // First align gate in center of camera frame
  align_cmd.surge_active = false;
  align_cmd.sway_active = true;
  align_cmd.heave_active = true;
  master->alignment_pub.publish(align_cmd);
  alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &CasinoGate::PositionAlignmentStatusCB, this);
  ROS_INFO("CasinoGate: Starting alignment. Checking sway/heave error");
}

// Make sure the color is aligned on the correct side of the camera frame
void CasinoGate::PositionAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (yValidator->Validate(status_msg->y.error) && zValidator->Validate(status_msg->z.error))
  {
    yValidator->Reset();
    zValidator->Reset();
    alignment_status_sub.shutdown();

    // Activate X alignment controller
    align_cmd.surge_active = true;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &CasinoGate::BBoxAlignmentStatusCB, this);
    ROS_INFO("CasinoGate: %s in center of frame. Aligning on bbox width", object_name.c_str());
  }
}

// Set the bbox to desired width
void CasinoGate::BBoxAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (xValidator->Validate(status_msg->x.error))
  {
    xValidator->Reset();
    alignment_status_sub.shutdown();

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
    master->StartTask();
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
