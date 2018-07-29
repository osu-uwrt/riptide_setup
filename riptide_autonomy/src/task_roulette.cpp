#include "riptide_autonomy/task_roulette.h"

/* Roulette - Order of Execution:
1. Start. Set alignment command.
2. ID roulette.
3. Align to center.
4. Find depth at which bbox is of good size. Lock in current depth.
5. Get green heading.
6. Calculate marker drop heading, set heading, then check.
7. Align to offcenter
8. Drop two markers with short duration between.
9. Abort. Start next task.
*/

Roulette::Roulette(BeAutonomous *master)
{
  this->master = master;
  od = new ObjectDescriber(master);
  Roulette::Initialize();
  ROS_INFO("Roulette: Initialized");
}

void Roulette::Initialize()
{
  num_markers_dropped = 0;
  drop_time = 0;
  drop_duration = 0;
  drop_clock_is_ticking = false;

  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void Roulette::Start()
{
  drop_duration = master->tasks["tasks"][master->task_id]["drop_duration"].as<double>();

  detectionValidator = new DetectionValidator(master->detections_req, master->detection_duration);
  xValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  yValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  zValidator = new ErrorValidator(master->bbox_thresh, master->bbox_heave_duration);
  yawValidator = new ErrorValidator(master->yaw_thresh, master->error_duration);

  roulette_bbox_height = master->tasks["tasks"][master->task_id]["roulette_bbox_height"].as<double>();
  roulette_ycenter_offset = master->tasks["tasks"][master->task_id]["roulette_ycenter_offset"].as<double>();

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = master->object_names.at(0); // Roulette
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)(master->frame_height * roulette_bbox_height);
  align_cmd.bbox_control = rc::CONTROL_BBOX_HEIGHT;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Roulette: alignment command published (but disabled)");

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &Roulette::IDRoulette, this);
  ROS_INFO("Roulette: subscribed to /task/bboxes");
}

// ID the roulette task
// TODO: Add a timeout in case the vehicle did not pass over it well enough
// TODO: Configure TSlam to take us back to a previous location or to a different part of the path
void Roulette::IDRoulette(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  // Get number of objects and make sure you have 'x' many within 't' seconds
  // Simply entering this callback signifies the object was detected (unless it was a false-positive)

  if (detectionValidator->Validate())
  {
    task_bbox_sub.shutdown();
    master->tslam->Abort(false);

    // Send alignment command to put in center of frame (activate controllers)
    // Set points already specified in initial alignment command
    align_cmd.surge_active = true;
    align_cmd.sway_active = true;
    align_cmd.heave_active = false;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Roulette::CenterAlignmentStatusCB, this);
    ROS_INFO("Roulette: Identified roulette. Now aligning to center");
  }
}

// Make sure the vehicle is aligned to the center of the roulette wheel
void Roulette::CenterAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (xValidator->Validate(status_msg->x.error) && yValidator->Validate(status_msg->y.error))
  {
    xValidator->Reset();
    yValidator->Reset();
    alignment_status_sub.shutdown();
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Roulette::BBoxAlignmentStatusCB, this);
  }
}

// Align to bbox height of roulette wheel
void Roulette::BBoxAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (zValidator->Validate(status_msg->z.error))
  {
    zValidator->Reset();
    alignment_status_sub.shutdown();

    // Lock in current depth
    align_cmd.heave_active = false;
    master->alignment_pub.publish(align_cmd);

    // Calculate heading for roulette wheel
    od->GetRouletteHeading(&Roulette::SetMarkerDropHeading, this);
    ROS_INFO("ROulette: Locking depth. Now finding green heading.");
  }
}

// When the green_heading has been found, set the robot to a heading normal to the green section
void Roulette::SetMarkerDropHeading(double heading)
{
  ROS_INFO("Roulette angle in camera frame: %f", heading);

  double offset = 0;
  if (heading <= 90)
    green_heading = heading + 90;
  else if (heading > 90)
    green_heading = heading - 90;

  offset = green_heading - 90;

  marker_drop_heading = master->euler_rpy.z + offset; // Center about current heading
  marker_drop_heading = master->tslam->KeepHeadingInRange(marker_drop_heading);

  ROS_INFO("Roulette: Marker Drop Heading is %f deg", marker_drop_heading);

  // Publish attitude command
  attitude_cmd.roll_active = true;
  attitude_cmd.pitch_active = true;
  attitude_cmd.yaw_active = true;
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;
  attitude_cmd.euler_rpy.z = marker_drop_heading;
  master->attitude_pub.publish(attitude_cmd);

  attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &Roulette::AttitudeStatusCB, this);
  ROS_INFO("Roulette: Published marker drop attitude. Checking heading.");
}

// Make sure the robot goes to the marker drop heading
void Roulette::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
{
  // Depth is good, now verify heading error

  if (yawValidator->Validate(status_msg->yaw.error))
  {
    attitude_status_sub.shutdown();
    yawValidator->Reset();

    // Now align to a bit left of the roulette center
    align_cmd.target_pos.x = 0;
    align_cmd.target_pos.y = -(int)(master->frame_width * roulette_ycenter_offset);
    align_cmd.target_pos.z = 0;
    master->alignment_pub.publish(align_cmd);
    //align_id = ALIGN_OFFSET;
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Roulette::OffsetAlignmentStatusCB, this);
    ROS_INFO("Roulette: At marker drop heading. Now aligning off-center");
  }
}

void Roulette::OffsetAlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (xValidator->Validate(status_msg->x.error) && yValidator->Validate(status_msg->y.error))
  {
    xValidator->Reset();
    yValidator->Reset();
    if (num_markers_dropped < 2)
    {
      pneumatics_cmd.header.stamp = ros::Time::now();
      pneumatics_cmd.torpedo_stbd = false;
      pneumatics_cmd.torpedo_port = false;
      pneumatics_cmd.markerdropper = true;
      pneumatics_cmd.manipulator = false;
      pneumatics_cmd.duration = 300; // [ms]

      if (!drop_clock_is_ticking || drop_time > drop_duration)
      {
        drop_start_time = ros::Time::now();
        drop_clock_is_ticking = true;
        master->pneumatics_pub.publish(pneumatics_cmd);
        num_markers_dropped++;
        ROS_INFO("Roulette: Dropped it like it's hot");
      }
      else
      {
        drop_time = ros::Time::now().toSec() - drop_start_time.toSec();
      }
    }
    else
    {
      pneumatics_cmd.markerdropper = false;
      master->pneumatics_pub.publish(pneumatics_cmd);
      ROS_INFO("Roulette is DONE!!!");
      master->tslam->SetEndPos();
      Roulette::Abort();
      master->StartTask();
    }
  }
  else
  {
    drop_clock_is_ticking = false;
  }
}

// Shutdown all active subscribers
void Roulette::Abort()
{
  Roulette::Initialize();

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("Roulette: Aborting");
}
