#include "riptide_autonomy/roulette.h"

#define ALIGN_CENTER 0
#define ALIGN_BBOX_HEIGHT 1
#define ALIGN_OFFSET 2

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
  detections = 0;
  attempts = 0;
  num_markers_dropped = 0;
  align_id = ALIGN_CENTER;

  detection_duration = 0;
  error_duration = 0;
  drop_duration = 0;
  drop_duration_thresh = 0;
  clock_is_ticking = false;
  drop_clock_is_ticking = false;

  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void Roulette::Start()
{
  drop_duration_thresh = master->tasks["tasks"][master->task_id]["drop_duration_thresh"].as<double>();
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = master->object_names.at(0); // Roulette
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)(master->frame_height * 0.7);
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
  detections++;
  if (ValidateDetections(&detections, &detection_duration, master->detections_req, master->detection_duration_thresh, &detect_start, &attempts))
  {
    task_bbox_sub.shutdown();
    master->tslam->Abort(true);

    // Send alignment command to put in center of frame (activate controllers)
    // Set points already specified in initial alignment command
    align_cmd.surge_active = true;
    align_cmd.sway_active = true;
    align_cmd.heave_active = false;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Roulette::AlignmentStatusCB, this);
    ROS_INFO("Roulette: Identified roulette. Now aligning to center");
  }
  else
  {
    ROS_INFO("Roulette: %i Attemps - %i detections in %f sec", attempts, detections, detection_duration);
  }
}

// Align roulette in center of camera frame
// Find depth for optimal bbox height
// Align roulette offcenter to drop markers
void Roulette::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (align_id == ALIGN_CENTER)
  { // Align roulette in center of camera frame
    if (ValidateError(status_msg->x.error, &error_duration, master->align_thresh, master->error_duration_thresh, &clock_is_ticking, &error_check_start))
    {
      align_id = ALIGN_BBOX_HEIGHT;

      // Calculate heading for roulette wheel
      //od->GetRouletteHeading(&Roulette::SetMarkerDropHeading, this);
    }
  }
  else if (align_id == ALIGN_BBOX_HEIGHT) // Find depth for optimal bbox height
  { 
    if (ValidateError(status_msg->z.error, &error_duration, master->bbox_thresh, master->bbox_heave_duration_thresh, &clock_is_ticking, &error_check_start))
    {
      alignment_status_sub.shutdown();
      align_cmd.heave_active = false; // Disabling heave will lock in current depth
      master->alignment_pub.publish(align_cmd);

      // Calculate heading for roulette wheel
      od->GetRouletteHeading(&Roulette::SetMarkerDropHeading, this);
    }
  }
  else if (align_id == ALIGN_OFFSET)
  { // Align roulette offcenter to drop markers
    if (ValidateError2(status_msg->x.error, status_msg->y.error, &error_duration, master->align_thresh, master->error_duration_thresh, &clock_is_ticking, &error_check_start))
    {
      if (num_markers_dropped < 2)
      {
        pneumatics_cmd.header.stamp = ros::Time::now();
        pneumatics_cmd.torpedo_stbd = false;
        pneumatics_cmd.torpedo_port = false;
        pneumatics_cmd.markerdropper = true;
        pneumatics_cmd.manipulator = false;
        pneumatics_cmd.duration = 300; // [ms]

        if (!drop_clock_is_ticking || drop_duration > drop_duration_thresh)
        {
          drop_time = ros::Time::now();
          drop_clock_is_ticking = true;
          master->pneumatics_pub.publish(pneumatics_cmd);
          num_markers_dropped++;
        }
        else
        {
          drop_duration = ros::Time::now().toSec() - drop_time.toSec();
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
}

// When the green_heading has been found, set the robot to a heading normal to the green section
void Roulette::SetMarkerDropHeading(double heading)
{
  ROS_INFO("Roulette angle in camera frame: %f deg.", heading);

  double offset = 0;
  if (heading <= 90)
    green_heading = heading + 90;
  else if (heading > 90)
    green_heading = heading - 90;

  offset = green_heading - 90;

  marker_drop_heading = master->euler_rpy.z + offset; // Center about current heading
  marker_drop_heading = master->tslam->KeepHeadingInRange(marker_drop_heading);

  ROS_INFO("Roulette: Marker Drop Heading is %f deg.", marker_drop_heading);

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
  if (ValidateError(status_msg->yaw.error, &error_duration, master->yaw_thresh, master->error_duration_thresh, &clock_is_ticking, &error_check_start))
  {
    attitude_status_sub.shutdown();

    // Now align to a bit left of the roulette center
    align_cmd.target_pos.x = 0;
    align_cmd.target_pos.y = -(master->frame_width / 6);
    align_cmd.target_pos.z = 0;
    master->alignment_pub.publish(align_cmd);
    align_id = ALIGN_OFFSET;
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &Roulette::AlignmentStatusCB, this);
    ROS_INFO("Roulette: At marker drop heading. Now aligning off-center");
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
