#include "riptide_autonomy/task_path_marker.h"

#define ALIGN_CENTER 0
#define ALIGN_BBOX_WIDTH 1
#define ALIGN_OFFSET 2

//  Path Marker, what it does:
// 1: If we see the Path Marker, abort tslam & get path heading
// 2: Once we have determined the heading start aligning
// 3: Once we are at the right angle, go forward
// 4: Once we are over the center, turn
// 5: Once we have turned, go
// 6: Once we have gone, be done

PathMarker::PathMarker(BeAutonomous *master)
{
  this->master = master;
  od = new ObjectDescriber(master);
  PathMarker::Initialize();
}

void PathMarker::Initialize()
{
  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void PathMarker::Start()
{
  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  align_cmd.object_name = master->object_names.at(0); // PathMarker
  align_cmd.alignment_plane = master->alignment_plane;
  align_cmd.bbox_dim = (int)(master->frame_height * 0.7);
  align_cmd.bbox_control = rc::CONTROL_BBOX_HEIGHT;
  align_cmd.target_pos.x = 0;
  align_cmd.target_pos.y = 0;
  align_cmd.target_pos.z = 0;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("PathMarker: alignment command published (but disabled)");

  task_bbox_sub = master->nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/task/bboxes", 1, &PathMarker::IDPathMarker, this);
  detectionValidator = new DetectionValidator(master->detections_req, master->detection_duration);
  yawValidator = new ErrorValidator(master->yaw_thresh, master->error_duration);
  xValidator = new ErrorValidator(master->align_thresh, master->error_duration);
  yValidator = new ErrorValidator(master->align_thresh, master->error_duration);

  path_angle = master->tasks["tasks"][master->task_id]["path_angle"].as<double>();
  ROS_INFO("PathMarker: Loaded variables from tasks yaml");
}

// If we see the Path Marker, abort tslam & get angle
void PathMarker::IDPathMarker(const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
  if (detectionValidator->Validate())
  {
    detectionValidator->Reset();
    task_bbox_sub.shutdown();
    master->tslam->Abort(true);

    // Wait for TSlam to finish braking before proceeding
    timer = master->nh.createTimer(ros::Duration(master->brake_duration), &PathMarker::EndTSlamTimer, this, true);
    ROS_INFO("Pathmarker; Found path. Awaiting TSlam to end.");

    /*od->GetPathHeading(&PathMarker::GotHeading, this);
    ROS_INFO("Found path, getting heading");*/
  }
}

// Put rest of IDPathMarker code here
void PathMarker::EndTSlamTimer(const ros::TimerEvent &event)
{
  od->GetPathHeading(&PathMarker::GotHeading, this);
  ROS_INFO("PathMarker: Getting heading");
}

// Once we have determined the heading start aligning
void PathMarker::GotHeading(double heading)
{
  ROS_INFO("PathMarker angle: %f", heading);

  attitude_cmd.roll_active = true;
  attitude_cmd.pitch_active = true;
  attitude_cmd.yaw_active = true;
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;

  if (heading < 90 && heading > -90)
  {
    pathDirection = right;
    ROS_INFO("Path Right");
    path_heading = master->euler_rpy.z + (heading - (-90 + path_angle / 2));
  }
  else
  {
    pathDirection = left;
    ROS_INFO("Path Left");
    path_heading = master->euler_rpy.z + (heading - (-90 - path_angle / 2));
  }

  path_heading = master->tslam->KeepHeadingInRange(path_heading);

  attitude_cmd.euler_rpy.z = path_heading;
  master->attitude_pub.publish(attitude_cmd);
  align_cmd.surge_active = false;
  align_cmd.sway_active = true;
  align_cmd.heave_active = false;
  master->alignment_pub.publish(align_cmd);
  attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &PathMarker::FirstAttitudeStatusCB, this);

  ROS_INFO("PathMarker: Identified Heading. Now rotating and aligning along y");
}

// Once we are at the right angle, go forward
void PathMarker::FirstAttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
{

  if (yawValidator->Validate(status_msg->yaw.error))
  {
    attitude_status_sub.shutdown();
    yawValidator->Reset();

    // TODO: If you dont see it, do something

    align_cmd.surge_active = true;
    align_cmd.sway_active = true;
    align_cmd.heave_active = false;
    master->alignment_pub.publish(align_cmd);
    alignment_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1, &PathMarker::AlignmentStatusCB, this);
    ROS_INFO("PathMarker: At heading. Now aligning center");
  }
}

// Once we are over the center, turn
void PathMarker::AlignmentStatusCB(const riptide_msgs::ControlStatusLinear::ConstPtr &status_msg)
{
  if (xValidator->Validate(status_msg->x.error) && yValidator->Validate(status_msg->y.error))
  {
    alignment_status_sub.shutdown();

    if (pathDirection == right)
      attitude_cmd.euler_rpy.z = path_heading - (180 - path_angle);
    else
      attitude_cmd.euler_rpy.z = path_heading + (180 - path_angle);

    attitude_cmd.euler_rpy.z = master->tslam->KeepHeadingInRange(attitude_cmd.euler_rpy.z);
    master->attitude_pub.publish(attitude_cmd);

    xValidator->Reset();
    yValidator->Reset();

    attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &PathMarker::SecondAttitudeStatusCB, this);
    ROS_INFO("Now center, turning to %f", attitude_cmd.euler_rpy.z);
  }
}

// Once we have turned, go
void PathMarker::SecondAttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
{
  if (yawValidator->Validate(status_msg->yaw.error))
  {
    attitude_status_sub.shutdown();
    yawValidator->Reset();

    std_msgs::Float64 msg;
    msg.data = master->search_accel;
    master->x_accel_pub.publish(msg);

    ROS_INFO("HALF SPEED AHEAD!!!");
    timer = master->nh.createTimer(ros::Duration(2), &PathMarker::Success, this, true);
  }
}

// Once we have gone, be done
void PathMarker::Success(const ros::TimerEvent &event)
{
  std_msgs::Float64 msg;
  msg.data = 0;
  master->x_accel_pub.publish(msg);
  Abort();
  master->tslam->SetEndPos();
  master->LaunchTSlam();
}

// Shutdown all active subscribers
void PathMarker::Abort()
{
  PathMarker::Initialize();

  align_cmd.surge_active = false;
  align_cmd.sway_active = false;
  align_cmd.heave_active = false;
  master->alignment_pub.publish(align_cmd);
  ROS_INFO("PathMarker: Aborting");
}
