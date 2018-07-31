#include "riptide_autonomy/tslam.h"

#define PI 3.141592653

// Slope and y-int for Maelstrom x-accel to x-velocity
#define A2V_SLOPE 0.2546
#define A2V_INT .1090

/* TSlam - Order of Execution:
1. Read map, calculate distances and angles.
2. Set attitude cmd (only enable roll/pitch).
3. Check pitch (roll is probably fine).
4. Set search depth. Check search depth.
5. Set search heading. Check heading.
6. Move forward (initiate timer in case).
7. Abort (brake if required).
*/

// TODO: Configure TSlam to take us back to a previous location or to a different part of the path
// TODO: Add move forward, backwards, left, and right functionality for moving around other objects

TSlam::TSlam(BeAutonomous *master)
{
  this->master = master;

  
  if (master->competition_id == rc::COMPETITION_SEMIS)
    task_map_file = rc::FILE_MAP_SEMIS;
  else
    task_map_file = rc::FILE_MAP_FINALS;

  task_map = YAML::LoadFile(task_map_file);
  ROS_INFO("TSlam: Loaded map");

  TSlam::Initialize();
  ROS_INFO("TSlam: Initialized");
}

void TSlam::Initialize()
{
  delta_x = 0;
  delta_y = 0;
  angle = 0;
  search_heading = 0;

  start_x = 0;
  start_y = 0;
  eta = 0;
  x_vel = 0;

  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void TSlam::ReadMap()
{
  quadrant = floor(master->load_id / 2.0);
  ROS_INFO("TSlam: Quadrant %i", quadrant);

  if (current_x == 420)
  {
    current_x = task_map["task_map"]["dock_x"][quadrant].as<double>();
    current_y = task_map["task_map"]["dock_y"][quadrant].as<double>();
  }

  ROS_INFO("Quadrant: %i", quadrant);
  if (quadrant < 4)
  {
    start_x = task_map["task_map"]["map"][master->task_id]["start_x"][quadrant].as<double>();
    start_y = task_map["task_map"]["map"][master->task_id]["start_y"][quadrant].as<double>();

    end_x = task_map["task_map"]["map"][master->task_id]["end_x"][quadrant].as<double>();
    end_y = task_map["task_map"]["map"][master->task_id]["end_y"][quadrant].as<double>();

    if (master->run_single_task && master->task_order.size() == 1)
    {
      current_x = start_x + master->relative_current_x;
      current_y = start_y + master->relative_current_y;
    }
  }
  else
  {
    current_x = 420;
    current_y = 420;
    start_x = 420;
    start_y = 420;
  }

  // Verify current_x and current_y were not set to an arbitrarily large number
  if (abs(current_x) > 55)
  {
    current_x = 0;
    ROS_INFO("ERROR: Current X not initialized properly, exceeded limit. Setting to 0.");
  }
  if (abs(current_y) > 40)
  {
    current_y = 0;
    ROS_INFO("ERROR: Current Y not initialized properly, exceeded limit. Setting to 0.");
  }

  ROS_INFO("Next Task (%s) Start X: %f", master->task_name.c_str(), start_x);
  ROS_INFO("Next Task (%s) Start Y: %f", master->task_name.c_str(), start_y);
}

void TSlam::SetPos(double x, double y)
{
  current_x = x;
  current_y = y;
}

void TSlam::SetEndPos()
{
  current_x = end_x;
  current_y = end_y;
}

void TSlam::EndMission()
{
  current_x = 420;
  current_y = 420;
}

// Calculate eta for TSlam to bring vehicle to next task
void TSlam::CalcETA(double Ax, double dist)
{
  if (Ax >= 0.6 && Ax <= 1.25)
  { // Eqn only valid for Ax=[0.6, 1.25] m/s^2
    x_vel = A2V_SLOPE * Ax + A2V_INT;
    eta = dist / x_vel;
  }
  else
    eta = 0;
}

// Keeps heading in the range determined by the IMU [180, -180]
double TSlam::KeepHeadingInRange(double input)
{
  if (input > 180)
    return (input - 360.0);
  else if (input < -180)
    return (input + 360.0);
  else
    return input;
}

void TSlam::Start()
{
  pitchValidator = new ErrorValidator(master->pitch_thresh, master->error_duration);
  yawValidator = new ErrorValidator(master->yaw_thresh, master->error_duration);
  depthValidator = new ErrorValidator(master->depth_thresh, master->error_duration);
  
  quadrant = floor(master->load_id / 2.0);
  ROS_INFO("TSlam: Quadrant %i", quadrant);

  user_defined_y_axis_heading = task_map["task_map"]["user_defined_y_axis_heading"][quadrant].as<double>();
  brake_duration = master->brake_duration;
  ROS_INFO("TSlam: Loaded a few variables from task_map");

  // Calculate heading to point towards next task
  TSlam::ReadMap();
  delta_x = start_x - current_x;
  delta_y = start_y - current_y;
  angle = atan2(delta_y, delta_x) * 180 / PI;
  double offset = angle - 90;
  search_heading = user_defined_y_axis_heading + offset; // Center about global_y_axis_heading
  search_heading = TSlam::KeepHeadingInRange(search_heading);

  ROS_INFO("Cur X: %f", current_x);
  ROS_INFO("Cur Y: %f", current_y);
  ROS_INFO("TSlam: Vehicle search heading: %f", search_heading);

  // Calculate distance and ETA
  distance = sqrt(delta_x * delta_x + delta_y * delta_y);
  TSlam::CalcETA(master->search_accel, distance);
  ROS_INFO("TSlam: Distance %f with eta of %f sec at %f m/s", distance, eta, x_vel);

  // Publish attitude command
  attitude_cmd.roll_active = true;
  attitude_cmd.pitch_active = true;
  attitude_cmd.yaw_active = false; // Rotate to heading once search depth is reached
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;
  attitude_cmd.euler_rpy.z = search_heading;
  master->attitude_pub.publish(attitude_cmd);

  attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::PitchAttitudeStatusCB, this);
  ROS_INFO("TSlam: Checking pitch error");
}

// Must first validate pitch so vehicle can submerge properly
void TSlam::PitchAttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
{
  if (pitchValidator->Validate(status_msg->pitch.error))
  {
    pitchValidator->Reset();
    attitude_status_sub.shutdown();

    // Publish depth command
    depth_cmd.active = true;
    depth_cmd.depth = master->search_depth;
    master->depth_pub.publish(depth_cmd);
    depth_status_sub = master->nh.subscribe<riptide_msgs::ControlStatus>("/status/controls/depth", 1, &TSlam::DepthStatusCB, this);
    ROS_INFO("TSlam: Pitch good. Now checking depth.");
  }
}

void TSlam::DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr &status_msg)
{
  if (depthValidator->Validate(status_msg->error))
  {
    depthValidator->Reset();
    depth_status_sub.shutdown();

    attitude_cmd.yaw_active = true;
    master->attitude_pub.publish(attitude_cmd);
    attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::YawAttitudeStatusCB, this);
    ROS_INFO("TSlam: Reached depth, now checking heading.");
  }
}

// Validate heading after depth is reached
void TSlam::YawAttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
{
  if (yawValidator->Validate(status_msg->yaw.error))
  {
    yawValidator->Reset();
    attitude_status_sub.shutdown();

    // Drive forward
    std_msgs::Float64 msg;
    msg.data = master->search_accel;
    master->x_accel_pub.publish(msg);

    double tslam_duration = 1.2 * eta;
    timer = master->nh.createTimer(ros::Duration(tslam_duration), &TSlam::AbortTSlamTimer, this, true);
    ROS_INFO("TSlam: Reached heading, now moving forward. Abort timer initiated. ETA: %f", tslam_duration);
  }
}

void TSlam::AbortTSlamTimer(const ros::TimerEvent &event)
{
  TSlam::Abort(true);
}

void TSlam::BrakeTimer(const ros::TimerEvent &event)
{
  std_msgs::Float64 msg;
  msg.data = 0;
  master->x_accel_pub.publish(msg);
  ROS_INFO("TSlam: Thruster brake applied");
}

// Shutdown all active subscribers
void TSlam::Abort(bool apply_brake)
{
  TSlam::Initialize();

  timer.stop();
  std_msgs::Float64 msg;
  if (apply_brake)
  {
    msg.data = -(master->search_accel);
    master->x_accel_pub.publish(msg);
    timer = master->nh.createTimer(ros::Duration(brake_duration), &TSlam::BrakeTimer, this, true);
    ROS_INFO("TSlam: Aborting. Braking now.");
  }
  else
  {
    msg.data = 0;
    master->x_accel_pub.publish(msg);
  }
}
