#include "riptide_autonomy/tslam.h"

#define PI 3.141592653
#define VALIDATE_PITCH 0
#define VALIDATE_YAW 1

// Slope and y-int for Maelstrom x-accel to x-velocity
#define A2V_SLOPE 0.2546
#define A2V_INT .1090

TSlam::TSlam(BeAutonomous *master)
{
  this->master = master;

  ROS_INFO("comp id: %i", master->competition_id);
  if (master->competition_id == rc::COMPETITION_SEMIS)
    task_map_file = rc::FILE_MAP_SEMIS;
  else
    task_map_file = rc::FILE_MAP_FINALS;

  task_map = YAML::LoadFile(task_map_file);

  TSlam::Initialize();
}

void TSlam::Initialize()
{
  error_duration = 0;
  delta_x = 0;
  delta_y = 0;
  angle = 0;
  search_heading = 0;

  start_x = 0;
  start_y = 0;
  eta = 0;
  x_vel = 0;
  clock_is_ticking = false;
  validate_id = VALIDATE_PITCH;

  for (int i = 0; i < sizeof(active_subs) / sizeof(active_subs[0]); i++)
    active_subs[i]->shutdown();
}

void TSlam::ReadMap()
{
  quadrant = floor(master->load_id / 2.0);

  current_x = task_map["task_map"][quadrant]["dock_x"].as<double>();
  current_y = task_map["task_map"][quadrant]["dock_y"].as<double>();

  ROS_INFO("Quadrant: %i", quadrant);
  if (quadrant < 4)
  {
    start_x = task_map["task_map"][quadrant]["map"][master->task_id]["start_x"].as<double>();
    start_y = task_map["task_map"][quadrant]["map"][master->task_id]["start_y"].as<double>();

    if (master->run_single_task && master->task_order.size() == 1)
    {
      current_x = start_x + master->relative_current_x;
      current_y = start_y + master->relative_current_y;
    }

    if (master->last_task_id == rc::TASK_CASINO_GATE)
    { // Calculate ending pos on other side of the gate
      double gate_heading = master->casino_gate->gate_heading;
      bool passing_on_left = master->casino_gate->passing_on_left;
      bool passing_on_right = master->casino_gate->passing_on_right;
      double alpha = 0;
      double end_pos_offset = master->casino_gate->end_pos_offset;

      if (passing_on_left)
        alpha = gate_heading + 90;
      else if (passing_on_right)
        alpha = gate_heading - 90;

      current_x = current_x - end_pos_offset * sin(alpha * PI / 180);
      current_y = current_y + end_pos_offset * cos(alpha * PI / 180);
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
  ReadMap();

  // Calculate heading to point towards next task
  delta_x = start_x - current_x;
  delta_y = start_y - current_y;
  angle = atan2(delta_y, delta_x) * 180 / PI;
  double offset = angle - 90;
  search_heading = master->global_y_axis_heading + offset; // Center about global_y_axis_heading
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
  attitude_cmd.yaw_active = false; // Don't go to heading yet
  attitude_cmd.euler_rpy.x = 0;
  attitude_cmd.euler_rpy.y = 0;
  attitude_cmd.euler_rpy.z = search_heading;
  master->attitude_pub.publish(attitude_cmd);
  ROS_INFO("TSlam: Published attitude cmd");

  attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::AttitudeStatusCB, this);

  ROS_INFO("TSlam: Checking pitch error");
}

void TSlam::AttitudeStatusCB(const riptide_msgs::ControlStatusAngular::ConstPtr &status_msg)
{
  if (validate_id == VALIDATE_PITCH)
  { // Must first validate pitch so vehicle can submerge properly
    if (abs(status_msg->pitch.error) < master->pitch_thresh)
    {
      if (!clock_is_ticking)
      {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        error_duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if (error_duration >= master->error_duration_thresh)
      {
        attitude_status_sub.shutdown();
        error_duration = 0;
        clock_is_ticking = false;

        // Publish depth command
        depth_cmd.active = true;
        depth_cmd.depth = master->search_depth;
        master->depth_pub.publish(depth_cmd);
        ROS_INFO("TSlam: Pitch good. Published depth cmd");
        depth_status_sub = master->nh.subscribe<riptide_msgs::ControlStatus>("/status/controls/depth", 1, &TSlam::DepthStatusCB, this);
      }
    }
    else
    {
      error_duration = 0;
      clock_is_ticking = false;
    }
  }
  else if (validate_id == VALIDATE_YAW)
  { // Validate heading after depth is reached
    if (abs(status_msg->yaw.error) < master->yaw_thresh)
    {
      if (!clock_is_ticking)
      {
        acceptable_begin = ros::Time::now();
        clock_is_ticking = true;
      }
      else
        error_duration = ros::Time::now().toSec() - acceptable_begin.toSec();

      if (error_duration >= master->error_duration_thresh)
      {
        attitude_status_sub.shutdown();
        error_duration = 0;
        clock_is_ticking = false;

        // Drive forward
        geometry_msgs::Vector3 msg;
        msg.x = master->search_accel;
        msg.y = 0;
        msg.z = 0;
        master->linear_accel_pub.publish(msg);

        double tslam_duration = 1.5 * eta;
        timer = master->nh.createTimer(ros::Duration(tslam_duration), &TSlam::AbortTSlamTimer, this, true);
        ROS_INFO("TSlam: Reached heading, now moving forward. Abort timer initiated. ETA: %f", tslam_duration);
      }
    }
    else
    {
      error_duration = 0;
      clock_is_ticking = false;
    }
  }
}

void TSlam::DepthStatusCB(const riptide_msgs::ControlStatus::ConstPtr &status_msg)
{
  if (abs(status_msg->error) < master->depth_thresh)
  {
    if (!clock_is_ticking)
    {
      acceptable_begin = ros::Time::now();
      clock_is_ticking = true;
    }
    else
      error_duration = ros::Time::now().toSec() - acceptable_begin.toSec();

    if (error_duration >= master->error_duration_thresh)
    {
      depth_status_sub.shutdown();
      error_duration = 0;
      clock_is_ticking = false;
      validate_id = VALIDATE_YAW;

      attitude_cmd.yaw_active = true;
      master->attitude_pub.publish(attitude_cmd);
      attitude_status_sub = master->nh.subscribe<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1, &TSlam::AttitudeStatusCB, this);
      ROS_INFO("TSlam: Reached search depth, now checking heading error");
    }
  }
  else
  {
    error_duration = 0;
    clock_is_ticking = false;
  }
}

void TSlam::AbortTSlamTimer(const ros::TimerEvent &event)
{
  TSlam::Abort(true);
}

void TSlam::BrakeTimer(const ros::TimerEvent &event)
{
  geometry_msgs::Vector3 msg;
  msg.x = 0;
  msg.y = 0;
  msg.z = 0;
  master->linear_accel_pub.publish(msg);
  ROS_INFO("TSlam: Thruster brake applied");
}

// Shutdown all active subscribers
void TSlam::Abort(bool apply_brake)
{
  TSlam::Initialize();

  timer.stop();
  geometry_msgs::Vector3 msg;
  if (apply_brake)
  {
    msg.x = -(master->search_accel);
    msg.y = 0;
    msg.z = 0;
    master->linear_accel_pub.publish(msg);
    timer = master->nh.createTimer(ros::Duration(0.25), &TSlam::BrakeTimer, this, true);
    ROS_INFO("TSlam: Aborting. Braking now.");
  }
  else
  {
    msg.x = 0;
    msg.y = 0;
    msg.z = 0;
    master->linear_accel_pub.publish(msg);
  }
}
