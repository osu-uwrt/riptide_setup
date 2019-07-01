#include "riptide_controllers/alignment_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653
#define RESET_ID 0
#define DISABLE_ID 1

int main(int argc, char **argv) {
  ros::init(argc, argv, "alignment_controller");
  AlignmentController ac;
  try
  {
    ros::spin();
  }
  catch (exception &e)
  {
    ROS_ERROR("Alignment Error: %s", e.what());
    ROS_ERROR("Alignment: Shutting Down");
  }
}

// Constructor: AlignmentController()
AlignmentController::AlignmentController() : nh("~") {
    ros::NodeHandle surge("surge_controller");
    ros::NodeHandle sway("sway_controller");
    ros::NodeHandle heave("heave_controller");

    x_pid.init(surge, false);
    y_pid.init(sway, false);
    z_pid.init(heave, false);

    object_sub = nh.subscribe<riptide_msgs::Object>("/state/object", 1, &AlignmentController::ObjectCB, this);
    alignment_cmd_sub = nh.subscribe<riptide_msgs::AlignmentCommand>("/command/alignment", 1, &AlignmentController::CommandCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &AlignmentController::DepthCB, this);
    task_info_sub = nh.subscribe<riptide_msgs::TaskInfo>("/task/info", 1, &AlignmentController::TaskInfoCB, this);

    x_pub = nh.advertise<std_msgs::Float64>("/command/force_x", 1);
    y_pub = nh.advertise<std_msgs::Float64>("/command/force_y", 1);
    depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/depth", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1);

    AlignmentController::LoadParam<double>("max_x_error", MAX_X_ERROR);
    AlignmentController::LoadParam<double>("max_y_error", MAX_Y_ERROR);
    AlignmentController::LoadParam<double>("max_z_error", MAX_Z_ERROR);
    AlignmentController::LoadParam<double>("max_bbox_surge_error", MAX_BBOX_SURGE_ERROR);
    AlignmentController::LoadParam<double>("max_bbox_depth_error", MAX_BBOX_DEPTH_ERROR);
    AlignmentController::LoadParam<double>("max_zero_detect_duration", max_zero_detect_duration);
    AlignmentController::LoadParam<double>("PID_IIR_LPF_bandwidth", PID_IIR_LPF_bandwidth);
    AlignmentController::LoadParam<double>("imu_filter_rate", imu_filter_rate);

    pid_surge_active = false;
    pid_sway_active = false;
    pid_heave_active = false;
    pid_alignment_active = false;

    sample_start = ros::Time::now();
    bbox_control = rc::CONTROL_BBOX_WIDTH;
    alignment_plane = rc::PLANE_YZ;
    current_depth = 0;
    AlignmentController::InitMsgs();

    // IIR LPF Variables
    double fc = PID_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
    dt_iir = 1.0/imu_filter_rate;
    alpha = 2*PI*dt_iir*fc / (2*PI*dt_iir*fc + 1); // Multiplier

    AlignmentController::ResetSurge();
    AlignmentController::ResetSway();
    AlignmentController::ResetHeave();

    timer = nh.createTimer(ros::Duration(max_zero_detect_duration/2.0), &AlignmentController::DisableControllerTimer, this, true);
}

void AlignmentController::InitMsgs() {
  status_msg.x.reference = 0;
  status_msg.x.current = 0;
  status_msg.x.error = 0;

  status_msg.y.reference = 0;
  status_msg.y.current = 0;
  status_msg.y.error = 0;

  status_msg.z.reference = 0;
  status_msg.z.current = 0;
  status_msg.z.error = 0;

  obj_pos.x = 0;
  obj_pos.y = 0;
  obj_pos.z = 0;
  obj_bbox_dim = 0;
}

// Load parameter from namespace
template <typename T>
void AlignmentController::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("Alignment Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void AlignmentController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // When controlling on position (NOT bbox dimension), you MUST subtract target pos FROM current pos
  // for the accel direction to be correct

  if(pid_sway_active) {
    error.y = (obj_pos.y - target_pos.y); // MUST subtract target pos from current pos
    status_msg.y.error = error.y;
    error.y = AlignmentController::Constrain(error.y, MAX_Y_ERROR);
    error_dot.y = (error.y - last_error.y) / dt;
    error_dot.y = AlignmentController::SmoothErrorIIR(error_dot.y, last_error_dot.y);
    last_error.y = error.y;
    last_error_dot.y = error_dot.y;

    cmd_force_y.data = y_pid.computeCommand(error.y, error_dot.y, sample_duration);
    cmd_force_y.data = min(max(cmd_force_y.data, -30.0), 30.0);
    y_pub.publish(cmd_force_y);
  }

  // If we are aligning in the YZ plane (forward cam), then X acceleration is
  // dependent on bounding box size (approximation for distance from task), while
  // Z acceleration is determined by offset in the Z axis.
  // If we are aligning in the YX plane (downward cam), then Z acceleration is
  // dependent on boudning box size, while X acceleration is determined by offset
  // in the X axis.
  if(pid_surge_active) {
    error.x = (obj_pos.x - target_pos.x); // MUST subtract target pos from current pos
    status_msg.x.error = error.x;
    error.x = AlignmentController::Constrain(error.x, MAX_X_ERROR);
    error_dot.x = (error.x - last_error.x) / dt;
    error_dot.x = AlignmentController::SmoothErrorIIR(error_dot.x, last_error_dot.x);
    last_error.x = error.x;
    last_error_dot.x = error_dot.x;

    cmd_force_x.data = x_pid.computeCommand(error.x, error_dot.x, sample_duration);
    cmd_force_x.data = min(max(cmd_force_x.data, -30.0), 30.0);
    x_pub.publish(cmd_force_x);
  }

  if(pid_heave_active) {
    error.z = (target_pos.z - obj_pos.z); // DO NOT subtract target pos from current pos. Depth is positive DOWNWARD
    status_msg.z.error = error.z;
    error.z = AlignmentController::Constrain(error.z, MAX_Z_ERROR);
    error_dot.z = (error.z - last_error.z) / dt;
    error_dot.z = AlignmentController::SmoothErrorIIR(error_dot.z, last_error_dot.z);
    last_error.z = error.z;
    last_error_dot.z = error_dot.z;

    cmd_heave = z_pid.computeCommand(error.z, error_dot.z, sample_duration);
    depth_cmd.active = true;
    depth_cmd.depth = current_depth + cmd_heave;
    depth_pub.publish(depth_cmd);
  }

  if(pid_alignment_active) {
    status_msg.header.stamp = sample_start;
    status_pub.publish(status_msg);
  }

  sample_start = ros::Time::now();
}

double AlignmentController::Constrain(double current, double max) {
  if(current > max)
    return max;
  else if(current < -1*max)
    return -1*max;
  return current;
}

// Apply IIR LPF to alignment error_dot
double AlignmentController::SmoothErrorIIR(double input, double prev) {
  return (alpha*input + (1-alpha)*prev);
}

// UpdateError should ONLY be called in this callback b/c this is the only thing
// that signals a new "sensor" input.
void AlignmentController::ObjectCB(const riptide_msgs::Object::ConstPtr &obj_msg) {
  timer.stop();
  obj_pos.x = obj_msg->pos.x;
  obj_pos.y = obj_msg->pos.y;
  obj_pos.z = obj_msg->pos.z;

  // This if-block handles bbox control (width or height)
  if(bbox_control == rc::CONTROL_BBOX_WIDTH)
    obj_bbox_dim = obj_msg->bbox_width;
  else
    obj_bbox_dim = obj_msg->bbox_height;

  // Update status msg
  status_msg.y.current = obj_pos.y;
  if(alignment_plane == rc::PLANE_YZ) { // Using fwd cam
    status_msg.x.current = obj_bbox_dim;
    status_msg.z.current = obj_pos.z;
  }
  else if(alignment_plane == rc::PLANE_XY) { // Using dwn cam
    status_msg.x.current = obj_pos.x;
    status_msg.z.current = obj_bbox_dim;
  }

  AlignmentController::UpdateError();
  timer = nh.createTimer(ros::Duration(max_zero_detect_duration), &AlignmentController::DisableControllerTimer, this, true);
}

// Send zero accel if an object has not been seen for 'x' seconds to keep the
// vehicle from moving endlessly
void AlignmentController::DisableControllerTimer(const ros::TimerEvent& event) {
  cmd_force_x.data = 0;
  cmd_force_y.data = 0;

  // MUST be active and NOT reset to publish the zero
  if(pid_surge_active)
    x_pub.publish(cmd_force_x);
  if(pid_sway_active) 
    y_pub.publish(cmd_force_y);
}

// Do NOT call UpdateError in this callback.
void AlignmentController::CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &cmd) {
  pid_surge_active = cmd->surge_active;
  pid_sway_active = cmd->sway_active;
  pid_heave_active = cmd->heave_active;

  //alignment_plane = cmd->alignment_plane;
  if(alignment_plane != rc::PLANE_YZ && alignment_plane != rc::PLANE_XY)
    alignment_plane = rc::PLANE_YZ; // Default to YZ-plane (fwd cam)

  bbox_control = cmd->bbox_control;
  if(bbox_control != rc::CONTROL_BBOX_WIDTH && bbox_control != rc::CONTROL_BBOX_HEIGHT)
    bbox_control = rc::CONTROL_BBOX_WIDTH; // Default to bbox width control

  // Update target values
  target_bbox_dim = cmd->bbox_dim;
  if(pid_sway_active) { // Sway
    target_pos.y = cmd->target_pos.y;
    status_msg.y.reference = target_pos.y;
  }
  else {
    AlignmentController::ResetSway();
  }
  target_pos.x = cmd->target_pos.x;
  if(alignment_plane == rc::PLANE_YZ) { // Using fwd cam
    if(pid_surge_active) { // Surge
      status_msg.x.reference = target_bbox_dim;
      ROS_INFO("Target BBox: %i", target_bbox_dim);
    }
    else {
      AlignmentController::ResetSurge();
    }

    if(pid_heave_active) { // Heave
      target_pos.z = cmd->target_pos.z;
      status_msg.z.reference = target_pos.z;
    }
    else {
      AlignmentController::ResetHeave();
    }
  }
  else if(alignment_plane == rc::PLANE_XY) { // Using dwn cam
    if(pid_surge_active) { // Surge
      target_pos.x = cmd->target_pos.x;
      status_msg.x.reference = target_pos.x;
    }
    else {
      AlignmentController::ResetSurge();
    }

    if(pid_heave_active) { // heave
      status_msg.z.reference = target_bbox_dim;
    }
    else {
      AlignmentController::ResetHeave();
    }
  }

  if(pid_surge_active || pid_sway_active || pid_heave_active)
    pid_alignment_active = true; // Only need one to be active
  else
    pid_alignment_active = false;
}

void AlignmentController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {
  current_depth = depth_msg->depth;
}

void AlignmentController::TaskInfoCB(const riptide_msgs::TaskInfo::ConstPtr& task_msg) {
  alignment_plane = task_msg->alignment_plane;
}

void AlignmentController::ResetSurge() {
    x_pid.reset();
    target_pos.x = 0;
    error.x = 0;
    error_dot.x = 0;
    last_error.x = 0;
    last_error_dot.x = 0;

    status_msg.x.reference = 0;
    status_msg.x.error = 0;
    status_msg.header.stamp = ros::Time::now();
    status_pub.publish(status_msg);

    cmd_force_x.data = 0;
    x_pub.publish(cmd_force_x);
}

void AlignmentController::ResetSway() {
    y_pid.reset();
    target_pos.y = 0;
    error.y = 0;
    error_dot.y = 0;
    last_error.y = 0;
    last_error_dot.y = 0;

    status_msg.y.reference = 0;
    status_msg.y.error = 0;
    status_msg.header.stamp = ros::Time::now();
    status_pub.publish(status_msg);

    cmd_force_y.data = 0;
    y_pub.publish(cmd_force_y);
}

void AlignmentController::ResetHeave() {
    z_pid.reset();
    target_pos.z = 0;
    error.z = 0;
    error_dot.z = 0;
    last_error.z = 0;
    last_error_dot.z = 0;

    status_msg.z.reference = 0;
    status_msg.z.error = 0;
    status_msg.header.stamp = ros::Time::now();
    status_pub.publish(status_msg);

    cmd_heave = 0;
    depth_cmd.active = true;
    depth_cmd.depth = current_depth + cmd_heave;
    depth_pub.publish(depth_cmd);
}
