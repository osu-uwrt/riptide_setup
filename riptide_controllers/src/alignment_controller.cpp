#include "riptide_controllers/alignment_controller.h"

#undef debug
#undef report
#undef progress

#define RESET_ID 0
#define DISABLE_ID 1

int main(int argc, char **argv) {
  ros::init(argc, argv, "alignment_controller");
  AlignmentController ac;
  ros::spin();
}

// Constructor: AlignmentController()
AlignmentController::AlignmentController() : nh("alignment_controller") {
    ros::NodeHandle surge("surge_controller");
    ros::NodeHandle sway("sway_controller");
    ros::NodeHandle heave("heave_controller");

    x_pid.init(surge, false);
    y_pid.init(sway, false);
    z_pid.init(heave, false);

    object_sub = nh.subscribe<riptide_msgs::Object>("/state/object", 1, &AlignmentController::ObjectCB, this);
    alignment_cmd_sub = nh.subscribe<riptide_msgs::AlignmentCommand>("/command/alignment", 1, &AlignmentController::CommandCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &AlignmentController::DepthCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("/controls/reset", 1, &AlignmentController::ResetController, this);
    task_info_sub = nh.subscribe<riptide_msgs::TaskInfo>("/task/info", 1, &AlignmentController::TaskInfoCB, this);

    xy_pub = nh.advertise<geometry_msgs::Vector3>("/command/accel_linear", 1);
    depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/depth", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1);

    AlignmentController::LoadParam<double>("max_x_error", MAX_X_ERROR);
    AlignmentController::LoadParam<double>("max_y_error", MAX_Y_ERROR);
    AlignmentController::LoadParam<double>("max_z_error", MAX_Z_ERROR);
    AlignmentController::LoadParam<double>("max_bbox_accel_error", MAX_BBOX_ACCEL_ERROR);
    AlignmentController::LoadParam<double>("max_bbox_depth_error", MAX_BBOX_DEPTH_ERROR);
    AlignmentController::LoadParam<double>("max_zero_detect_duration", max_zero_detect_duration);

    pid_surge_reset = true;
    pid_sway_reset = true;
    pid_heave_reset = true;
    pid_alignment_reset = true;

    pid_surge_active = false;
    pid_sway_active = false;
    pid_heave_active = false;
    pid_alignment_active = false;

    sample_start = ros::Time::now();
    bbox_control = riptide_msgs::Constants::CONTROL_BBOX_WIDTH;
    alignment_plane = riptide_msgs::Constants::PLANE_YZ;
    current_depth = 0;
    AlignmentController::InitMsgs();

    AlignmentController::ResetSurge(RESET_ID);
    AlignmentController::ResetSway(RESET_ID);
    AlignmentController::ResetHeave(RESET_ID);

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

// Send zero accel if an object has not been seen for 'x' seconds to keep the
// vehicle from moving endlessly
void AlignmentController::DisableControllerTimer(const ros::TimerEvent& event) {
  xy_cmd.x = 0;
  xy_cmd.y = 0;
  xy_cmd.z = 0;
  xy_pub.publish(xy_cmd);
}

void AlignmentController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // When controlling on position (NOT bbox dimension), you MUST subtract target pos FROM current pos
  // for the accel direction to be correct

  if(!pid_sway_reset && pid_sway_active) {
    error.y = (obj_pos.y - target_pos.y); // MUST subtract target pos from current pos
    error.y = AlignmentController::Constrain(error.y, MAX_Y_ERROR);
    error_dot.y = (error.y - last_error.y) / dt;
    last_error.y = error.y;
    status_msg.y.error = last_error.y;
    xy_cmd.y = y_pid.computeCommand(error.y, error_dot.y, sample_duration);
  }

  // If we are aligning in the YZ plane (forward cam), then X acceleration is
  // dependent on bounding box size (approximation for distance from task), while
  // Z acceleration is determined by offset in the Z axis.
  // If we are aligning in the YX plane (downward cam), then Z acceleration is
  // dependent on boudning box size, while X acceleration is determined by offset
  // in the X axis.
  if(!pid_surge_reset && pid_surge_active) {
    if (alignment_plane == riptide_msgs::Constants::PLANE_YZ) { // Using fwd cam
      error.x = (target_bbox_dim - obj_bbox_dim);
      error.x = AlignmentController::Constrain(error.x, MAX_BBOX_ACCEL_ERROR);
    }
    else if(alignment_plane == riptide_msgs::Constants::PLANE_XY) { // Using dwn cam
      error.x = (obj_pos.x - target_pos.x); // MUST subtract target pos from current pos
      error.x = AlignmentController::Constrain(error.x, MAX_X_ERROR);
    }

    error_dot.x = (error.x - last_error.x) / dt;
    last_error.x = error.x;
    status_msg.x.error = last_error.x;
    xy_cmd.x = x_pid.computeCommand(error.x, error_dot.x, sample_duration);
  }

  if(!pid_heave_reset && pid_heave_active) {
    if (alignment_plane == riptide_msgs::Constants::PLANE_YZ) { // Using fwd cam
      error.z = (obj_pos.z - target_pos.z); // MUST subtract target pos from current pos
      error.z = AlignmentController::Constrain(error.z, MAX_Z_ERROR);
    }
    else if(alignment_plane == riptide_msgs::Constants::PLANE_XY) { // Using dwn cam
      error.z = (target_bbox_dim - obj_bbox_dim);
      error.z = AlignmentController::Constrain(error.z, MAX_BBOX_DEPTH_ERROR);
    }

    error_dot.z = (error.z - last_error.z) / dt;
    last_error.z = error.z;
    status_msg.z.error = last_error.z;
    heave_cmd = z_pid.computeCommand(error.z, error_dot.z, sample_duration);

    depth_cmd.active = true;
    depth_cmd.depth = current_depth + heave_cmd;
    depth_pub.publish(depth_cmd);
  }

  if(!pid_alignment_reset && pid_alignment_active) {
    xy_cmd.z = 0;
    xy_pub.publish(xy_cmd);

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

// UpdateError should ONLY be called in this callback b/c this is the only thing
// that signals a new "sensor" input.
void AlignmentController::ObjectCB(const riptide_msgs::Object::ConstPtr &obj_msg) {
  timer.stop();
  obj_pos.x = obj_msg->pos.x;
  obj_pos.y = obj_msg->pos.y;
  obj_pos.z = obj_msg->pos.z;

  // This if-block handles bbox control (width or height)
  if(bbox_control == riptide_msgs::Constants::CONTROL_BBOX_WIDTH)
    obj_bbox_dim = obj_msg->bbox_width;
  else
    obj_bbox_dim = obj_msg->bbox_height;

  // Update status msg
  status_msg.y.current = obj_pos.y;
  if(alignment_plane == riptide_msgs::Constants::PLANE_YZ) { // Using fwd cam
    status_msg.x.current = obj_bbox_dim;
    status_msg.z.current = obj_pos.z;
  }
  else if(alignment_plane == riptide_msgs::Constants::PLANE_XY) { // Using dwn cam
    status_msg.x.current = obj_pos.x;
    status_msg.z.current = obj_bbox_dim;
  }

  AlignmentController::UpdateError();
  timer = nh.createTimer(ros::Duration(max_zero_detect_duration), &AlignmentController::DisableControllerTimer, this, true);
}

// Do NOT call UpdateError in this callback.
void AlignmentController::CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &cmd) {
  pid_surge_active = cmd->surge_active;
  pid_sway_active = cmd->sway_active;
  pid_heave_active = cmd->heave_active;

  //alignment_plane = cmd->alignment_plane;
  if(alignment_plane != riptide_msgs::Constants::PLANE_YZ && alignment_plane != riptide_msgs::Constants::PLANE_XY)
    alignment_plane = riptide_msgs::Constants::PLANE_YZ; // Default to YZ-plane (fwd cam)

  bbox_control = cmd->bbox_control;
  if(bbox_control != riptide_msgs::Constants::CONTROL_BBOX_WIDTH && bbox_control != riptide_msgs::Constants::CONTROL_BBOX_HEIGHT)
    bbox_control = riptide_msgs::Constants::CONTROL_BBOX_WIDTH; // Default to bbox width control

  // Update target values
  target_bbox_dim = cmd->bbox_dim;
  if(!pid_sway_reset && pid_sway_active) {
    target_pos.y = cmd->target_pos.y;
    status_msg.y.reference = target_pos.y;
  }
  else {
    AlignmentController::ResetSway(DISABLE_ID);
  }
  if(alignment_plane == riptide_msgs::Constants::PLANE_YZ) { // Using fwd cam
    if(!pid_surge_reset && pid_surge_active) {
      status_msg.x.reference = target_bbox_dim;
    }
    else {
      AlignmentController::ResetSurge(DISABLE_ID);
    }
    if(!pid_heave_reset && pid_heave_active) {
      target_pos.z = cmd->target_pos.z;
      status_msg.z.reference = target_pos.z;
    }
    else {
      AlignmentController::ResetHeave(DISABLE_ID);
    }
  }
  else if(alignment_plane == riptide_msgs::Constants::PLANE_XY) { // Using dwn cam
    if(!pid_surge_reset && pid_surge_active) {
      target_pos.x = cmd->target_pos.x;
      status_msg.x.reference = target_pos.x;
    }
    else {
      AlignmentController::ResetSurge(DISABLE_ID);
    }
    if(!pid_heave_reset && pid_heave_active) {
      status_msg.z.reference = target_bbox_dim;
    }
    else {
      AlignmentController::ResetHeave(DISABLE_ID);
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

void AlignmentController::ResetController(const riptide_msgs::ResetControls::ConstPtr& reset_msg) {
  if(reset_msg->reset_surge) {
    AlignmentController::ResetSurge(RESET_ID);
  }
  else pid_surge_reset = false;

  if(reset_msg->reset_sway) {
    AlignmentController::ResetSway(RESET_ID);
  }
  else pid_sway_reset = false;

  if(reset_msg->reset_heave) {
    AlignmentController::ResetHeave(RESET_ID);
  }
  else pid_heave_reset = false;

  if(pid_surge_reset && pid_sway_reset && pid_heave_reset)
    pid_alignment_reset = true;
  else
    pid_alignment_reset = false;
}

void AlignmentController::ResetSurge(int id) {
  x_pid.reset();
  target_pos.x = 0;
  error.x = 0;
  error_dot.x = 0;
  last_error.x = 0;

  status_msg.x.reference = 0;
  status_msg.x.error = 0;
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);

  xy_cmd.x = 0;
  xy_pub.publish(xy_cmd);

  if(id == RESET_ID)
    pid_surge_reset = true;
  else if(id == DISABLE_ID)
    pid_surge_active = false;
}

void AlignmentController::ResetSway(int id) {
  y_pid.reset();
  target_pos.y = 0;
  error.y = 0;
  error_dot.y = 0;
  last_error.y = 0;

  status_msg.y.reference = 0;
  status_msg.y.error = 0;
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);

  xy_cmd.y = 0;
  xy_pub.publish(xy_cmd);
  
  if(id == RESET_ID)
    pid_sway_reset = true;
  else if(id == DISABLE_ID)
    pid_sway_active = false;
}

void AlignmentController::ResetHeave(int id) {
  z_pid.reset();
  target_pos.z = 0;
  error.z = 0;
  error_dot.z = 0;
  last_error.z = 0;

  status_msg.z.reference = 0;
  status_msg.z.error = 0;
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);

  heave_cmd = 0;
  depth_cmd.active = true;
  depth_cmd.depth = current_depth + heave_cmd;
  depth_pub.publish(depth_cmd);

  if(id == RESET_ID)
    pid_heave_reset = true;
  else if(id == DISABLE_ID)
    pid_heave_active = false;
}
