#include "riptide_controllers/alignment_controller.h"

#undef debug
#undef report
#undef progress
#define MAX_TASK_ID 1

int main(int argc, char **argv) {
  ros::init(argc, argv, "alignment_controller");
  AlignmentController ac;
  ac.Loop();
}

// Constructor: AlignmentController()
AlignmentController::AlignmentController() : nh("alignment_controller") {
    ros::NodeHandle surge("surge_controller");
    ros::NodeHandle sway("sway_controller");
    ros::NodeHandle heave("heave_controller");

    x_pid.init(surge, false);
    y_pid.init(sway, false);
    z_pid.init(heave, false);

    // Default to gate alignment
    alignment_sub = nh.subscribe<riptide_msgs::TaskAlignment>("/task/gate/alignment", 1, &AlignmentController::AlignmentCB, this);
    command_sub = nh.subscribe<riptide_msgs::AlignmentCommand>("/command/alignment", 1, &AlignmentController::CommandCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("/controls/reset", 1, &AlignmentController::ResetController, this);

    xy_pub = nh.advertise<geometry_msgs::Vector3>("/command/auto/accel/linear", 1); // auto -> published by controller
    z_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/auto/depth", 1); // auto -> published by controller
    status_pub = nh.advertise<riptide_msgs::ControlStatusLinear>("/status/controls/linear", 1);

    AlignmentController::LoadParam<double>("max_x_error", MAX_X_ERROR);
    AlignmentController::LoadParam<double>("max_y_error", MAX_Y_ERROR);
    AlignmentController::LoadParam<double>("max_z_error", MAX_Z_ERROR);

    sample_start = ros::Time::now();
    AlignmentController::InitMsgs();
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

// Function: UpdateError()
// Computes and publishes new commands based on incoming alignment data and
// target data. There is some clever stuff in here to deal with the two alignment
// planes we deal in.
void AlignmentController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  if(pid_sway_init) {
    error.y = (target.y - task.y);
    error.y = AlignmentController::Constrain(error.y, MAX_Y_ERROR);
    error_dot.y = (error.y - last_error.y) / dt;
    last_error.y = error.y;
    status_msg.y.error = last_error.y;
    xy_cmd.y = y_pid.computeCommand(error.y, error_dot.y, sample_duration);
  }

  // If we are aligning in the YZ plane (forward cam), then X acceleration is
  // dependent on bounding box width (approximation for distance from task), while
  // Z acceleration is determined by offset in the Z axis.
  // If we are aligning in the YX plane (downward cam), then Z acceleration is
  // dependent on boudning box width, while X acceleration is determined by offset
  // in the X axis.
  if(pid_surge_init) {
    if (alignment_plane == riptide_msgs::AlignmentCommand::YZ) // Using fwd cam
      error.x = (target_bbox_width - task_bbox_width);
    else if (alignment_plane == riptide_msgs::AlignmentCommand::YX) // Using dwn cam
      error.x = (target.x - task.x);

    error.x = AlignmentController::Constrain(error.x, MAX_X_ERROR);
    error_dot.x = (error.x - last_error.x) / dt;
    last_error.x = error.x;
    status_msg.x.error = last_error.x;
    xy_cmd.x = x_pid.computeCommand(error.x, error_dot.x, sample_duration);
  }

  if(pid_heave_init) {
    if (alignment_plane == riptide_msgs::AlignmentCommand::YZ) // Using fwd cam
      error.z = (target.z - task.z);
    else if (alignment_plane == riptide_msgs::AlignmentCommand::YX) // Using dwn cam
      error.z = (target_bbox_width - task_bbox_width);

    error.z = AlignmentController::Constrain(error.z, MAX_Z_ERROR);
    error_dot.z = (error.z - last_error.z) / dt;
    last_error.z = error.z;
    status_msg.z.error = last_error.z;
    heave_cmd = z_pid.computeCommand(error.z, error_dot.z, sample_duration);
  }

  xy_cmd.z = 0;
  xy_pub.publish(xy_cmd);

  depth_cmd.isManual = false;
  depth_cmd.absolute = 0;
  depth_cmd.relative = heave_cmd;
  z_pub.publish(depth_cmd); // Output goes straight to the depth controller

  status_msg.header.stamp = sample_start;
  status_pub.publish(status_msg);

  sample_start = ros::Time::now();
}

double AlignmentController::Constrain(double current, double max) {
  if(current > max)
    return max;
  else if(current < -1*max)
    return -1*max;
  return current;
}

// Function: UpdateTaskID
// Parameters: id - ID of the new task to subscribe to
//
void AlignmentController::UpdateTaskID(int id) {
  // Validate id
  if (id >= 0 && id <= MAX_TASK_ID) {
    alignment_sub.shutdown(); // Unsubscribe from old task topic
    current_task_id = id;
    alignment_sub = nh.subscribe<riptide_msgs::TaskAlignment>(topics[current_task_id], 1, &AlignmentController::AlignmentCB, this);
  }
}

// Function: AlignmentCB
// Parameters: TaskAlignment msg
// Subscribe to state/vision/<task>/object_data to get relative position of task.
void AlignmentController::AlignmentCB(const riptide_msgs::TaskAlignment::ConstPtr &msg) {
  task.x = msg->relative_pos.x;
  task.y = msg->relative_pos.y;
  task.z = msg->relative_pos.z;

  // Bounding box width is always captured by the Y coordinate of the bounding box vertices.
  // This is because we only care about the YZ (forward cam) and YX (downward cam)
  // planes
  task_bbox_width = abs(msg->bbox.top_left.y - msg->bbox.bottom_right.y);

  status_msg.y.current = task.y;
  if (alignment_plane == riptide_msgs::AlignmentCommand::YZ) { // Using fwd cam
    status_msg.z.current = task.z;
    status_msg.x.current = task_bbox_width;
  }
  else if (alignment_plane == riptide_msgs::AlignmentCommand::YX) { // Using dwn cam
    status_msg.z.current = task_bbox_width;
    status_msg.x.current = task.x;
  }
}

// Function: CommandCB
// Parameters: AlignmentCommand msg
// Subscribe to /command/alignment to get target alignment relative to task
void AlignmentController::CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &cmd) {
  alignment_plane = cmd->alignment_plane;
  if(alignment_plane != riptide_msgs::AlignmentCommand::YZ && alignment_plane != riptide_msgs::AlignmentCommand::YX)
    alignment_plane = riptide_msgs::AlignmentCommand::YZ; // Default to YZ-plane (fwd cam)

  if(cmd->task_id != current_task_id) {
    AlignmentController::UpdateTaskID(cmd->task_id);
  }

  // Reset conroller if target value has changed
  // Also update commands and status_msg
  target_bbox_width = cmd->bbox_width;
  if(pid_sway_init && (cmd->target_pos.y != last_target.y)) {
    y_pid.reset();
    target.y = cmd->target_pos.y;
    last_target.y = target.y;
    status_msg.y.reference = target.y;
  }
  if(alignment_plane == riptide_msgs::AlignmentCommand::YZ) { // Using fwd cam
    if(pid_surge_init && (cmd->bbox_width != last_target_bbox_width)) {
      x_pid.reset();
      status_msg.x.reference = target_bbox_width;
    }
    if(pid_heave_init && (cmd->target_pos.z != last_target.z)) {
      z_pid.reset();
      target.z = cmd->target_pos.z;
      last_target.z = target.z;
      status_msg.z.reference = target.z;
    }
  }
  else if(alignment_plane == riptide_msgs::AlignmentCommand::YX) { // Using dwn cam
    if(pid_heave_init && (cmd->bbox_width != last_target_bbox_width)) {
      z_pid.reset();
      status_msg.z.reference = target_bbox_width;
    }
    if(pid_surge_init && (cmd->target_pos.x != last_target.x)) {
      x_pid.reset();
      target.x = cmd->target_pos.x;
      last_target.x = target.x;
      status_msg.x.reference = target.x;
    }
  }

  last_target_bbox_width = target_bbox_width;
}

void AlignmentController::ResetController(const riptide_msgs::ResetControls::ConstPtr& reset_msg) {
  //Reset any controllers if required from incoming message
  if(reset_msg->reset_surge)
    AlignmentController::ResetSurge();
  else pid_surge_init = true;

  if(reset_msg->reset_sway)
    AlignmentController::ResetSway();
  else pid_sway_init = true;

  if(reset_msg->reset_heave)
    AlignmentController::ResetHeave();
  else pid_heave_init = true;
}

void AlignmentController::ResetSurge() {
  x_pid.reset();
  target.x = 0;
  error.x = 0;
  error_dot.x = 0;
  last_target.x = 0;
  last_error.x = 0;

  status_msg.x.reference = 0;
  status_msg.x.error = 0;

  pid_surge_init = false;
  xy_cmd.x = 0;
}

void AlignmentController::ResetSway() {
  y_pid.reset();
  target.y = 0;
  error.y = 0;
  error_dot.y = 0;
  last_target.y = 0;
  last_error.y = 0;

  status_msg.y.reference = 0;
  status_msg.y.error = 0;

  pid_sway_init = false;
  xy_cmd.y = 0;
}

void AlignmentController::ResetHeave() {
  z_pid.reset();
  target.z = 0;
  error.z = 0;
  error_dot.z = 0;
  last_target.z = 0;
  last_error.z = 0;

  status_msg.z.reference = 0;
  status_msg.z.error = 0;

  pid_heave_init = false;
  heave_cmd = 0;
}

void AlignmentController::Loop() {
  ros::Rate rate(200);
  while(!ros::isShuttingDown()) {
    AlignmentController::UpdateError(); // ALWAYS update error, regardless of circumstance
    ros::spinOnce();
    rate.sleep();
  }
}
