#include "riptide_controllers/alignment_controller.h"

#undef debug
#undef report
#undef progress
#define MAX_TASK_ID 1

string[] tasks = {"gate", "pole"};
int current_task_id = 0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "alignment_controller");
  AlignmentController ac;
  ros::spin();
}

// Function: UpdateError()
// Computes and publishes new commands based on incoming alignment data and
// target data. There is some clever stuff in here to deal with the two alignment
// planes we deal in.
void AlignmentController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  error.y = (target.y - task.y);
  error_dot.y = (error.y - last_error.y) / dt;
  last_error.y = error.y;
  sway_cmd = y_pid.computeCommand(error.y, error_dot.y, sample_duration);

  // If we are aligning in the YZ plane (forward cam), then X acceleration is
  // dependent on bounding box width (approximation for distance from task), while
  // Z acceleration is determined by offset in the Z axis.
  // If we are aligning in the YX plane (downward cam), then Z acceleration is
  // dependent on boudning box width, while X acceleration is determined by offset
  // in the X axis.
  if (alignment_plane == AlignmentCommand.YZ) {
    error.z = (target.z - task.z);
    error.x = (target_bbox_width - task_bbox_width);
  } else if (alignment_plane == AlignmentCommand.YX) {
    error.z = (target_bbox_width - task_bbox_width);
    error.x = (target.x - task.x);
  }

  error_dot.x = (error.x - last_error.x) / dt;
  last_error.x = error.x;
  surge_cmd = x_pid.computeCommand(error.x, error_dot.x, sample_duration);

  error_dot.z = (error.z - last_error.z) / dt;
  last_error.z = error.z;
  heave_cmd = z_pid.computeCommand(error.z, error_dot.z, sample_duration);


  x_pub.publish(surge_cmd);
  y_pub.publish(sway_cmd);
  z_pub.publish(heave_cmd);

  sample_start = ros::Time::now();
}

// Constructor: AlignmentController()
AlignmentController::AlignmentController() {
    ros::NodeHandle surge("surge_controller");
    ros::NodeHandle sway("sway_controller");
    ros::NodeHandle heave("heave_controller");

    alignment_sub = nh.subscribe<riptide_msgs::TaskAlignment>("task/" + tasks[current_task_id] + "/alignment", 1, &AlignmentController::AlignmentCB, this);
    command_sub = nh.subscribe<riptide_msgs::AlignmentCommand>("command/alignment", 1, &AlignmentController::CommandCB, this);

    x_pid.init(surge, false);
    y_pid.init(sway, false);
    z_pid.init(heave, false);

    x_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/x", 1);
    y_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/y", 1);
    z_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/z", 1);

    sample_start = ros::Time::now();
}

// Function: UpdateTaskID
// Parameters: id - ID of the new task to subscribe to
//
void AlignmentController::UpdateTaskID(int id) {
  // Validate id
  if (id >= 0 && id < MAX_TASK_ID) {
    alignment_sub.shutdown(); // Unsubscribe from old task topic
    alignment_sub = nh.subscribe<riptide_msgs::TaskAlignment>("task/" + tasks[current_task_id] + "/alignment", 1, &AlignmentController::AlignmentCB, this);
    current_task_id = id;
  }
}

// Function: AlignmentCB
// Parameters: TaskAlignment msg
// Subscribe to state/vision/<task>/object_data to get relative position of task.
void AlignmentController::AlignmentCB(const riptide_msgs::TaskAlignment::ConstPtr &msg) {
  if (pid_initialized) {
    task.x = msg.relative_pos.x;
    task.y = msg.relative_pos.y;
    task.z = msg.relative_pos.z;

    // Boudning box width is always captured by the Y coordinate of the bounding box vertices.
    // This is because we only care about the YZ (forward cam) and YX (downward cam)
    // planes
    task_bbox_width = abs(msg.bbox.top_left.y - msg.bbox.bottom_right.y);

    AlignmentController::UpdateError();
  }
}

// Function: CommandCB
// Parameters: AlignmentCommand msg
// Subscribe to /command/alignment to get target alignment relative to task
void AlignmentController::CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &cmd) {
  alignment_plane = cmd.alignment_plane;

  target.x = cmd.target_pos.x;
  target.y = cmd.target_pos.y;
  target.z = cmd.target_pos.z;

  target_bbox_width = cmd.bbox_width;

  if (cmd.task_id != current_task_id) {
    AlignmentController::UpdateTaskID(cmd.task_id);
  }

  AlignmentController::UpdateError();

  if (!pid_initialized)
    pid_initialized = true;
}
