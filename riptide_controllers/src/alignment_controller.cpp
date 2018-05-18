#include "riptide_controllers/alignment_controller.h"

#undef debug
#undef report
#undef progress

string[] tasks = {"gate", "pole"};
int tindex = 0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "alignment_controller");
  AlignmentController ac;
  ros::spin();
}

void AlignmentController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  error.y = (target.y - task.y);
  error_dot.y = (error.y - last_error.y) / dt;
  last_error.y = error.y;
  sway_cmd = y_pid.computeCommand(error.y, error_dot.y, sample_duration);

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


  sample_start = ros::Time::now();
}

AlignmentController::AlignmentController() {
    ros::NodeHandle surge("surge_controller");
    ros::NodeHandle sway("sway_controller");
    ros::NodeHandle heave("heave_controller");

    alignment_sub = nh.subscribe<riptide_msgs::TaskAlignment>("task/" + tasks[tindex] + "/alignment", 1, &AlignmentController::AlignmentCB, this);
    command_sub = nh.subscribe<riptide_msgs::AlignmentCommand>("command/alignment", 1, &AlignmentController::CommandCB, this);

    x_pid.init(surge, false);
    y_pid.init(sway, false);
    z_pid.init(heave, false);

    x_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/x", 1);
    y_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/y", 1);
    z_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/z", 1);

    sample_start = ros::Time::now();
}

// Subscribe to state/vision/<task>/object_data to get relative position of task.
// Task position is the setpoint
void AlignmentController::AlignmentCB(const riptide_msgs::TaskAlignment::ConstPtr &msg) {
  task.x = msg.relative_pos.x;
  task.y = msg.relative_pos.y;
  task.z = msg.relative_pos.z;

  task_bbox_width = abs(msg.bbox.top_left.y - msg.bbox.bottom_right.y);

  AlignmentController::UpdateError();
}

void AlignmentController::CommandCB(const riptide_msgs::AlignmentCommand::ConstPtr &cmd) {
  alignment_plane = cmd.alignment_plane;

  target.x = cmd.target_pos.x;
  target.y = cmd.target_pos.y;
  target.z = cmd.target_pos.z;

  target_bbox_width = cmd.bbox_width;

  AlignmentController::UpdateError();
}
