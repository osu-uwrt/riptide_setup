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


  d_surge_error = (surge_error - last_surge_error) / dt;
  d_sway_error = (sway_error - last_sway_error) / dt;

  last_surge_error = surge_error;
  last_sway_error = sway_error;

  accel_x.data = surge_pid.computeCommand(surge_error, d_surge_error, sample_duration);
  accel_y.data = sway_pid.computeCommand(sway_error, d_sway_error, sample_duration);

  surge_pub.publish(accel_x);
  sway_pub.publish(accel_y);
  sample_start = ros::Time::now();
}

AlignmentController::AlignmentController() {
    ros::NodeHandle surge_pid("surge_controller");
    ros::NodeHandle sway_pid("sway_controller");
    task_sub = nh.subscribe<riptide_msgs::TaskAlignment>("task/" + tasks[tindex] + "/alignment", 1, &AlignmentController::TaskAlignmentCB, this);
    cmd_sub = nh.subscribe<riptide_msgs::AlignmentCommand>("command/alignment", 1, &AlignmentController::CommandAlignmentCB, this)

    surge_pid.init(surgepid, false);
    sway_pid.init(ypid, false);
    pid_initialized = false;

    surge_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/x", 1);
    sway_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/y", 1);

    sample_start = ros::Time::now();
}

// Subscribe to state/vision/<task>/object_data to get relative position of task.
// Task position is the setpoint
void AlignmentController::TaskAlignmentCB(const riptide_msgs::ObjectData::ConstPtr &msg) {
  if (pid_initialized) {

    AlignmentController::UpdateError();
  }
}

void AlignmentController::CommandAlignmentCB(const riptide_msgs::CommandAlignment::ConstPtr &msg) {
  if (!pid_initialized)
    pid_initialized = true;

  AlignmentController::UpdateError();
}
