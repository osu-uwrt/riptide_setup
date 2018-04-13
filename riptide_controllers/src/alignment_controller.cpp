#include "riptide_controllers/alignment_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv) {
  ros::init(argc, argv, "alignment_controller");
  AlignmentController ac;
  ros::spin();
}

void AlignmentController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  d_y_error = (y_error - last_y_error) / dt;
  last_y_error = y_error;

  accel.data = y_pid.computeCommand(y_error, d_y_error, sample_duration);

  cmd_pub.publish(accel);
  sample_start = ros::Time::now();
}


AlignmentController::AlignmentController() {
    ros::NodeHandle ypid("sway_controller");
    object_sub = nh.subscribe<riptide_msgs::ObjectData>("state/vision/pole/object_data", 1000, &AlignmentController::ObjectCB, this);
    y_pid.init(ypid, false);

    cmd_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/y", 1);
    sample_start = ros::Time::now();
}

// Subscribe to state/vision/gate
void AlignmentController::ObjectCB(const riptide_msgs::ObjectData::ConstPtr &msg) {
  // Y axis in robot frame maps to X axis of camera frame
  y_error = msg->rel_pos.x;
  ROS_INFO("%f", y_error);
  AlignmentController::UpdateError();
}
