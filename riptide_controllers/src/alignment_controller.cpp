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

  // We are trying to align: -error = target
  x_error = -x_target;
  y_error = -y_target;

  d_x_error = (x_error - last_x_error) / dt;
  d_y_error = (y_error - last_y_error) / dt;

  last_x_error = x_error;
  last_y_error = y_error;

  accel_x.data = x_pid.computeCommand(x_error, d_x_error, sample_duration);
  accel_y.data = y_pid.computeCommand(y_error, d_y_error, sample_duration);
  depth.data = d_error + current_depth;

  // Use depth controller to set
  d_pub.publish(depth);
  x_pub.publish(accel_x);
  y_pub.publish(accel_y);
  sample_start = ros::Time::now();
}

AlignmentController::AlignmentController() {
    ros::NodeHandle xpid("surge_controller");
    ros::NodeHandle ypid("sway_controller");
    object_sub = nh.subscribe<riptide_msgs::ObjectData>("task/" + tasks[tindex] + "/alignment", 1, &AlignmentController::ObjectCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1, &AlignmentController::DepthCB, this);

    x_pid.init(xpid, false);
    y_pid.init(ypid, false);

    x_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/x", 1);
    y_pub = nh.advertise<std_msgs::Float32>("command/accel/linear/y", 1);
    d_pub = nh.advertise<std_msgs::Float32>("command/depth", 1);

    sample_start = ros::Time::now();
}

// Subscribe to state/vision/<task>/object_data to get relative position of task.
// Task position is the setpoint
void AlignmentController::ObjectCB(const riptide_msgs::ObjectData::ConstPtr &msg) {
  x_target = msg->rel_pos.x;
  y_target = msg->rel_pos.y;
  d_error = -msg->rel_pos.z;

  ROS_INFO("%f", y_target);
  AlignmentController::UpdateError();
}

// Subscribe to state/depth to update the target depth passed along to the depth controller
void AlignmentController::DepthCB(const riptide_msgs::ObjectData::ConstPtr &msg) {

}
