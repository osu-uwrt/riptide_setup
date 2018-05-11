#include "riptide_controllers/depth_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_controller");
  DepthController dc;
  ros::spin();
}

void DepthController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  depth_error = current_depth - cmd_depth;
  depth_error_dot = (depth_error - last_error) / dt;
  last_error = depth_error;
  status_msg.error = depth_error;

  accel.data = depth_controller_pid.computeCommand(depth_error, depth_error_dot, sample_duration);

  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);
  cmd_pub.publish(accel);
  sample_start = ros::Time::now();
}

DepthController::DepthController() {
    ros::NodeHandle dcpid("depth_controller");

    pid_depth_init = false;

    depth_controller_pid.init(dcpid, false);

    cmd_sub = nh.subscribe<riptide_msgs::Depth>("command/depth", 1000, &DepthController::CommandCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1000, &DepthController::DepthCB, this);
    kill_sub = nh.subscribe<riptide_msgs::SwitchState>("state/switches", 10, &DepthController::SwitchCB, this);

    cmd_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/z", 1);

    sample_start = ros::Time::now();

    status_msg.reference = 0;
    status_msg.current = 0;
    status_msg.error = 0;
    accel.data = 0;
}

// Subscribe to command/depth
void DepthController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth) {
  current_depth = depth->depth;
  status_msg.current = current_depth;

  //Leave commented out
  /*if (!pid_depth_init)
    cmd_depth = current_depth;*/

  if(pid_depth_init)
    DepthController::UpdateError();
}

// Subscribe to state/depth
void DepthController::CommandCB(const riptide_msgs::Depth::ConstPtr &cmd) {
  cmd_depth = cmd->depth;
  status_msg.reference = cmd_depth;

  /*if (!pid_initialized)
    pid_initialized = true;*/

  if(pid_depth_init)
    DepthController::UpdateError();
}

//Subscribe to state/switches
void DepthController::SwitchCB(const riptide_msgs::SwitchState::ConstPtr &state) {
  if (!state->kill) {
    DepthController::ResetDepth();
  }
}

void DepthController::ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg) {
  //Reset controller if required from incoming message
  if(reset_msg->reset_depth)
    DepthController::ResetDepth();
  else pid_depth_init = true;
}

void DepthController::ResetDepth() {
  cmd_depth = 0;
  depth_error = 0;
  depth_error_dot = 0;
  depth_controller_pid.reset();
  current_depth = 0;
  last_error = 0;

  sample_start = ros::Time::now();
  sample_duration = ros::Duration(0);
  dt = 0;

  status_msg.reference = 0;
  status_msg.error = 0;

  pid_depth_init = false;
  accel.data = 0;
}
