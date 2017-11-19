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

  depth_error = cmd_depth - current_depth;
  d_error = (depth_error - last_error) / dt;
  last_error = depth_error;

  accel.data = depth_controller_pid.computeCommand(depth_error, d_error, sample_duration);


  // Gravity (m/s^2)
  double GRAVITY = 9.81;

  double VOLUME = 0.0334;
  double MASS = 34.47940950;
  // Water density (kg/m^3)
  double WATER_DENSITY = 1000.0;
  double BUOYANCY = VOLUME * WATER_DENSITY * GRAVITY;

  if (current_depth > 0.15)
    accel.data -=  BUOYANCY / MASS;
  cmd_pub.publish(accel);
  sample_start = ros::Time::now();
}


DepthController::DepthController() {
    ros::NodeHandle dcpid("depth_controller");
    cmd_sub = nh.subscribe<riptide_msgs::Depth>("command/depth", 1000, &DepthController::CommandCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1000, &DepthController::DepthCB, this);

    double p = 0.0;
    dcpid.setParam("p", 0.0);

    depth_controller_pid.init(dcpid, false);

    cmd_pub = nh.advertise<std_msgs::Float64>("command/accel/linear/z", 1);
    sample_start = ros::Time::now();
}

// Subscribe to command/depth
void DepthController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth) {
  current_depth = depth->depth;

  if (!pid_initialized)
    cmd_depth = current_depth;

  DepthController::UpdateError();
}

// Subscribe to state/depth
void DepthController::CommandCB(const riptide_msgs::Depth::ConstPtr &cmd) {
  cmd_depth = cmd->depth;

  if (!pid_initialized)
    pid_initialized = true;

  DepthController::UpdateError();
}
