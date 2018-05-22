#include "riptide_controllers/depth_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653
#define MAX_DEPTH_ERROR 1.5

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_controller");
  DepthController dc;
  dc.Loop();
}

DepthController::DepthController() {
    ros::NodeHandle dcpid("depth_controller");
    R_b2w.setIdentity();
    R_w2b.setIdentity();
    tf.setValue(0, 0, 0);

    pid_depth_init = false;

    depth_controller_pid.init(dcpid, false);

    cmd_sub = nh.subscribe<riptide_msgs::Depth>("command/depth", 1, &DepthController::CommandCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1, &DepthController::DepthCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1, &DepthController::ImuCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("controls/reset", 1, &DepthController::ResetController, this);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("command/accel/depth", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatus>("controls/status/depth", 1);

    sample_start = ros::Time::now();

    status_msg.reference = 0;
    status_msg.current = 0;
    status_msg.error = 0;
    accel.x = 0;
    accel.y = 0;
    accel.z = 0;
    DepthController::ResetDepth();
}

void DepthController::UpdateError() {
  if(pid_depth_init) {
    sample_duration = ros::Time::now() - sample_start;
    dt = sample_duration.toSec();

    depth_error = depth_cmd - current_depth;
    depth_error = DepthController::ConstrainError(depth_error, MAX_DEPTH_ERROR);
    depth_error_dot = (depth_error - last_error) / dt;
    last_error = depth_error;
    status_msg.error = depth_error;

    output = depth_controller_pid.computeCommand(depth_error, depth_error_dot, sample_duration);

    // Apply rotation matrix to control on depth regarldess of orientation
    // Use a world vector of [0 0 -1].
    // The z-value of -1 accounts for the proper direction of thrust force
    // since depth is positive downward, which does not adhere to the
    // body-frame coordinate system.
    accel.x = output * R_w2b.getRow(0).z() * -1;
    accel.y = output * R_w2b.getRow(1).z() * -1;
    accel.z = output * R_w2b.getRow(2).z() * -1;
  }

  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);
  cmd_pub.publish(accel);
  sample_start = ros::Time::now();
}

// Constrain physical error. This acts as a way to break up the motion into
// smaller segments. Otherwise, if error is too large, the vehicle would move
// too quickly in the water and exhibit large overshoot
double DepthController::ConstrainError(double error, double max) {
  if(error > max)
    return max;
  else if(error < -1*max)
    return -1*max;
  return error;
}

// Subscribe to command/depth
void DepthController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {
  current_depth = depth_msg->depth;
  status_msg.current = current_depth;
}

// Subscribe to state/depth
void DepthController::CommandCB(const riptide_msgs::Depth::ConstPtr &cmd) {
  // If a new AND different command arrives, reset the controllers
  if(cmd->depth != prev_depth_cmd)
    DepthController::ResetDepth();

  depth_cmd = cmd->depth;
  status_msg.reference = depth_cmd;
  prev_depth_cmd = depth_cmd;
}

// Create rotation matrix from IMU orientation
void DepthController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  vector3MsgToTF(imu_msg->euler_rpy, tf);
  tf.setValue(tf.x()*PI/180, tf.y()*PI/180, tf.z()*PI/180);
  R_b2w.setRPY(tf.x(), tf.y(), tf.z()); //Body to world rotations --> world_vector =  R_b2w * body_vector
  R_w2b = R_b2w.transpose(); //World to body rotations --> body_vector = R_w2b * world_vector
}

void DepthController::ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg) {
  //Reset controller if required from incoming message
  if(reset_msg->reset_depth)
    DepthController::ResetDepth();
  else pid_depth_init = true;
}

void DepthController::ResetDepth() {
  prev_depth_cmd = 0;
  depth_cmd = 0;
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
  accel.x = 0;
  accel.y = 0;
  accel.z = 0;
}

void DepthController::Loop() {
  while(!ros::isShuttingDown()) {
    DepthController::UpdateError(); // ALWAYS update error, regardless of circumstance
    ros::spinOnce();
  }
}
