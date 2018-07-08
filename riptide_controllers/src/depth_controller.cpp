#include "riptide_controllers/depth_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653
#define MIN_DEPTH 0

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_controller");
  DepthController dc;
  dc.Loop();
}

DepthController::DepthController() : nh("depth_controller") {
    ros::NodeHandle dcpid("depth_controller");
    R_b2w.setIdentity();
    R_w2b.setIdentity();
    tf.setValue(0, 0, 0);

    pid_depth_init = false;

    depth_controller_pid.init(dcpid, false);

    man_cmd_sub = nh.subscribe<riptide_msgs::DepthCommand>("/command/manual/depth", 1, &DepthController::ManualCommandCB, this);
    auto_cmd_sub = nh.subscribe<riptide_msgs::DepthCommand>("/command/auto/depth", 1, &DepthController::AutoCommandCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &DepthController::DepthCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &DepthController::ImuCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("/controls/reset", 1, &DepthController::ResetController, this);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("/command/auto/accel/depth", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatus>("/status/controls/depth", 1);

    DepthController::LoadParam<double>("max_depth", MAX_DEPTH);
    DepthController::LoadParam<double>("max_depth_error", MAX_DEPTH_ERROR);
    DepthController::LoadParam<double>("PID_IIR_LPF_bandwidth", PID_IIR_LPF_bandwidth);
    DepthController::LoadParam<double>("sensor_rate", sensor_rate);

    // IIR LPF Variables
    double fc = PID_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
    dt_iir = 1.0/sensor_rate;
    alpha = 2*PI*dt_iir*fc / (2*PI*dt_iir*fc + 1); // Multiplier

    sample_start = ros::Time::now();
    auto_enabled = true;

    status_msg.reference = 0;
    status_msg.current = 0;
    status_msg.error = 0;
    accel.x = 0;
    accel.y = 0;
    accel.z = 0;
    DepthController::ResetDepth();
}

// Load parameter from namespace
template <typename T>
void DepthController::LoadParam(string param, T &var)
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
    ROS_ERROR("Depth Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void DepthController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  if(pid_depth_init) {
    depth_error = depth_cmd - current_depth;
    depth_error = DepthController::Constrain(depth_error, MAX_DEPTH_ERROR);
    depth_error_dot = (depth_error - last_error) / dt;
    depth_error_dot = DepthController::SmoothErrorIIR(depth_error_dot, last_error_dot);
    last_error = depth_error;
    last_error_dot = depth_error_dot;
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
double DepthController::Constrain(double current, double max) {
  if(current > max)
    return max;
  else if(current < -1*max)
    return -1*max;
  return current;
}

// Apply IIR LPF to depth error
double DepthController::SmoothErrorIIR(double input, double prev) {
  return (alpha*input + (1-alpha)*prev);
}

// Subscribe to state/depth
void DepthController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {
  current_depth = depth_msg->depth;
  status_msg.current = current_depth;
}

// Subscribe to manual depth command
void DepthController::ManualCommandCB(const riptide_msgs::DepthCommand::ConstPtr &cmd) {
  // Reset controller if absolute target value has changed
  if(!auto_enabled && pid_depth_init && cmd->isManual) {
    if(cmd->absolute != last_depth_cmd_absolute)
      depth_controller_pid.reset();

    depth_cmd = cmd->absolute;
    depth_cmd = DepthController::Constrain(depth_cmd, MAX_DEPTH);
    if(depth_cmd < 0) // Min. depth is zero
      depth_cmd = 0;
    status_msg.reference = depth_cmd;
    last_depth_cmd_absolute = cmd->absolute;
  }
}

// Subscribe to automatic depth command
void DepthController::AutoCommandCB(const riptide_msgs::DepthCommand::ConstPtr &cmd) {
  // Do NOT reset controller if using auto commands - this will only waste time
  // in having the controller reset itself with every callback
  if(pid_depth_init && !cmd->isManual) { // Auto commands take priority
    if(cmd->relative != 0) {
      auto_enabled = true;
      auto_disable_duration = ros::Duration(0.0);
      auto_time = ros::Time::now();
    }
    else {
      auto_disable_duration = ros::Time::now() - auto_time;
      if(auto_disable_duration.toSec() > 0.1)
        auto_enabled = false;
    }

    if(auto_enabled) {
      depth_cmd = current_depth + cmd->relative;
      depth_cmd = DepthController::Constrain(depth_cmd, MAX_DEPTH);
      if(depth_cmd < 0) // Min. depth is zero
        depth_cmd = 0;
      status_msg.reference = depth_cmd;
    }
  }
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
  depth_controller_pid.reset();
  depth_cmd = 0;
  depth_error = 0;
  depth_error_dot = 0;
  last_depth_cmd_absolute = 0;
  last_error = 0;
  last_error_dot = 0;

  status_msg.reference = 0;
  status_msg.error = 0;

  pid_depth_init = false;
  output = 0;
  accel.x = 0;
  accel.y = 0;
  accel.z = 0;

  auto_disable_duration = ros::Duration(0.0);
}

void DepthController::Loop() {
  ros::Rate rate(200);
  while(!ros::isShuttingDown()) {
    DepthController::UpdateError(); // ALWAYS update error, regardless of circumstance
    ros::spinOnce();
    rate.sleep();
  }
}
