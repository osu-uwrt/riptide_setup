#include "riptide_controllers/depth_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653
#define MIN_DEPTH 0

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_controller");
  DepthController dc;
  ros::spin();
}

DepthController::DepthController() : nh("depth_controller") {
    ros::NodeHandle dcpid("depth_controller");
    R_b2w.setIdentity();
    R_w2b.setIdentity();
    tf.setValue(0, 0, 0);

    depth_controller_pid.init(dcpid, false);

    cmd_sub = nh.subscribe<riptide_msgs::DepthCommand>("/command/depth", 1, &DepthController::CommandCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &DepthController::DepthCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &DepthController::ImuCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("/controls/reset", 1, &DepthController::ResetController, this);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("/command/accel_depth", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatus>("/status/controls/depth", 1);

    DepthController::LoadParam<double>("max_depth", MAX_DEPTH);
    DepthController::LoadParam<double>("max_depth_error", MAX_DEPTH_ERROR);
    DepthController::LoadParam<double>("PID_IIR_LPF_bandwidth", PID_IIR_LPF_bandwidth);
    DepthController::LoadParam<double>("sensor_rate", sensor_rate);

    pid_depth_reset = true;
    pid_depth_active = false;

    // IIR LPF Variables
    double fc = PID_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
    dt_iir = 1.0/sensor_rate;
    alpha = 2*PI*dt_iir*fc / (2*PI*dt_iir*fc + 1); // Multiplier

    sample_start = ros::Time::now();

    DepthController::InitMsgs();
    DepthController::ResetDepth();
}

void DepthController::InitMsgs() {
  status_msg.reference = 0;
  status_msg.current = 0;
  status_msg.error = 0;
  accel.x = 0;
  accel.y = 0;
  accel.z = 0;
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

  if(!pid_depth_reset && pid_depth_active) {
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

  if(!pid_depth_reset && pid_depth_active) {
    status_msg.header.stamp = ros::Time::now();
    status_pub.publish(status_msg);
    cmd_pub.publish(accel);
  }
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
  DepthController::UpdateError();
}

// Subscribe to manual depth command
void DepthController::CommandCB(const riptide_msgs::DepthCommand::ConstPtr &cmd) {
  pid_depth_active = cmd->active;
  if(!pid_depth_reset && pid_depth_active) {
    depth_cmd = cmd->depth;
    depth_cmd = DepthController::Constrain(depth_cmd, MAX_DEPTH);
    if(depth_cmd < 0) // Min. depth is zero
      depth_cmd = 0;
    status_msg.reference = depth_cmd;

    DepthController::UpdateError();
  }
  else {
    DepthController::ResetDepth();
  }
}

// Create rotation matrix from IMU orientation
void DepthController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  vector3MsgToTF(imu_msg->euler_rpy, tf);
  tf.setValue(tf.x()*PI/180, tf.y()*PI/180, tf.z()*PI/180);
  R_b2w.setRPY(tf.x(), tf.y(), tf.z()); //Body to world rotations --> world_vector =  R_b2w * body_vector
  R_w2b = R_b2w.transpose(); //World to body rotations --> body_vector = R_w2b * world_vector
  DepthController::UpdateError();
}

void DepthController::ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg) {
  if(reset_msg->reset_depth) { // Reset
    pid_depth_reset = true;
    DepthController::ResetDepth();
  }
  else pid_depth_reset = false;
}

void DepthController::ResetDepth() {
  depth_controller_pid.reset();
  depth_cmd = 0;
  depth_error = 0;
  depth_error_dot = 0;
  last_error = 0;
  last_error_dot = 0;

  status_msg.reference = 0;
  status_msg.error = 0;

  output = 0;
  accel.x = 0;
  accel.y = 0;
  accel.z = 0;

  cmd_pub.publish(accel);
}
