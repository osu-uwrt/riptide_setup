#include "riptide_controllers/attitude_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653

float round(float d) {
  return floor(d + 0.5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "attitude_controller");
  AttitudeController ac;
  ac.Loop();
}

AttitudeController::AttitudeController() : nh("attitude_controller") {
    ros::NodeHandle rcpid("roll_controller");
    ros::NodeHandle ycpid("yaw_controller");
    ros::NodeHandle pcpid("pitch_controller");

    pid_roll_init = false;
    pid_pitch_init = false;
    pid_yaw_init = false;

    roll_controller_pid.init(rcpid, false);
    yaw_controller_pid.init(ycpid, false);
    pitch_controller_pid.init(pcpid, false);

    R_b2w.setIdentity();
    R_w2b.setIdentity();
    tf.setValue(0, 0, 0);
    ang_vel.setValue(0, 0, 0);

    cmd_sub = nh.subscribe<geometry_msgs::Vector3>("/command/manual/attitude", 1, &AttitudeController::ManualCommandCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &AttitudeController::ImuCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("/controls/reset", 1, &AttitudeController::ResetController, this);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("/command/auto/accel/angular", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1);

    AttitudeController::LoadParam<double>("max_roll_error", MAX_ROLL_ERROR);
    AttitudeController::LoadParam<double>("max_pitch_error", MAX_PITCH_ERROR);
    AttitudeController::LoadParam<double>("max_yaw_error", MAX_YAW_ERROR);
    AttitudeController::LoadParam<double>("max_roll_limit", MAX_ROLL_LIMIT);
    AttitudeController::LoadParam<double>("max_pitch_limit", MAX_PITCH_LIMIT);
    AttitudeController::LoadParam<double>("PID_IIR_LPF_bandwidth", PID_IIR_LPF_bandwidth);
    AttitudeController::LoadParam<double>("imu_filter_rate", imu_filter_rate);

    // IIR LPF Variables
    double fc = PID_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
    dt_iir = 1.0/imu_filter_rate;
    alpha = 2*PI*dt_iir*fc / (2*PI*dt_iir*fc + 1); // Multiplier

    sample_start = ros::Time::now();

    AttitudeController::InitMsgs();
    AttitudeController::ResetRoll();
    AttitudeController::ResetPitch();
    AttitudeController::ResetYaw();
}

void AttitudeController::InitMsgs() {
  status_msg.roll.reference = 0;
  status_msg.roll.current = 0;
  status_msg.roll.error = 0;

  status_msg.pitch.reference = 0;
  status_msg.pitch.current = 0;
  status_msg.pitch.error = 0;

  status_msg.yaw.reference = 0;
  status_msg.yaw.current = 0;
  status_msg.yaw.error = 0;

  ang_accel_cmd.x = 0;
  ang_accel_cmd.y = 0;
  ang_accel_cmd.z = 0;
}

// Load parameter from namespace
template <typename T>
void AttitudeController::LoadParam(string param, T &var)
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
    ROS_ERROR("Attitude Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void AttitudeController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // Roll error
  if(pid_roll_init) {
    roll_error = roll_cmd - current_attitude.x;
    roll_error = AttitudeController::Constrain(roll_error, MAX_ROLL_ERROR);
    roll_error_dot = -ang_vel.x(); // Negative sign is necesary to account for correct sign of error_dot
    last_error.x = roll_error;
    last_error_dot.x = roll_error_dot;
    status_msg.roll.error = roll_error;

    ang_accel_cmd.x = roll_controller_pid.computeCommand(roll_error, roll_error_dot, sample_duration);
  }

  // Pitch error
  if(pid_pitch_init) {
    pitch_error = pitch_cmd - current_attitude.y;
    pitch_error = AttitudeController::Constrain(pitch_error, MAX_PITCH_ERROR);
    pitch_error_dot = -ang_vel.y(); // Negative sign is necesary to account for correct sign of error_dot
    last_error.y = pitch_error;
    last_error_dot.y = pitch_error_dot;
    status_msg.pitch.error = pitch_error;

    ang_accel_cmd.y = pitch_controller_pid.computeCommand(pitch_error, pitch_error_dot, sample_duration);
  }

  // Yaw error
  if(pid_yaw_init) {
    // Always take shortest path to setpoint
    yaw_error = yaw_cmd - current_attitude.z;
    if (yaw_error > 180)
        yaw_error -= 360;
    else if (yaw_error < -180)
        yaw_error += 360;
    yaw_error = AttitudeController::Constrain(yaw_error, MAX_YAW_ERROR);

    yaw_error_dot = -ang_vel.z(); // Negative sign is necesary to account for correct sign of error_dot
    last_error.z = yaw_error;
    last_error_dot.z = yaw_error_dot;
    status_msg.yaw.error = yaw_error;

    ang_accel_cmd.z = yaw_controller_pid.computeCommand(yaw_error, yaw_error_dot, sample_duration);
  }

  // ALWAYS Publish status and command messages
  cmd_pub.publish(ang_accel_cmd);
  status_msg.header.stamp = sample_start;
  status_pub.publish(status_msg);

  sample_start = ros::Time::now();
}

// Constrain physical error. This acts as a way to break up the motion into
// smaller segments. Otherwise, if error is too large, the vehicle would move
// too quickly in the water in exhibit large overshoot
double AttitudeController::Constrain(double current, double max) {
  if(current > max)
    return max;
  else if(current < -1*max)
    return -1*max;
  return current;
}

// Apply IIR LPF to error
double AttitudeController::SmoothErrorIIR(double input, double prev) {
  return (alpha*input + (1-alpha)*prev);
}

// Subscribe to state/imu
void AttitudeController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  current_attitude = imu_msg->euler_rpy;
  status_msg.roll.current = current_attitude.x;
  status_msg.pitch.current = current_attitude.y;
  status_msg.yaw.current = current_attitude.z;

  //Get angular velocity (leave in [deg/s])
  vector3MsgToTF(imu_msg->ang_vel, ang_vel);
}

// Subscribe to command/attitude
// set the MAX_ROLL and MAX_PITCH value in the header
void AttitudeController::ManualCommandCB(const geometry_msgs::Vector3::ConstPtr &cmd) {
  // Reset controller if target value has changed
  // Also update commands and status_msg
  if(pid_roll_init && (cmd->x != last_roll_cmd)) {
    roll_controller_pid.reset();
    roll_cmd = cmd->x;
    roll_cmd = AttitudeController::Constrain(roll_cmd, MAX_ROLL_LIMIT);
    last_roll_cmd = roll_cmd;
    status_msg.roll.reference = roll_cmd;
  }

  if(pid_pitch_init && (cmd->y != last_pitch_cmd)) {
    pitch_controller_pid.reset();
    pitch_cmd = cmd->y;
    pitch_cmd = AttitudeController::Constrain(pitch_cmd, MAX_PITCH_LIMIT);
    last_pitch_cmd = pitch_cmd;
    status_msg.pitch.reference = pitch_cmd;
  }

  if(pid_yaw_init && (cmd->z != last_yaw_cmd)) {
    yaw_controller_pid.reset();
    yaw_cmd = cmd->z;
    last_yaw_cmd = yaw_cmd;
    status_msg.yaw.reference = yaw_cmd;
  }
}

void AttitudeController::ResetController(const riptide_msgs::ResetControls::ConstPtr& reset_msg) {
  //Reset any controllers if required from incoming message
  if(reset_msg->reset_roll)
    AttitudeController::ResetRoll();
  else pid_roll_init = true;

  if(reset_msg->reset_pitch)
    AttitudeController::ResetPitch();
  else pid_pitch_init = true;

  if(reset_msg->reset_yaw)
    AttitudeController::ResetYaw();
  else pid_yaw_init = true;
}

void AttitudeController::ResetRoll() {
  roll_controller_pid.reset();
  last_roll_cmd = 0;
  roll_cmd = 0;
  roll_error = 0;
  roll_error_dot = 0;
  last_error.x = 0;
  last_error_dot.x = 0;

  status_msg.roll.reference = 0;
  status_msg.roll.error = 0;

  pid_roll_init = false;
  ang_accel_cmd.x = 0;
}

void AttitudeController::ResetPitch() {
  pitch_controller_pid.reset();
  last_pitch_cmd = 0;
  pitch_cmd = 0;
  pitch_error = 0;
  pitch_error_dot = 0;
  last_error.y = 0;
  last_error_dot.y = 0;

  status_msg.pitch.reference = 0;
  status_msg.pitch.error = 0;

  pid_pitch_init = false;
  ang_accel_cmd.y = 0;
}

void AttitudeController::ResetYaw() {
  yaw_controller_pid.reset();
  last_yaw_cmd = 0;
  yaw_cmd = 0;
  yaw_error = 0;
  yaw_error_dot = 0;
  last_error.z = 0;
  last_error_dot.z = 0;

  status_msg.yaw.reference = 0;
  status_msg.yaw.error = 0;

  pid_yaw_init = false;
  ang_accel_cmd.z = 0;
}

void AttitudeController::Loop() {
  ros::Rate rate(200);
  while(!ros::isShuttingDown()) {
    AttitudeController::UpdateError(); // ALWAYS update error, regardless of circumstance
    ros::spinOnce();
    rate.sleep();
  }
}
