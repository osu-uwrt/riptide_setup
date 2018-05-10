#include "riptide_controllers/attitude_controller.h"

#undef debug
#undef report
#undef progress

float round(float d) {
  return floor(d + 0.5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "attitude_controller");
  AttitudeController ac;
  ros::spin();
}

void AttitudeController::UpdateError() {
  // Roll error
  if(pid_roll_init) {
    sample_duration_roll = ros::Time::now() - sample_start_roll;
    dt_roll = sample_duration_roll.toSec();

    roll_error = roll_cmd - round(current_attitude.x);
    roll_error_dot = (roll_error - last_error.x) / dt_roll;
    last_error.x = roll_error;
    status_msg.roll.error = roll_error;

    accel_cmd.x = roll_controller_pid.computeCommand(roll_error, roll_error_dot, sample_duration_roll);
  }

  // Pitch error
  if(pid_pitch_init) {
    sample_duration_pitch = ros::Time::now() - sample_start_pitch;
    dt_pitch = sample_duration_pitch.toSec();

    pitch_error = pitch_cmd - round(current_attitude.y);
    pitch_error_dot = (pitch_error - last_error.y) / dt_pitch;
    last_error.y = pitch_error;
    status_msg.pitch.error = pitch_error;

    accel_cmd.y = pitch_controller_pid.computeCommand(pitch_error, pitch_error_dot, sample_duration_pitch);
  }

  // Yaw error
  if(pid_yaw_init) {
    sample_duration_yaw = ros::Time::now() - sample_start_yaw;
    dt_yaw = sample_duration_yaw.toSec();

    // Always take shortest path to setpoint
    yaw_error = yaw_cmd - round(current_attitude.z);
    if (yaw_error > 180)
        yaw_error -= 360;
    else if (yaw_error < -180)
        yaw_error += 360;

    yaw_error_dot = (yaw_error - last_error.z) / dt_yaw;
    last_error.z = yaw_error;
    status_msg.yaw.error = yaw_error;

    accel_cmd.z = yaw_controller_pid.computeCommand(yaw_error, yaw_error_dot, sample_duration_yaw);
  }

  //Publish status and command messages
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);
  cmd_pub.publish(accel_cmd);

  //update sample_start time if pid initialized for that axis
  if(pid_roll_init)
    sample_start_roll = ros::Time::now();
  if(pid_pitch_init)
    sample_start_pitch = ros::Time::now();
  if(pid_yaw_init)
    sample_start_yaw = ros::Time::now();

  /*sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // Roll error
  roll_error = roll_cmd - round(current_attitude.x);
  roll_error_dot = (roll_error - last_error.x) / dt;
  last_error.x = roll_error;

  // Pitch error
  pitch_error = pitch_cmd - round(current_attitude.y);
  pitch_error_dot = (pitch_error - last_error.y) / dt;
  last_error.y = pitch_error;

  // Yaw error
  // Always take shortest path to setpoint
  yaw_error = yaw_cmd - round(current_attitude.z);
  if (yaw_error > 180)
      yaw_error -= 360;
  else if (yaw_error < -180)
      yaw_error += 360;

  yaw_error_dot = (yaw_error - last_error.z) / dt;
  last_error.z = yaw_error;

  accel_cmd.x = roll_controller_pid.computeCommand(roll_error, roll_error_dot, sample_duration);
  accel_cmd.y = pitch_controller_pid.computeCommand(pitch_error, pitch_error_dot, sample_duration);
  accel_cmd.z = yaw_controller_pid.computeCommand(yaw_error, yaw_error_dot, sample_duration);

  error_msg.x = roll_error;
  error_msg.y = pitch_error;
  error_msg.z = yaw_error;

  error_pub.publish(error_msg);
  cmd_pub.publish(accel_cmd);
  sample_start = ros::Time::now();*/
}

AttitudeController::AttitudeController() {
    ros::NodeHandle rcpid("roll_controller");
    ros::NodeHandle ycpid("yaw_controller");
    ros::NodeHandle pcpid("pitch_controller");

    pid_roll_init = false;
    pid_pitch_init = false;
    pid_yaw_init = false;

    cmd_sub = nh.subscribe<geometry_msgs::Vector3>("command/attitude", 1000, &AttitudeController::CommandCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1000, &AttitudeController::ImuCB, this);
    kill_sub = nh.subscribe<riptide_msgs::SwitchState>("state/switches", 10, &AttitudeController::SwitchCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("controls/reset", 10, &AttitudeController::ResetController, this);

    roll_controller_pid.init(rcpid, false);
    yaw_controller_pid.init(ycpid, false);
    pitch_controller_pid.init(pcpid, false);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("command/accel/angular", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatusAngular>("controls/status/angular", 1);
    sample_start_roll = ros::Time::now();
    sample_start_pitch = sample_start_roll;
    sample_start_yaw = sample_start_roll;

    AttitudeController::InitPubMsg();
}

void AttitudeController::InitPubMsg() {
  status_msg.roll.reference = 0;
  status_msg.roll.current = 0;
  status_msg.roll.error = 0;

  status_msg.pitch.reference = 0;
  status_msg.pitch.current = 0;
  status_msg.pitch.error = 0;

  status_msg.yaw.reference = 0;
  status_msg.yaw.current = 0;
  status_msg.yaw.error = 0;

  accel_cmd.x = 0;
  accel_cmd.y = 0;
  accel_cmd.z = 0;
}

// Subscribe to state/imu
void AttitudeController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu) {
  current_attitude = imu->euler_rpy;
  status_msg.roll.current = current_attitude.x;
  status_msg.pitch.current = current_attitude.y;
  status_msg.yaw.current = current_attitude.z;

  if (pid_roll_init || pid_pitch_init || pid_yaw_init) {
    AttitudeController::UpdateError();
  }
}

//Subscribe to state/switches
void AttitudeController::SwitchCB(const riptide_msgs::SwitchState::ConstPtr &state) {
  if (!state->kill) {
    AttitudeController::ResetRoll();
    AttitudeController::ResetPitch();
    AttitudeController::ResetYaw();
  }
}

// Subscribe to command/orientation
// set the MAX_ROLL and MAX_PITCH value in the header
void AttitudeController::CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd) {
  roll_cmd = round(cmd->x);
  pitch_cmd = round(cmd->y);
  yaw_cmd = round(cmd->z);
  status_msg.roll.current = roll_cmd;
  status_msg.pitch.current = pitch_cmd;
  status_msg.yaw.current = yaw_cmd;

  // Constrain pitch
  if(roll_cmd > MAX_ROLL)
    roll_cmd = MAX_ROLL;
  else if (roll_cmd < -MAX_ROLL)
    roll_cmd = -MAX_ROLL;

  // constrain roll
  if (pitch_cmd > MAX_PITCH)
      pitch_cmd = MAX_PITCH;
  else if (pitch_cmd < -MAX_PITCH)
    pitch_cmd = -MAX_PITCH;

  AttitudeController::UpdateError();
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
  roll_cmd = 0;
  roll_error = 0;
  roll_error_dot = 0;
  roll_controller_pid.reset();
  current_attitude.x = 0;
  last_error.x = 0;

  sample_start_roll = ros::Time::now();
  sample_duration_roll = ros::Duration(0);
  dt_roll = 0;

  status_msg.roll.reference = 0;
  status_msg.roll.error = 0;

  pid_roll_init = false;
  accel_cmd.x = 0;
}

void AttitudeController::ResetPitch() {
  pitch_cmd = 0;
  pitch_error = 0;
  pitch_error_dot = 0;
  pitch_controller_pid.reset();
  current_attitude.y = 0;
  last_error.y = 0;

  sample_start_pitch = ros::Time::now();
  sample_duration_pitch = ros::Duration(0);
  dt_pitch = 0;

  status_msg.pitch.reference = 0;
  status_msg.pitch.error = 0;

  pid_pitch_init = false;
  accel_cmd.y = 0;
}

void AttitudeController::ResetYaw() {
  yaw_cmd = 0;
  yaw_error = 0;
  yaw_error_dot = 0;
  yaw_controller_pid.reset();
  current_attitude.z = 0;
  last_error.z = 0;

  sample_start_yaw = ros::Time::now();
  sample_duration_yaw = ros::Duration(0);
  dt_yaw = 0;

  status_msg.yaw.reference = 0;
  status_msg.yaw.error = 0;

  pid_yaw_init = false;
  accel_cmd.z = 0;
}
