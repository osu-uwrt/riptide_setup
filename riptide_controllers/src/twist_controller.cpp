#include "riptide_controllers/twist_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv) {
  ros::init(argc, argv, "twist_controller");
  TwistController tc;
  ros::spin();
}

void TwistController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // Roll error
  roll_twist_error = roll_twist_cmd - current_twist.x;
  roll_twist_error_dot = (roll_twist_error - last_error.x) / dt;
  last_error.x = roll_twist_error;

  // Pitch error
  pitch_twist_error = pitch_twist_cmd - current_twist.y;
  pitch_twist_error_dot = (pitch_twist_error - last_error.y) / dt;
  last_error.y = pitch_twist_error;

  // Yaw error
  // Always take shortest path to setpoint
  yaw_twist_error = yaw_twist_cmd - current_twist.z;
  yaw_twist_error_dot = (yaw_twist_error - last_error.z) / dt;
  last_error.z = yaw_twist_error;

  angular_accel_cmd.x = roll_twist_controller_pid.computeCommand(roll_twist_error, roll_twist_error_dot, sample_duration);
  angular_accel_cmd.y = pitch_twist_controller_pid.computeCommand(pitch_twist_error, pitch_twist_error_dot, sample_duration);
  angular_accel_cmd.z = yaw_twist_controller_pid.computeCommand(yaw_twist_error, yaw_twist_error_dot, sample_duration);

  cmd_pub.publish(angular_accel_cmd);
  sample_start = ros::Time::now();
}


TwistController::TwistController() {
    ros::NodeHandle rtcpid("roll_twist_controller");
    ros::NodeHandle ptcpid("pitch_twist_controller");
    ros::NodeHandle ytcpid("yaw_twist_controller");


    cmd_sub = nh.subscribe<geometry_msgs::Vector3>("command/twist", 1000, &TwistController::CommandCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1000, &TwistController::ImuCB, this);

    rtcpid.setParam("p", 0.0);
    ptcpid.setParam("p", 0.0);
    ytcpid.setParam("p", 0.0);


    roll_twist_controller_pid.init(rtcpid, false);
    pitch_twist_controller_pid.init(ptcpid, false);
    yaw_twist_controller_pid.init(ytcpid, false);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("command/accel/angular", 1);
    sample_start = ros::Time::now();
}

// Subscribe to state/imu
void TwistController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu) {
  current_twist = imu->angular_velocity;
  if (!pid_initialized) {
    roll_twist_cmd = current_twist.x;
    pitch_twist_cmd = current_twist.y;
    yaw_twist_cmd = current_twist.z;
  }

  TwistController::UpdateError();
}

// Subscribe to command/orientation
// set the MAX_ROLL and MAX_PITCH value in the header
void TwistController::CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd) {
  roll_twist_cmd = cmd->x;
  pitch_twist_cmd = cmd->y;
  yaw_twist_cmd = cmd->z;

  // Constrain pitch
  if(roll_twist_cmd > MAX_ROLL_TWIST)
    roll_twist_cmd = MAX_ROLL_TWIST;
  else if (roll_twist_cmd < -MAX_ROLL_TWIST)
    roll_twist_cmd = -MAX_ROLL_TWIST;

  // constrain roll
  if (pitch_twist_cmd > MAX_PITCH_TWIST)
    pitch_twist_cmd = MAX_PITCH_TWIST;
  else if (pitch_twist_cmd < -MAX_PITCH_TWIST)
    pitch_twist_cmd = -MAX_PITCH_TWIST;
  
  // constrain yaw
  if (yaw_twist_cmd > MAX_YAW_TWIST)
    yaw_twist_cmd = MAX_YAW_TWIST;
  else if (yaw_twist_cmd < -MAX_YAW_TWIST)
    yaw_twist_cmd = -MAX_YAW_TWIST;
    
  if (!pid_initialized)
    pid_initialized = true;

  TwistController::UpdateError();
}