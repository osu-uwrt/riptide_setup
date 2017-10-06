#include "riptide_controllers/depth_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_controller");
  OrientationController dc;
  ros::spin();
}

void OrientationController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();


  orienetation_error.x = cmd_orientation.x - current_orientation.x;
  d_error.x = (orientation_error.x - last_error.x) / dt;
  last_error.x = orientation_error.x;

  orienetation_error.y = cmd_orientation.y - current_orientation.y;
  d_error.y = (orientation_error.y - last_error.y) / dt;
  last_error.y = orientation_error.y;


  orienetation_error.x = cmd_orientation.x - current_orientation.x;
  if (orientation_error.z>180)
      orientation_error.z-180;
  if (orientation_error.z<-180)
      orienation_eroor+180;

  d_error.z = (orientation_error.z- last_error.z) / dt;
  last_error.z = orientation_error.z;

  accel.linear.y = 0;
  accel.linear.y = 0;
  accel.linear.z = 0;
  accel.angular.x = roll_controller_pid.computeCommand(orientation_error.x, d_error, sample_duration);
  accel.angular.y = pitch_controller_pid.computeCommand(orientation_error.y, d_error, sample_duration);
  accel.angular.z = yaw_controller_pid.computeCommand(orientation_error.z, d_error, sample_duration);;

  cmd_pub.publish(accel);
  sample_start = ros::Time::now();
}


Orientation::OrientationController() {
    ros::NodeHandle rcpid("roll_controller");
    ros::NodeHandle ycpid("yaw_controller");
    ros::NodeHandle pcpid("pitch_controller");


    cmd_sub = nh.subscribe<geometry_msgs::Vector3>("command/orientation", 1000, &OrientationController::CommandCB, this);
    orientation_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1000, &OrientationController::OrientationCB, this);

    rcpid.setParam("p", 0.0);
    ycpid.setParam("p", 0.0);
    pcpid.setParam("p", 0.0);


    roll_controller_pid.init(rcpid, false);
    yaw_controller_pid.init(ycpid, false);
    pitch_controller_pid.init(pcpid, false);

    cmd_pub = nh.advertise<geometry_msgs::Accel>("command/accel/angular", 1);
    sample_start = ros::Time::now();
}

// Subscribe to state/imu
void OrientationController::OrientationCB(const riptide_msgs::Imu::ConstPtr &imu) {
  current_orientation = imu->euler_rpy;
  if (!pid_initialized) {
    cmd_orientation = current_orientation;
  }

  OrientationController::UpdateError();
}

// Subscribe to command/orientation
// set the MAX_ROLL and MAX_PITCH value in the header
void OrientationController::CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd) {
  cmd_orientation = cmd;

  if(cmd_orientation.x>MAX_ROLL)
      cmd_orientation.x=MAX_ROLL;
  if (cmd_orientation.y>MAX_PITCH)
      cmd_orientation.y=MAX_PITCH;

  if (!pid_initialized)
    pid_initialized = true;

  OrientationController::UpdateError();
}
