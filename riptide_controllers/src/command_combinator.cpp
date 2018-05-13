#include "riptide_controllers/command_combinator.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "command_combinator");
  CommandCombinator cc;
  ros::spin();
}

void CommandCombinator::linearXCB(const std_msgs::Float64::ConstPtr &accel) {
  current_accel.linear.x = accel->data;
  cmd_pub.publish(current_accel);
}

void CommandCombinator::linearYCB(const std_msgs::Float64::ConstPtr &accel) {
  current_accel.linear.y = accel->data;
  cmd_pub.publish(current_accel);
}

void CommandCombinator::linearZCB(const std_msgs::Float64::ConstPtr &accel) {
  current_accel.linear.z = accel->data;
  cmd_pub.publish(current_accel);
}

void CommandCombinator::angularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel) {
  current_accel.angular.x = ang_accel->x;
  current_accel.angular.y = ang_accel->y;
  current_accel.angular.z = ang_accel->z;
  cmd_pub.publish(current_accel);
}

CommandCombinator::CommandCombinator() {
    linear_x_sub = nh.subscribe<std_msgs::Float64>("command/accel/linear/x", 1, &CommandCombinator::linearXCB, this);
    linear_y_sub = nh.subscribe<std_msgs::Float64>("command/accel/linear/y", 1, &CommandCombinator::linearYCB, this);
    linear_z_sub = nh.subscribe<std_msgs::Float64>("command/accel/linear/z", 1, &CommandCombinator::linearZCB, this);

    angular_sub = nh.subscribe<geometry_msgs::Vector3>("command/accel/angular", 1, &CommandCombinator::angularCB, this);
    cmd_pub = nh.advertise<geometry_msgs::Accel>("command/accel", 1);

    current_accel.linear.x = 0;
    current_accel.linear.y = 0;
    current_accel.linear.z = 0;
    current_accel.angular.x = 0;
    current_accel.angular.y = 0;
    current_accel.angular.z = 0;
}

void CommandCombinator::ResetController() {
    current_accel.linear.x = 0;
    current_accel.linear.y = 0;
    current_accel.linear.z = 0;
    current_accel.angular.x = 0;
    current_accel.angular.y = 0;
    current_accel.angular.z = 0;
    cmd_pub.publish(current_accel);
}
