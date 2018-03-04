#include "riptide_controllers/pwm_controller.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pwm_controller");
  PWMController pwm_controller;
  pwm_controller.Loop();
}

PWMController::PWMController() : nh()
{
  cmd_sub = nh.subscribe<riptide_msgs::ThrustStamped>("command/thrust", 1, &PWMController::ThrustCB, this);
  kill_sub = nh.subscribe<riptide_msgs::SwitchState>("state/switches", 1, &PWMController::SwitchCB, this);
  pwm_pub = nh.advertise<riptide_msgs::PwmStamped>("command/pwm", 1);

  //Initialization of the two trust/pwm slope arrays
  //The first column is for negative force, second column is positive force

  ccw_coeffs[0][0] = -24.4498;
  ccw_coeffs[0][1] = -28.0898;
  ccw_coeffs[1][0] = 30.2115;
  ccw_coeffs[1][1] = 23.3645;
  ccw_coeffs[2][0] = -23.9234;
  ccw_coeffs[2][1] = -28.4091;
  ccw_coeffs[3][0] = -23.9810;
  ccw_coeffs[3][1] = -27.47;

  cw_coeffs[0][0] = 29.1545;
  cw_coeffs[0][1] = 23.8095;
  cw_coeffs[1][0] = -24.8138;
  cw_coeffs[1][1] = -28.49;
  cw_coeffs[2][0] = 34.7222;
  cw_coeffs[2][1] = 25.5754;
  cw_coeffs[3][0] = -30.1205;
  cw_coeffs[3][1] = -22.6244;

  ros::Duration alive_timeout(ALIVE_TIMEOUT);
  last_alive_time = ros::Time::now();
  dead = false;
}

void PWMController::ThrustCB(const riptide_msgs::ThrustStamped::ConstPtr& thrust)
{
  pwm.header.stamp = thrust->header.stamp;

  pwm.pwm.surge_port_lo = counterclockwise(thrust->force.surge_port_lo, 0);
  pwm.pwm.surge_stbd_lo = clockwise(thrust->force.surge_stbd_lo, 0);

  pwm.pwm.sway_fwd = counterclockwise(thrust->force.sway_fwd, 1);
  pwm.pwm.sway_aft = clockwise(thrust->force.sway_aft, 1);

  pwm.pwm.heave_stbd_fwd = clockwise(thrust->force.heave_stbd_fwd, 2);
  pwm.pwm.heave_stbd_aft = counterclockwise(thrust->force.heave_stbd_aft, 2);

  pwm.pwm.heave_port_aft = clockwise(thrust->force.heave_port_aft, 3);
  pwm.pwm.heave_port_fwd = counterclockwise(thrust->force.heave_port_fwd, 3);
  pwm_pub.publish(pwm);

}

void PWMController::SwitchCB(const riptide_msgs::SwitchState::ConstPtr &state)
{
  if (!state->kill)
  {
      dead = false;
  }
}

void PWMController::Loop()
{
  ros::Rate rate(50);
  while (!ros::isShuttingDown())
  {
    ros::spinOnce();
    ros::Duration quiet_time = ros::Time::now() - last_alive_time;
    if (quiet_time > alive_timeout)
    {
      dead = true;
    }

    if (dead)
    {
      PWMController::PublishZeroPWM();
    }
    rate.sleep();
  }
}

int PWMController::counterclockwise(double raw_force, int thruster)
{
  int us = 1500;
  if(raw_force<0){
    us = 1500 + static_cast<int>(raw_force*ccw_coeffs[thruster][0]);
  }else if(raw_force>0){
    us = (int) (1500 + (raw_force*ccw_coeffs[thruster][1]));
  }else{
    us = 1500;
  }
  return us;
}

int PWMController::clockwise(double raw_force, int thruster)
{
  int us = 1500;
  if(raw_force<0){
    us = 1500 + static_cast<int>(raw_force*cw_coeffs[thruster][0]);
  }else if(raw_force>0){
    us = 1500 + static_cast<int>(raw_force*cw_coeffs[thruster][1]);
  }else{
    us = 1500;
  }

  return us;
}

void PWMController::PublishZeroPWM()
{
  pwm.header.stamp = ros::Time::now();

  pwm.pwm.surge_port_lo = 1500;
  pwm.pwm.surge_stbd_lo = 1500;
  pwm.pwm.sway_fwd = 1500;
  pwm.pwm.sway_aft = 1500;
  pwm.pwm.heave_stbd_fwd = 1500;
  pwm.pwm.heave_stbd_aft = 1500;
  pwm.pwm.heave_port_aft = 1500;
  pwm.pwm.heave_port_fwd = 1500;
  pwm_pub.publish(pwm);
}
