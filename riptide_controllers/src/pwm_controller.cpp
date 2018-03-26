#include "riptide_controllers/pwm_controller.h"
#define SPL 0
#define SSL 1
#define SWA 2
#define SWF 3
#define HSF 4
#define HSA 5
#define HPA 6
#define HPF 7
#define NEG 0
#define POS 1

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
  //The first column is for negative forces, second column is positive forces

  // Surge Port Low
  thrust_slope[SPL][NEG] = -5.349046;
  thrust_slope[SPL][POS] = -6.176643;
  // Surge Starboard Low
  thrust_slope[SSL][NEG] = 5.234392;
  thrust_slope[SSL][POS] = 6.393562;
  // Sway Forward
  thrust_slope[SWF][NEG] = 5.161853;
  thrust_slope[SWF][POS] = 6.621582;
  // Sway Aft
  thrust_slope[SWA][NEG] = -5.444654;
  thrust_slope[SWA][POS] = -6.239471;
  // Heave Surge Forward
  thrust_slope[HSF][NEG] = 5.159923;
  thrust_slope[HSF][POS] = 6.277069
  // Heave Port Forward
  thrust_slope[HPF][NEG] = -5.222560;
  thrust_slope[HPF][POS] = -6.042335;
  // Heave Port Aft
  thrust_slope[HPA][NEG] = -4.886643;
  thrust_slope[HPA][POS] = -6.449618;
  // Heave Starboard Aft
  thrust_slope[HSA][NEG] = -5.215345;
  thrust_slope[HSA][POS] = -6.202313;

  alive_timeout = ros::Duration(2);
  last_alive_time = ros::Time::now();
  silent = false; // Silent refers to not receiving commands from the control stack
  dead = true; // Dead refers to the kill switch being pulled
}

void PWMController::ThrustCB(const riptide_msgs::ThrustStamped::ConstPtr& thrust)
{
  if (!dead)
  {
    pwm.header.stamp = thrust->header.stamp;

    pwm.pwm.surge_port_lo = thrust2pwm(thrust->force.surge_port_lo, SPL);
    pwm.pwm.surge_stbd_lo = thrust2pwm(thrust->force.surge_stbd_lo, SSL);

    pwm.pwm.sway_fwd = thrust2pwm(thrust->force.sway_fwd, SWF);
    pwm.pwm.sway_aft = thrust2pwm(thrust->force.sway_aft, SWA);

    pwm.pwm.heave_stbd_fwd = thrust2pwm(thrust->force.heave_stbd_fwd, HSF);
    pwm.pwm.heave_stbd_aft = thrust2pwm(thrust->force.heave_stbd_aft, HSA);

    pwm.pwm.heave_port_aft = thrust2pwm(thrust->force.heave_port_aft, HPA);
    pwm.pwm.heave_port_fwd = thrust2pwm(thrust->force.heave_port_fwd, HPF);
    pwm_pub.publish(pwm);
    last_alive_time = ros::Time::now();
    silent = false;
  }
}

void PWMController::SwitchCB(const riptide_msgs::SwitchState::ConstPtr &state)
{
  dead = !state->kill;
}

void PWMController::Loop()
{
  ros::Rate rate(10);
  while (!ros::isShuttingDown())
  {
    ros::spinOnce();
    ros::Duration quiet_time = ros::Time::now() - last_alive_time;
    if (quiet_time >= alive_timeout)
    {
      silent = true;
    }

    if (silent || dead)
    {
      PWMController::PublishZeroPWM();
    }
    rate.sleep();
  }
}

int PWMController::thrust2pwm(double raw_force, int thruster)
{
  int us = 1500;
  if(raw_force<0){
    us = 1500 + static_cast<int>(raw_force*thrust_slope[thruster][NEG]);
  }else if(raw_force>0){
    us = (int) (1500 + (raw_force*thrust_slope[thruster][POS]));
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
