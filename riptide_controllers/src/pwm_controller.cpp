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
  //The first column is for negative forces, second column is positive force

  // Surge Port Low
  load_calibration(thrust_slope[SPL][NEG], "/SPL/NEG");
  load_calibration(thrust_slope[SPL][POS], "/SPL/POS");

  load_calibration(thrust_slope[SSL][NEG], "/SSL/NEG");
  load_calibration(thrust_slope[SSL][POS], "/SSL/POS");

  load_calibration(thrust_slope[HPA][NEG], "/HPA/NEG");
  load_calibration(thrust_slope[HPA][POS], "/HPA/POS");

  load_calibration(thrust_slope[HPF][NEG], "/HPF/NEG");
  load_calibration(thrust_slope[HPF][POS], "/HPF/POS");

  load_calibration(thrust_slope[HSA][NEG], "/HSA/NEG");
  load_calibration(thrust_slope[HSA][POS], "/HSA/POS");

  load_calibration(thrust_slope[HSF][NEG], "/HSF/NEG");
  load_calibration(thrust_slope[HSF][POS], "/HSF/POS");

  load_calibration(thrust_slope[SWF][NEG], "/SWF/NEG");
  load_calibration(thrust_slope[SWF][POS], "/SWF/POS");

  load_calibration(thrust_slope[SWA][NEG], "/SWA/NEG");
  load_calibration(thrust_slope[SWA][POS], "/SWA/POS");

  alive_timeout = ros::Duration(2);
  last_alive_time = ros::Time::now();
  silent = false; // Silent refers to not receiving commands from the control stack
  dead = true; // Dead refers to the kill switch being pulled
}

void PWMController::ThrustCB(const riptide_msgs::ThrustStamped::ConstPtr& thrust)
{
  if (!dead)
  {
    msg.header.stamp = thrust->header.stamp;

    msg.pwm.surge_port_lo = thrust2pwm(thrust->force.surge_port_lo, SPL);
    msg.pwm.surge_stbd_lo = thrust2pwm(thrust->force.surge_stbd_lo, SSL);

    msg.pwm.sway_fwd = thrust2pwm(thrust->force.sway_fwd, SWF);
    msg.pwm.sway_aft = thrust2pwm(thrust->force.sway_aft, SWA);

    msg.pwm.heave_stbd_fwd = thrust2pwm(thrust->force.heave_stbd_fwd, HSF);
    msg.pwm.heave_stbd_aft = thrust2pwm(thrust->force.heave_stbd_aft, HSA);

    msg.pwm.heave_port_aft = thrust2pwm(thrust->force.heave_port_aft, HPA);
    msg.pwm.heave_port_fwd = thrust2pwm(thrust->force.heave_port_fwd, HPF);
    pwm_pub.publish(msg);
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
  while (ros::ok())
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
  int pwm = 1500;
  // If force is negative, use negative calibration.
  // If force is positive, use positive calibration
  // Otherwise, set PWM to 1500 (0 thrust)
  if(raw_force < 0)
  {
    pwm = 1500 + static_cast<int>(raw_force*thrust_slope[thruster][NEG]);
  }
  else if(raw_force > 0){
    pwm = (int) (1500 + (raw_force*thrust_slope[thruster][POS]));
  }
  else{
    pwm = 1500;
  }
  return pwm;
}

void PWMController::PublishZeroPWM()
{
  msg.header.stamp = ros::Time::now();

  msg.pwm.surge_port_lo = 1500;
  msg.pwm.surge_stbd_lo = 1500;
  msg.pwm.sway_fwd = 1500;
  msg.pwm.sway_aft = 1500;
  msg.pwm.heave_stbd_fwd = 1500;
  msg.pwm.heave_stbd_aft = 1500;
  msg.pwm.heave_port_aft = 1500;
  msg.pwm.heave_port_fwd = 1500;
  pwm_pub.publish(msg);
}

void PWMController::load_calibration(float &param, std::string name)
{
  try
  {
    if (!nh.getParam("/pwm_controller/" + name, param))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    ROS_ERROR("Critical! No calibration set for %s. Shutting down...", name.c_str());
    ros::shutdown();
  }
}
