#include "riptide_controllers/pwm_controller.h"
#define SPL 0
#define SSL 1
#define SWA 2
#define SWF 3
#define HSF 4
#define HSA 5
#define HPA 6
#define HPF 7
#define NEG_SLOPE 0
#define NEG_XINT 1
#define POS_SLOPE 2
#define POS_XINT 3

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
  load_calibration(thrust_config[SPL][NEG_SLOPE], "/SPL/NEG/SLOPE");
  load_calibration(thrust_config[SPL][POS_SLOPE], "/SPL/POS/SLOPE");
  load_calibration(thrust_config[SPL][NEG_XINT], "/SPL/NEG/XINT");
  load_calibration(thrust_config[SPL][POS_XINT], "/SPL/POS/XINT");

  load_calibration(thrust_config[SSL][NEG_SLOPE], "/SSL/NEG/SLOPE");
  load_calibration(thrust_config[SSL][POS_SLOPE], "/SSL/POS/SLOPE");
  load_calibration(thrust_config[SSL][NEG_XINT], "/SSL/NEG/XINT");
  load_calibration(thrust_config[SSL][POS_XINT], "/SSL/POS/XINT");

  load_calibration(thrust_config[HPA][NEG_SLOPE], "/HPA/NEG/SLOPE");
  load_calibration(thrust_config[HPA][POS_SLOPE], "/HPA/POS/SLOPE");
  load_calibration(thrust_config[HPA][NEG_XINT], "/HPA/NEG/XINT");
  load_calibration(thrust_config[HPA][POS_XINT], "/HPA/POS/XINT");

  load_calibration(thrust_config[HPF][NEG_SLOPE], "/HPF/NEG/SLOPE");
  load_calibration(thrust_config[HPF][POS_SLOPE], "/HPF/POS/SLOPE");
  load_calibration(thrust_config[HPF][NEG_XINT], "/HPF/NEG/XINT");
  load_calibration(thrust_config[HPF][POS_XINT], "/HPF/POS/XINT");

  load_calibration(thrust_config[HSA][NEG_SLOPE], "/HSA/NEG/SLOPE");
  load_calibration(thrust_config[HSA][POS_SLOPE], "/HSA/POS/SLOPE");
  load_calibration(thrust_config[HSA][NEG_XINT], "/HSA/NEG/XINT");
  load_calibration(thrust_config[HSA][POS_XINT], "/HSA/POS/XINT");

  load_calibration(thrust_config[HSF][NEG_SLOPE], "/HSF/NEG/SLOPE");
  load_calibration(thrust_config[HSF][POS_SLOPE], "/HSF/POS/SLOPE");
  load_calibration(thrust_config[HSF][NEG_XINT], "/HSF/NEG/XINT");
  load_calibration(thrust_config[HSF][POS_XINT], "/HSF/POS/XINT");

  load_calibration(thrust_config[SWF][NEG_SLOPE], "/SWF/NEG/SLOPE");
  load_calibration(thrust_config[SWF][POS_SLOPE], "/SWF/POS/SLOPE");
  load_calibration(thrust_config[SWF][NEG_XINT], "/SWF/NEG/XINT");
  load_calibration(thrust_config[SWF][POS_XINT], "/SWF/POS/XINT");

  load_calibration(thrust_config[SWA][NEG_SLOPE], "/SWA/NEG/SLOPE");
  load_calibration(thrust_config[SWA][POS_SLOPE], "/SWA/POS/SLOPE");
  load_calibration(thrust_config[SWA][NEG_XINT], "/SWA/NEG/XINT");
  load_calibration(thrust_config[SWA][POS_XINT], "/SWA/POS/XINT");

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
    pwm = (int) (thrust_config[thruster][NEG_XINT] + (raw_force*thrust_config[thruster][NEG_SLOPE]));
  }
  else if(raw_force > 0){
    pwm = (int) (thrust_config[thruster][POS_XINT] + (raw_force*thrust_config[thruster][POS_SLOPE]));
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
