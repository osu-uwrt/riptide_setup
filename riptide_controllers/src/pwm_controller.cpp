#include "riptide_controllers/pwm_controller.h"

// CW Thrusters (odd numbers): SSL(1), SWF(3), HSF(5), HPA(7)
// CCW Thrusters (even numbers): SPL(0), SWA(2), HPF(4), HSA(6)
#define CCW 0
#define CW 1
#define SPL 0
#define SSL 1
#define SWA 2
#define SWF 3
#define HPF 4
#define HSF 5
#define HSA 6
#define HPA 7

// Critical Thrust Indeces
#define MIN_THRUST 0
#define STARTUP_THRUST 1

// Slope and y-intercept Indeces
#define POS_SLOPE 0
#define POS_YINT 1
#define NEG_SLOPE 2
#define NEG_YINT 3

#define MIN_PWM 1100
#define MAX_PWM 1900
#define NEUTRAL_PWM 1500
#define OFFSET 20 // Copro driver subtracts 20, but the thruster_config contains actual PWMs

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pwm_controller");
  PWMController pwm_controller;
  pwm_controller.Loop();
}

PWMController::PWMController() : nh("pwm_controller")
{
  cmd_sub = nh.subscribe<riptide_msgs::ThrustStamped>("/command/thrust", 1, &PWMController::ThrustCB, this);
  kill_sub = nh.subscribe<riptide_msgs::SwitchState>("/state/switches", 1, &PWMController::SwitchCB, this);
  reset_sub = nh.subscribe<riptide_msgs::ResetControls>("/controls/reset", 1, &PWMController::ResetController, this);
  pwm_pub = nh.advertise<riptide_msgs::PwmStamped>("/command/pwm", 1);

  // There are 3 regions of thrust: deadband, startup, and primary
  // Deadband is a region where the propeller will not move due to internal friction and inertia
  // Startup is nonlinear, but is approximated as a line (for now)
  // Primary is linear, but has different slope than startup

  // Critical Thrusts
  PWMController::LoadParam<float>("CW/MIN_THRUST", critical_thrusts[CW][MIN_THRUST]);
  PWMController::LoadParam<float>("CW/STARTUP_THRUST", critical_thrusts[CW][STARTUP_THRUST]);
  PWMController::LoadParam<float>("CCW/MIN_THRUST", critical_thrusts[CCW][MIN_THRUST]);
  PWMController::LoadParam<float>("CCW/STARTUP_THRUST", critical_thrusts[CCW][STARTUP_THRUST]);

  // Startup Config (Linear Fit: y = mx + b)
  PWMController::LoadParam<float>("CW/SU_POS_THRUST/SLOPE", startup_config[CW][POS_SLOPE]);
  PWMController::LoadParam<float>("CW/SU_POS_THRUST/YINT", startup_config[CW][POS_YINT]);
  PWMController::LoadParam<float>("CW/SU_NEG_THRUST/SLOPE", startup_config[CW][NEG_SLOPE]);
  PWMController::LoadParam<float>("CW/SU_NEG_THRUST/YINT", startup_config[CW][NEG_YINT]);

  PWMController::LoadParam<float>("CCW/SU_POS_THRUST/SLOPE", startup_config[CCW][POS_SLOPE]);
  PWMController::LoadParam<float>("CCW/SU_POS_THRUST/YINT", startup_config[CCW][POS_YINT]);
  PWMController::LoadParam<float>("CCW/SU_NEG_THRUST/SLOPE", startup_config[CCW][NEG_SLOPE]);
  PWMController::LoadParam<float>("CCW/SU_NEG_THRUST/YINT", startup_config[CCW][NEG_YINT]);

  // Primary Config (Linear Fit: y = mx + b)
  PWMController::LoadParam<float>("CW/POS_THRUST/SLOPE", primary_config[CW][POS_SLOPE]);
  PWMController::LoadParam<float>("CW/POS_THRUST/YINT", primary_config[CW][POS_YINT]);
  PWMController::LoadParam<float>("CW/NEG_THRUST/SLOPE", primary_config[CW][NEG_SLOPE]);
  PWMController::LoadParam<float>("CW/NEG_THRUST/YINT", primary_config[CW][NEG_YINT]);

  PWMController::LoadParam<float>("CCW/POS_THRUST/SLOPE", primary_config[CCW][POS_SLOPE]);
  PWMController::LoadParam<float>("CCW/POS_THRUST/YINT", primary_config[CCW][POS_YINT]);
  PWMController::LoadParam<float>("CCW/NEG_THRUST/SLOPE", primary_config[CCW][NEG_SLOPE]);
  PWMController::LoadParam<float>("CCW/NEG_THRUST/YINT", primary_config[CCW][NEG_YINT]);

  // Enable Status
  PWMController::LoadParam<bool>("ENABLE_SPL", enable[SPL]);
  PWMController::LoadParam<bool>("ENABLE_SSL", enable[SSL]);
  PWMController::LoadParam<bool>("ENABLE_SWA", enable[SWA]);
  PWMController::LoadParam<bool>("ENABLE_SWF", enable[SWF]);
  PWMController::LoadParam<bool>("ENABLE_HPF", enable[HPF]);
  PWMController::LoadParam<bool>("ENABLE_HSF", enable[HSF]);
  PWMController::LoadParam<bool>("ENABLE_HPA", enable[HPA]);
  PWMController::LoadParam<bool>("ENABLE_HSA", enable[HSA]);

  alive_timeout = ros::Duration(2);
  last_alive_time = ros::Time::now();
  silent = true;    // Silent refers to not receiving commands from the control stack
  dead = true;      // Dead refers to the kill switch being pulled
  reset_pwm = true; // Refers to controller being reset via reset command
}

// Load parameter from namespace
template <typename T>
void PWMController::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch (int e)
  {
    string ns = nh.getNamespace();
    ROS_INFO("PWM Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void PWMController::ThrustCB(const riptide_msgs::ThrustStamped::ConstPtr &thrust)
{
  if (!dead && !reset_pwm)
  {
    msg.header.stamp = thrust->header.stamp;

    //msg.pwm.surge_port_lo = Thrust2pwm(thrust->force.surge_port_lo, SPL);
    //msg.pwm.surge_stbd_lo = Thrust2pwm(thrust->force.surge_stbd_lo, SSL);

    //msg.pwm.sway_fwd = Thrust2pwm(thrust->force.sway_fwd, SWF);
    //msg.pwm.sway_aft = Thrust2pwm(thrust->force.sway_aft, SWA);

    msg.pwm.heave_stbd_fwd = Thrust2pwm(thrust->force.heave_stbd_fwd, HSF);
    msg.pwm.heave_stbd_aft = Thrust2pwm(thrust->force.heave_stbd_aft, HSA);

    msg.pwm.heave_port_aft = Thrust2pwm(thrust->force.heave_port_aft, HPA);
    msg.pwm.heave_port_fwd = Thrust2pwm(thrust->force.heave_port_fwd, HPF);
    pwm_pub.publish(msg);
    last_alive_time = ros::Time::now();
    silent = false;
  }
}

void PWMController::SwitchCB(const riptide_msgs::SwitchState::ConstPtr &state)
{
  dead = !state->kill;
}

void PWMController::ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg)
{
  if (reset_msg->reset_pwm)
    reset_pwm = true;
  else
    reset_pwm = false;
}

int PWMController::Thrust2pwm(double raw_force, int thruster)
{
  int pwm = NEUTRAL_PWM;
  int type = thruster % 2;

  if (enable[thruster])
  {
    if (abs(raw_force) < critical_thrusts[type][MIN_THRUST])
      pwm = NEUTRAL_PWM;

    else if (raw_force > 0 && raw_force <= critical_thrusts[type][STARTUP_THRUST]) // +Startup Thrust
      pwm = (int)(startup_config[type][POS_SLOPE] * raw_force + startup_config[type][POS_YINT]) + OFFSET;

    else if (raw_force > 0 && raw_force > critical_thrusts[type][STARTUP_THRUST]) // +Thrust
      pwm = (int)(primary_config[type][POS_SLOPE] * raw_force + primary_config[type][POS_YINT]) + OFFSET;

    else if (raw_force < 0 && raw_force >= -critical_thrusts[type][STARTUP_THRUST]) // -Startup Thrust
      pwm = (int)(startup_config[type][NEG_SLOPE] * raw_force + startup_config[type][NEG_YINT]) + OFFSET;

    else if (raw_force < 0 && raw_force < -critical_thrusts[type][STARTUP_THRUST]) // -Thrust
      pwm = (int)(primary_config[type][NEG_SLOPE] * raw_force + primary_config[type][NEG_YINT]) + OFFSET;

    else
      pwm = NEUTRAL_PWM;
  }

  // Constrain pwm output due to physical limitations of the ESCs
  if (pwm > MAX_PWM)
    pwm = MAX_PWM;
  if (pwm < MIN_PWM)
    pwm = MIN_PWM;

  return pwm;
}

void PWMController::PublishZeroPWM()
{
  msg.header.stamp = ros::Time::now();

  //msg.pwm.surge_port_lo = NEUTRAL_PWM;
  //msg.pwm.surge_stbd_lo = NEUTRAL_PWM;
  //msg.pwm.sway_fwd = NEUTRAL_PWM;
  //msg.pwm.sway_aft = NEUTRAL_PWM;
  msg.pwm.heave_stbd_fwd = NEUTRAL_PWM;
  msg.pwm.heave_stbd_aft = NEUTRAL_PWM;
  msg.pwm.heave_port_aft = NEUTRAL_PWM;
  msg.pwm.heave_port_fwd = NEUTRAL_PWM;
  pwm_pub.publish(msg);
}

void PWMController::Loop()
{
  // IMPORTANT: You MUST have a delay in publishing pwm msgs because copro
  // can only process data so fast
  ros::Rate rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration quiet_time = ros::Time::now() - last_alive_time;
    if (quiet_time >= alive_timeout)
    {
      silent = true;
    }

    if (silent || dead || reset_pwm)
    {
      PWMController::PublishZeroPWM();
    }
    rate.sleep();
  }
}
