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
#define MIN_PWM 1300
#define MAX_PWM 1700
#define MIN_THRUST 0.15

int main(int argc, char** argv)
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

  //Initialization of the two trust/pwm slope arrays
  //The first column is for negative forces, second column is positive force

  // Surge Port Low
  LoadCalibration("SPL/NEG/SLOPE", thrust_config[SPL][NEG_SLOPE]);
  LoadCalibration("SPL/POS/SLOPE", thrust_config[SPL][POS_SLOPE]);
  LoadCalibration("SPL/NEG/XINT", thrust_config[SPL][NEG_XINT]);
  LoadCalibration("SPL/POS/XINT", thrust_config[SPL][POS_XINT]);
  // Surge Starboard Low
  LoadCalibration("SSL/NEG/SLOPE", thrust_config[SSL][NEG_SLOPE]);
  LoadCalibration("SSL/POS/SLOPE", thrust_config[SSL][POS_SLOPE]);
  LoadCalibration("SSL/NEG/XINT", thrust_config[SSL][NEG_XINT]);
  LoadCalibration("SSL/POS/XINT", thrust_config[SSL][POS_XINT]);
  // Heave Port Aft
  LoadCalibration("HPA/NEG/SLOPE", thrust_config[HPA][NEG_SLOPE]);
  LoadCalibration("HPA/POS/SLOPE", thrust_config[HPA][POS_SLOPE]);
  LoadCalibration("HPA/NEG/XINT", thrust_config[HPA][NEG_XINT]);
  LoadCalibration("HPA/POS/XINT", thrust_config[HPA][POS_XINT]);
  // heave Port Forward
  LoadCalibration("HPF/NEG/SLOPE", thrust_config[HPF][NEG_SLOPE]);
  LoadCalibration("HPF/POS/SLOPE", thrust_config[HPF][POS_SLOPE]);
  LoadCalibration("HPF/NEG/XINT", thrust_config[HPF][NEG_XINT]);
  LoadCalibration("HPF/POS/XINT", thrust_config[HPF][POS_XINT]);
  // Heave Starboard Aft
  LoadCalibration("HSA/NEG/SLOPE", thrust_config[HSA][NEG_SLOPE]);
  LoadCalibration("HSA/POS/SLOPE", thrust_config[HSA][POS_SLOPE]);
  LoadCalibration("HSA/NEG/XINT", thrust_config[HSA][NEG_XINT]);
  LoadCalibration("HSA/POS/XINT", thrust_config[HSA][POS_XINT]);
  // Heave Starboard Forward
  LoadCalibration("HSF/NEG/SLOPE", thrust_config[HSF][NEG_SLOPE]);
  LoadCalibration("HSF/POS/SLOPE", thrust_config[HSF][POS_SLOPE]);
  LoadCalibration("HSF/NEG/XINT", thrust_config[HSF][NEG_XINT]);
  LoadCalibration("HSF/POS/XINT", thrust_config[HSF][POS_XINT]);
  // Sway Forward
  LoadCalibration("SWF/NEG/SLOPE", thrust_config[SWF][NEG_SLOPE]);
  LoadCalibration("SWF/POS/SLOPE", thrust_config[SWF][POS_SLOPE]);
  LoadCalibration("SWF/NEG/XINT", thrust_config[SWF][NEG_XINT]);
  LoadCalibration("SWF/POS/XINT", thrust_config[SWF][POS_XINT]);
  // Sway Aft
  LoadCalibration("SWA/NEG/SLOPE", thrust_config[SWA][NEG_SLOPE]);
  LoadCalibration("SWA/POS/SLOPE", thrust_config[SWA][POS_SLOPE]);
  LoadCalibration("SWA/NEG/XINT", thrust_config[SWA][NEG_XINT]);
  LoadCalibration("SWA/POS/XINT", thrust_config[SWA][POS_XINT]);

  alive_timeout = ros::Duration(2);
  last_alive_time = ros::Time::now();
  silent = true; // Silent refers to not receiving commands from the control stack
  dead = true; // Dead refers to the kill switch being pulled
}

void PWMController::LoadCalibration(std::string name, float &param)
{
  try
  {
    if (!nh.getParam(name, param))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    ROS_ERROR("Critical! PWM Controler has no calibration set for %s. Shutting down...", name.c_str());
    ros::shutdown();
  }
}

void PWMController::ThrustCB(const riptide_msgs::ThrustStamped::ConstPtr& thrust)
{
  if (!dead && !silent)
  {
    msg.header.stamp = thrust->header.stamp;

    msg.pwm.surge_port_lo = Thrust2pwm(thrust->force.surge_port_lo, SPL);
    msg.pwm.surge_stbd_lo = Thrust2pwm(thrust->force.surge_stbd_lo, SSL);

    msg.pwm.sway_fwd = Thrust2pwm(thrust->force.sway_fwd, SWF);
    msg.pwm.sway_aft = Thrust2pwm(thrust->force.sway_aft, SWA);

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

void PWMController::ResetController(const riptide_msgs::ResetControls::ConstPtr &reset_msg) {
  if(reset_msg->reset_pwm)
    silent = true;
  else
    silent = false;
}


int PWMController::Thrust2pwm(double raw_force, int thruster)
{
  int pwm = 1500;
  // If force is negative, use negative calibration.
  // If force is positive, use positive calibration
  // Otherwise, set PWM to 1500 (0 thrust)
  if(raw_force < -MIN_THRUST)
  {
    pwm = thrust_config[thruster][NEG_XINT] + static_cast<int>(raw_force*thrust_config[thruster][NEG_SLOPE]);
  }
  else if(raw_force > MIN_THRUST){
    pwm = (int) (thrust_config[thruster][POS_XINT] + (raw_force*thrust_config[thruster][POS_SLOPE]));
  }
  else{
    pwm = 1500;
  }

  // Constrain pwm output
  // The thruster controller should not be the one constraining the output, it
  // really should be the pwm controller.
  if(pwm > MAX_PWM)
    pwm = MAX_PWM;
  if(pwm < MIN_PWM)
    pwm = MIN_PWM;

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

    if (silent || dead)
    {
      PWMController::PublishZeroPWM();
    }
    rate.sleep();
  }
}
