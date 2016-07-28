#include "ros/ros.h"
#include "riptide_msgs/ThrustStamped.h"
#include "riptide_msgs/PwmStamped.h"
#include "std_msgs/Empty.h"

class ThrustCal
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber kill_it_with_fire;
    ros::Subscriber thrust;
    ros::Publisher pwm;
    riptide_msgs::PwmStamped us;
    double s_p_h_fw, s_p_h_rv;
    double s_s_h_fw, s_s_h_rv;
    double s_p_l_fw, s_p_l_rv;
    double s_s_l_fw, s_s_l_rv;
    double swa_f_fw, swa_f_rv;
    double swa_a_fw, swa_a_rv;
    double h_p_f_fw, h_p_f_rv;
    double h_s_f_fw, h_s_f_rv;
    double h_p_a_fw, h_p_a_rv;
    double h_s_a_fw, h_s_a_rv;
    ros::Time alive;
    bool dead;
    int cal(double raw_force, double fwd_scale, double rev_scale);
    int cal2(double raw_force, double fwd_scale, double rev_scale);

  public:
    ThrustCal();
    void callback(const riptide_msgs::ThrustStamped::ConstPtr& thrust);
    void killback(const std_msgs::Empty::ConstPtr& thrust);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thruster_calibration");
  ThrustCal thrust_cal;
  thrust_cal.loop();
}

ThrustCal::ThrustCal() : nh()
{
  alive = ros::Time::now();
  dead = false;

  thrust = nh.subscribe<riptide_msgs::ThrustStamped>("command/thrust", 1, &ThrustCal::callback, this);
  kill_it_with_fire = nh.subscribe<std_msgs::Empty>("state/kill", 1, &ThrustCal::killback, this);
  pwm = nh.advertise<riptide_msgs::PwmStamped>("command/pwm", 1);

  nh.param<double>("calibration/thruster/surge_port_hi_fw", s_p_h_fw, 1.0);
  nh.param<double>("calibration/thruster/surge_port_hi_rv", s_p_h_rv, 1.0);
  nh.param<double>("calibration/thruster/surge_stbd_hi_fw", s_s_h_fw, 1.0);
  nh.param<double>("calibration/thruster/surge_stbd_hi_rv", s_s_h_rv, 1.0);
  nh.param<double>("calibration/thruster/surge_port_lo_fw", s_p_l_fw, 1.0);
  nh.param<double>("calibration/thruster/surge_port_lo_rv", s_p_l_rv, 1.0);
  nh.param<double>("calibration/thruster/surge_stbd_lo_fw", s_s_l_fw, 1.0);
  nh.param<double>("calibration/thruster/surge_stbd_lo_rv", s_s_l_rv, 1.0);
  nh.param<double>("calibration/thruster/sway_fwd_fw", swa_f_fw, 1.0);
  nh.param<double>("calibration/thruster/sway_fwd_rv", swa_f_rv, 1.0);
  nh.param<double>("calibration/thruster/sway_aft_fw", swa_a_fw, 1.0);
  nh.param<double>("calibration/thruster/sway_aft_rv", swa_a_rv, 1.0);
  nh.param<double>("calibration/thruster/heave_port_fwd_fw", h_p_f_fw, 1.0);
  nh.param<double>("calibration/thruster/heave_port_fwd_rv", h_p_f_rv, 1.0);
  nh.param<double>("calibration/thruster/heave_stbd_fwd_fw", h_s_f_fw, 1.0);
  nh.param<double>("calibration/thruster/heave_stbd_fwd_rv", h_s_f_rv, 1.0);
  nh.param<double>("calibration/thruster/heave_port_aft_fw", h_p_a_fw, 1.0);
  nh.param<double>("calibration/thruster/heave_port_aft_rv", h_p_a_rv, 1.0);
  nh.param<double>("calibration/thruster/heave_stbd_aft_fw", h_s_a_fw, 1.0);
  nh.param<double>("calibration/thruster/heave_stbd_aft_rv", h_s_a_rv, 1.0);
}

void ThrustCal::callback(const riptide_msgs::ThrustStamped::ConstPtr& thrust)
{
  us.header.stamp = thrust->header.stamp;

  us.pwm.surge_port_hi = cal2(thrust->force.surge_port_hi, s_p_h_fw, s_p_h_rv);
  us.pwm.surge_stbd_hi = cal(thrust->force.surge_stbd_hi, s_s_h_fw, s_s_h_rv);
  us.pwm.surge_port_lo = cal(thrust->force.surge_port_lo, s_p_l_fw, s_p_l_rv);
  us.pwm.surge_stbd_lo = cal2(thrust->force.surge_stbd_lo, s_s_l_fw, s_s_l_rv);
  us.pwm.sway_fwd = cal(thrust->force.sway_fwd, swa_f_fw, swa_f_rv);
  us.pwm.sway_aft = cal2(thrust->force.sway_aft, swa_a_fw, swa_a_rv);
  us.pwm.heave_port_fwd = cal(thrust->force.heave_port_fwd, h_p_f_fw, h_p_f_rv);
  us.pwm.heave_stbd_fwd = cal2(thrust->force.heave_stbd_fwd, h_s_f_fw, h_s_f_rv);
  us.pwm.heave_port_aft = cal2(thrust->force.heave_port_aft, h_p_a_fw, h_p_a_rv);
  us.pwm.heave_stbd_aft = cal(thrust->force.heave_stbd_aft, h_s_a_fw, h_s_a_rv);

  pwm.publish(us);
}

void ThrustCal::killback(const std_msgs::Empty::ConstPtr& thrust)
{
  dead = false;
  alive = ros::Time::now();
}

void ThrustCal::loop()
{
  ros::Duration safe(0.05);
  ros::Rate rate(50);
  while(!ros::isShuttingDown())
  {
    ros::Duration timeout = ros::Time::now() - alive;
    if(timeout > safe)
    {
      dead = true;
    }
    ros::spinOnce();
    rate.sleep();
  }
}

int ThrustCal::cal(double raw_force, double fwd_scale, double rev_scale)
{
  int pwm = 1500;
  if(!dead)
  {
    pwm = 1500 + int(raw_force * 14);
  }
  return pwm;
}

int ThrustCal::cal2(double raw_force, double fwd_scale, double rev_scale)
{
  int pwm = 1500;

  if(!dead)
  {
    pwm = 1500 - int(raw_force * 14);
  }

  return pwm;
}
