#include "ros/ros.h"
#include "riptide_msgs/ThrustStamped.h"
#include "riptide_msgs/PwmStamped.h"

class ThrustCal
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber ts;
    ros::Publisher pwm;
    riptide_msgs::PwmStamped duration;
    double max_force;
    double max_pwm;
    double f1_fwd,f2_fwd,f3_fwd,f4_fwd;
    double f1_rev,f2_rev,f3_rev,f4_rev;
    double u1_fwd,u2_fwd,u3_fwd,u4_fwd;
    double u1_rev,u2_rev,u3_rev,u4_rev;
    double s1_fwd,s2_fwd;
    double s1_rev,s2_rev;
    int calibrate(double raw_force, double fwd_cal, double rev_cal);

  public:
    ThrustCal();
    void callback(const riptide_msgs::ThrustStamped::ConstPtr& force);
    void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thrust_cal");
  ThrustCal thrust_cal;
  thrust_cal.loop();
}

ThrustCal::ThrustCal() : nh()
{
  ts = nh.subscribe<riptide_msgs::ThrustStamped>("solver/thrust", 1, &ThrustCal::callback, this);
  pwm = nh.advertise<riptide_msgs::PwmStamped>("thrust_cal/pwm", 1);

  nh.param<double>("nominal_max_force", max_force, 25.0);
  nh.param<double>("nominal_max_pwm", max_pwm, 100.0);
  nh.param<double>("thrust_cal/f1_fwd", f1_fwd, 1.0);
  nh.param<double>("thrust_cal/f1_rev", f1_rev, 1.0);
  nh.param<double>("thrust_cal/f2_fwd", f2_fwd, 1.0);
  nh.param<double>("thrust_cal/f2_rev", f2_rev, 1.0);
  nh.param<double>("thrust_cal/f3_fwd", f3_fwd, 1.0);
  nh.param<double>("thrust_cal/f3_rev", f3_rev, 1.0);
  nh.param<double>("thrust_cal/f4_fwd", f4_fwd, 1.0);
  nh.param<double>("thrust_cal/f4_rev", f4_rev, 1.0);
  nh.param<double>("thrust_cal/u1_fwd", u1_fwd, 1.0);
  nh.param<double>("thrust_cal/u1_rev", u1_rev, 1.0);
  nh.param<double>("thrust_cal/u2_fwd", u2_fwd, 1.0);
  nh.param<double>("thrust_cal/u2_rev", u2_rev, 1.0);
  nh.param<double>("thrust_cal/u3_fwd", u3_fwd, 1.0);
  nh.param<double>("thrust_cal/u3_rev", u3_rev, 1.0);
  nh.param<double>("thrust_cal/u4_fwd", u4_fwd, 1.0);
  nh.param<double>("thrust_cal/u4_rev", u4_rev, 1.0);
  nh.param<double>("thrust_cal/s1_fwd", s1_fwd, 1.0);
  nh.param<double>("thrust_cal/s1_rev", s1_rev, 1.0);
  nh.param<double>("thrust_cal/s2_fwd", s2_fwd, 1.0);
  nh.param<double>("thrust_cal/s2_rev", s2_rev, 1.0);
}

void ThrustCal::callback(const riptide_msgs::ThrustStamped::ConstPtr& force)
{
  duration.header.stamp = force->header.stamp;

  duration.pwm.f1 = calibrate(force->thrust.f1, f1_fwd, f1_rev);
  duration.pwm.f2 = calibrate(force->thrust.f2, f2_fwd, f2_rev);
  duration.pwm.f3 = calibrate(force->thrust.f3, f3_fwd, f3_rev);
  duration.pwm.f4 = calibrate(force->thrust.f4, f4_fwd, f4_rev);
  duration.pwm.u1 = calibrate(force->thrust.u1, u1_fwd, u1_rev);
  duration.pwm.u2 = calibrate(force->thrust.u2, u2_fwd, u2_rev);
  duration.pwm.u3 = calibrate(force->thrust.u3, u3_fwd, u3_rev);
  duration.pwm.u4 = calibrate(force->thrust.u4, u4_fwd, u4_rev);
  duration.pwm.s1 = calibrate(force->thrust.s1, s1_fwd, s1_rev);
  duration.pwm.s2 = calibrate(force->thrust.s2, s2_fwd, s2_rev);

  pwm.publish(duration);
}

void ThrustCal::loop()
{
  ros::spin();
}

int ThrustCal::calibrate(double raw_force, double fwd_cal, double rev_cal)
{
  if(raw_force < 0.0)
  {
    raw_force /= rev_cal;
  }
  else
  {
    raw_force /= fwd_cal;
  }
  return 1500 + int(raw_force * max_pwm / max_force);
}
