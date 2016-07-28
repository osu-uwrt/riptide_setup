#include "ros/ros.h"
#include "riptide_msgs/ThrustStamped.h"
#include "riptide_msgs/PwmStamped.h"
#include "riptide_msgs/Bat.h"
#include "std_msgs/Empty.h"

class ThrustCal
{
  private:
    ros::NodeHandle nh;
    ros::Subscriber kill_it_with_fire;
    ros::Subscriber thrust, outta_juice;
    ros::Publisher pwm;
    riptide_msgs::PwmStamped us;
    ros::Time alive;
    bool dead, low;
    double min_voltage;
    int counterclockwise(double raw_force);
    int clockwise(double raw_force);

  public:
    ThrustCal();
    void callback(const riptide_msgs::ThrustStamped::ConstPtr& thrust);
    void killback(const std_msgs::Empty::ConstPtr& thrust);
    void voltsbacken(const riptide_msgs::Bat::ConstPtr& bat_stat);
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
  low = false;

  nh.param<double>("battery/min_voltage", min_voltage, 17.5);

  thrust = nh.subscribe<riptide_msgs::ThrustStamped>("command/thrust", 1, &ThrustCal::callback, this);
  kill_it_with_fire = nh.subscribe<std_msgs::Empty>("state/kill", 1, &ThrustCal::killback, this);
  outta_juice = nh.subscribe<riptide_msgs::Bat>("state/batteries", 1, &ThrustCal::voltsbacken, this);
  pwm = nh.advertise<riptide_msgs::PwmStamped>("command/pwm", 1);
}

void ThrustCal::callback(const riptide_msgs::ThrustStamped::ConstPtr& thrust)
{
  us.header.stamp = thrust->header.stamp;

  us.pwm.surge_port_hi = clockwise(thrust->force.surge_port_hi);
  us.pwm.surge_stbd_hi = counterclockwise(thrust->force.surge_stbd_hi);
  us.pwm.surge_port_lo = counterclockwise(thrust->force.surge_port_lo);
  us.pwm.surge_stbd_lo = clockwise(thrust->force.surge_stbd_lo);
  us.pwm.sway_fwd = counterclockwise(thrust->force.sway_fwd);
  us.pwm.sway_aft = clockwise(thrust->force.sway_aft);
  us.pwm.heave_port_fwd = counterclockwise(thrust->force.heave_port_fwd);
  us.pwm.heave_stbd_fwd = clockwise(thrust->force.heave_stbd_fwd);
  us.pwm.heave_port_aft = clockwise(thrust->force.heave_port_aft);
  us.pwm.heave_stbd_aft = counterclockwise(thrust->force.heave_stbd_aft);

  pwm.publish(us);
}

void ThrustCal::killback(const std_msgs::Empty::ConstPtr& thrust)
{
  if(!low)
  {
    dead = false;
  }
  alive = ros::Time::now();
}

void ThrustCal::voltsbacken(const riptide_msgs::Bat::ConstPtr& bat_stat)
{
  if(bat_stat->voltage < min_voltage)
  {
    low = true;
  }
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

int ThrustCal::counterclockwise(double raw_force)
{
  int pwm = 1500;
  if(!dead)
  {
    pwm = 1500 + int(raw_force * 14);
  }
  return pwm;
}

int ThrustCal::clockwise(double raw_force)
{
  int pwm = 1500;

  if(!dead)
  {
    pwm = 1500 - int(raw_force * 14);
  }

  return pwm;
}
