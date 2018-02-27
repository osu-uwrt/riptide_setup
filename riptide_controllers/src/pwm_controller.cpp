#include "riptide_controllers/pwm_controller.h"

int main(int argc, char** argv)
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

}

void ThrustCal::callback(const riptide_msgs::ThrustStamped::ConstPtr& thrust)
{
  us.header.stamp = thrust->header.stamp;

  us.pwm.surge_port_lo = counterclockwise(thrust->force.surge_port_lo, 0);
  us.pwm.surge_stbd_lo = clockwise(thrust->force.surge_stbd_lo, 0);

  us.pwm.sway_fwd = counterclockwise(thrust->force.sway_fwd, 1);
  us.pwm.sway_aft = clockwise(thrust->force.sway_aft, 1);

  us.pwm.heave_stbd_fwd = clockwise(thrust->force.heave_stbd_fwd, 2);
  us.pwm.heave_stbd_aft = counterclockwise(thrust->force.heave_stbd_aft, 2);

  us.pwm.heave_port_aft = clockwise(thrust->force.heave_port_aft, 3);
  us.pwm.heave_port_fwd = counterclockwise(thrust->force.heave_port_fwd, 3);
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
  while (!ros::isShuttingDown())
  {
    ros::Duration timeout = ros::Time::now() - alive;
    if (timeout > safe)
    {
      dead = true;
    }
    ros::spinOnce();
    rate.sleep();
  }
}

int ThrustCal::counterclockwise(double raw_force, int thruster)
{
  int pwm = 1500;
  if(raw_force<0){
    pwm = 1500 + static_cast<int>(raw_force*ccw_coeffs[thruster][0]);
  }else if(raw_force>0){
    pwm = (int) (1500 + (raw_force*ccw_coeffs[thruster][1]));
  }else{
    pwm = 1500;
  }
  return pwm;
}

int ThrustCal::clockwise(double raw_force, int thruster)
{
  int pwm = 1500;
  if(raw_force<0){
    pwm = 1500 + static_cast<int>(raw_force*cw_coeffs[thruster][0]);
  }else if(raw_force>0){
    pwm = 1500 + static_cast<int>(raw_force*cw_coeffs[thruster][1]);
  }else{
    pwm = 1500;
  }

  return pwm;
}
