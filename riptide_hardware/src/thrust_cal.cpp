/*********************************************************************************
 *  Copyright (c) 2015, The Underwater Robotics Team
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include "riptide_calibration/thrust_cal.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "thruster_calibration");
  ThrustCal thrust_cal;
  thrust_cal.loop();
}

ThrustCal::ThrustCal() : nh()
{
  thrust = nh.subscribe<riptide_msgs::ThrustStamped>("command/thrust", 1, &ThrustCal::callback, this);
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

void ThrustCal::loop()
{
  ros::Rate rate(50);
  while (!ros::isShuttingDown())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

int ThrustCal::counterclockwise(double raw_force)
{
  int pwm = 1500;
  pwm = 1500 + static_cast<int>(raw_force * 25);
  return pwm;
}

int ThrustCal::clockwise(double raw_force)
{
  int pwm = 1500;
  pwm = 1500 - static_cast<int>(raw_force * 25);

  return pwm;
}
