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

#ifndef THRUST_CAL_H
#define THRUST_CAL_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"

#include "riptide_msgs/Bat.h"
#include "riptide_msgs/PwmStamped.h"
#include "riptide_msgs/ThrustStamped.h"

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

#endif
