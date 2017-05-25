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

#include "riptide_teleop/accel_cmd.h"

#undef zero

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_accel");
  Accel accel;
  accel.loop();
}

Accel::Accel()
{
  js = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Accel::joy_callback, this);
  accels = nh.advertise<geometry_msgs::Accel>("command/accel", 1);
}

void Accel::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
#ifdef zero
  accel.linear.x = 0;
  accel.linear.y = 0;
  accel.linear.z = 0;

  accel.angular.x = 0;
  accel.angular.y = 0;
  accel.angular.z = 0;
#else
  accel.linear.x = 0.75 * joy->axes[1];
  accel.linear.y = joy->axes[0];
  accel.linear.z = (joy->axes[14] - joy->axes[15]);

  accel.angular.x = 2.0 * 3.14159 * joy->axes[2] * -1;
  accel.angular.y = 1.2 * 3.14159 * joy->axes[3];
  accel.angular.z = 2.0 * 3.14159 * (joy->axes[13] - joy->axes[12]);
#endif
  // accel.linear.x = 5 * joy->axes[1];
  // accel.linear.y = 0;
  // accel.linear.z = 5 * (joy->axes[14] - joy->axes[15]);
  //
  // accel.angular.x = 5 * 2 * 3.14159 * joy->axes[2] * -1;
  // accel.angular.y = 5 * 2 * 3.14159 * joy->axes[3];
  // accel.angular.z = 5 * 2 * 3.14159 * (joy->axes[13] - joy->axes[12]);

  accels.publish(accel);
}

void Accel::loop()
{
  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
