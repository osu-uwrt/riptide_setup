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

#include "riptide_teleop/commander.h"

Accel::Accel()
{
  js = nh.subscribe<sensor_msgs::Joy>("joy", 1, &Accel::joy_callback, this);
  target_odom = nh.advertise<riptide_msgs::OdomWithAccel>("target/position", 1);

  target.header.frame_id = "";
  target.pose.position.x = 0;
  target.pose.position.y = 0;
  target.pose.position.z = 0;
  target.twist.linear.x = 0;
  target.twist.linear.y = 0;
  target.twist.linear.z = 0;
  target.twist.angular.x = 0;
  target.twist.angular.y = 0;
  target.twist.angular.z = 0;
  target.accel.angular.x = 0;
  target.accel.angular.y = 0;
  target.accel.angular.z = 0;
  target.accel.linear.z = 0;
}

void Accel::joy_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  target.accel.linear.x = 0.75 * joy->axes[1];
  target.accel.linear.y = joy->axes[0];
  target.pose.position.z += (joy->axes[14] - joy->axes[15]) / 100;

  double roll = 3.14159 / 4 * joy->axes[2] * -1;
  double pitch = 3.14159 / 4 * joy->axes[3];
  double yaw = 3.14159 * (joy->axes[13] - joy->axes[12]);

  target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  target_odom.publish(target);
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
