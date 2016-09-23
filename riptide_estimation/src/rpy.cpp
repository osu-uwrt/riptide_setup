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

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"

ros::Publisher msg_;
ros::Subscriber imu_;
geometry_msgs::Vector3Stamped rpy;

void callback(const sensor_msgs::Imu::ConstPtr &imu)
{
  tf::Quaternion quaternion;
  tf::quaternionMsgToTF(imu->orientation, quaternion);
  tf::Matrix3x3 attitude = tf::Matrix3x3(quaternion);
  attitude.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
  rpy.header.stamp = imu->header.stamp;
  msg_.publish(rpy);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orientation");
  ros::NodeHandle nh;
  imu_ = nh.subscribe<sensor_msgs::Imu>("state/imu", 1, callback);
  msg_ = nh.advertise<geometry_msgs::Vector3Stamped>("state/rpy", 1);
  rpy.header.frame_id = "base_link";
  ros::spin();
}
