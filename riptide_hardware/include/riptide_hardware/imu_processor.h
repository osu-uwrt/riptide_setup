/*********************************************************************************
 *  Copyright (c) 2017, The Underwater Robotics Team
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

#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "ros/ros.h"
//#include "message_filters/subscriber.h"
//#include "message_filters/synchronizer.h"
//#include "message_filters/sync_policies/approximate_time.h"
//#include "tf/transform_broadcaster.h"
#include "riptide_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "math.h"

class IMUProcessor
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_filter_sub;
  ros::Publisher imu_state_pub;
  int cycles;

  //0 = current state, 1 = one state ago, 2 = two states ago
  riptide_msgs::Imu state0;
  riptide_msgs::Imu state1;
  riptide_msgs::Imu state2;
  float zero_ang_vel_thresh;
public:
  IMUProcessor(char **argv);
  void callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg);
  void loop();
};

#endif
