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

 #include "riptide_hardware/imu_processor.h"

 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "imu_processor");
   IMUProcessor imu(argv);
   imu.loop();
 }

//Constructor
 IMUProcessor::IMUProcessor(char **argv) : nh()
 {
   imu_filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("imu/filter", 1, &IMUProcessor::callback, this);
   imu_state_pub = nh.advertise<riptide_msgs::Imu>("state/imu", 1);
 }

 //Callback
  void IMUProcessor::callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg)
  {
    //Take in message into state[0]
    state[0].header = filter_msg->header;
    state[0].header.frame_id = "base_link";
    state[0].euler_rpy = filter_msg->euler_rpy;
    state[0].euler_rpy_status = filter_msg->euler_rpy_status;
    state[0].linear_acceleration = filter_msg->linear_acceleration;
    state[0].linear_acceleration_status = filter_msg->linear_acceleration_status;
    state[0].angular_velocity = filter_msg->angular_velocity;
    state[0].angular_velocity_status = filter_msg->angular_velocity_status;

    //Set new delta time (nano seconds)
    if(dt[1] != 0) {
      dt[0] = state[0].header.stamp.nsec - dt[1];
    }

    //Process linear acceleration (Remove centrifugal and tangential components)

    //Process angular acceleration (Use 3-pt backwards rule to approximate angular acceleration)

    //Publish message for current state, adjust states and dt arrays
    imu_state_pub.publish(state[0]);
    state[2] = state[1];
    state[1] = state[0];
    dt[1] = dt[0]; //Will set dt[0] upon receiving new message
  }

//ROS loop function
 void IMUProcessor::loop()
 {
   ros::Rate rate(1000);
   while (!ros::isShuttingDown())
   {
     ros::spinOnce();
     rate.sleep();
   }
 }
