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

 #define PI 3.141592653

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

   zero_ang_vel_thresh = 1;
 }

 //Callback
  void IMUProcessor::callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg)
  {
    //Put message data into state[0]
    state[0].header = filter_msg->header;
    state[0].header.frame_id = "base_link";
    state[0].raw_euler_rpy = filter_msg->euler_rpy;
    state[0].euler_rpy = filter_msg->euler_rpy;
    state[0].euler_rpy_status = filter_msg->euler_rpy_status;
    state[0].linear_acceleration = filter_msg->linear_acceleration;
    state[0].linear_acceleration_status = filter_msg->linear_acceleration_status;
    state[0].angular_velocity = filter_msg->angular_velocity;
    state[0].angular_velocity_status = filter_msg->angular_velocity_status;

    //Convert angular values from radians to degrees
    state[0].raw_euler_rpy.x *= (180/PI);
    state[0].raw_euler_rpy.y *= (180/PI);
    state[0].raw_euler_rpy.z *= (180/PI);
    state[0].euler_rpy.x *= (180/PI);
    state[0].euler_rpy.y *= (180/PI);
    state[0].euler_rpy.z *= (180/PI);
    state[0].angular_velocity.x *= (180/PI);
    state[0].angular_velocity.y *= (180/PI);
    state[0].angular_velocity.z *= (180/PI);

    //Adjust direction/sign of Euler Angles to be consistent with AUV's axes
    if(state[0].euler_rpy.x > -180 && state[0].euler_rpy.x < 0) {
      state[0].euler_rpy.x += 180;
      state[0].raw_euler_rpy.x += 180;
    }
    else if(state[0].euler_rpy.x > 0 && state[0].euler_rpy.x < 180) {
      state[0].euler_rpy.x -= 180;
      state[0].raw_euler_rpy.x -= 180;
    }
    else if(state[0].euler_rpy.x == 0) {
      state[0].euler_rpy.x = 180;
      state[0].raw_euler_rpy.x = 180;
    }
    else if(state[0].euler_rpy.x == 180 || state[0].euler_rpy.x == -180) {
      state[0].euler_rpy.x = 0;
      state[0].raw_euler_rpy.x = 0;
    }
    state[0].euler_rpy.z *= -1; //Negate yaw angle
    state[0].raw_euler_rpy.z *= -1;

    //Set new delta time (convert nano seconds to seconds)
    if(state[1].dt == 0) { //If dt[0] not set yet, set to time in current header stamp
      state[0].dt = (double)state[0].header.stamp.sec + state[0].header.stamp.nsec/(1.0e9);
    }
    else { //dt has already been set, so find actual dt
      state[0].dt = (double)state[0].header.stamp.sec + state[0].header.stamp.nsec/(1.0e9) - state[1].dt;
    }

    //Process Drift
    //Check if angular velocity about any axis is approximately 0
    if(abs(state[0].angular_velocity.x) <= zero_ang_vel_thresh) {
      state[0].local_drift.x = state[0].euler_rpy.x - state[1].euler_rpy.x;
    }
    else {
      state[0].local_drift.x = 0;
    }
    state[0].cumulative_drift.x = abs(state[1].cumulative_drift.x) + abs(state[0].local_drift.x);
    state[0].local_drift_rate.x = state[0].local_drift.x / state[0].dt;

    if(abs(state[0].angular_velocity.y) <= zero_ang_vel_thresh) {
      state[0].local_drift.y = state[0].euler_rpy.y - state[1].euler_rpy.y;
    }
    else {
      state[0].local_drift.y = 0;
    }
    state[0].cumulative_drift.y += abs(state[1].cumulative_drift.y) + abs(state[0].local_drift.y);
    state[0].local_drift_rate.y = state[0].local_drift.y / state[0].dt;

    if(abs(state[0].angular_velocity.z) <= zero_ang_vel_thresh) {
      state[0].local_drift.z = state[0].euler_rpy.z - state[1].euler_rpy.z;
    }
    else {
      state[0].local_drift.z = 0;
    }
    state[0].cumulative_drift.z += abs(state[1].cumulative_drift.z) + abs(state[0].local_drift.z);
    state[0].local_drift_rate.z = state[0].local_drift.z / state[0].dt;

    //Process linear acceleration (Remove centrifugal and tangential components)

    //Process angular acceleration (Use 3-pt backwards rule to approximate angular acceleration)

    //Publish message for current state, adjust states and dt arrays
    //imu_state_pub.publish(state[0]);
    state[2] = state[1];
    state[1] = state[0];
    imu_state_pub.publish(state[1]);
    //dt = dt; //Will set dt[0] upon receiving new message
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
