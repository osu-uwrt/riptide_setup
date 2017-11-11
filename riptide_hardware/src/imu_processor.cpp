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
   cycles = 0;
 }

 //Callback
  void IMUProcessor::callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg)
  {
    //Put message data into state0
    state0.header = filter_msg->header;
    state0.header.frame_id = "base_link";
    state0.raw_euler_rpy = filter_msg->euler_rpy;
    state0.euler_rpy = filter_msg->euler_rpy;
    state0.gyro_bias = filter_msg->gyro_bias;
    state0.euler_rpy_status = filter_msg->euler_rpy_status;
    state0.heading = filter_msg->heading;
    state0.heading_uncertainty = filter_msg->heading_uncertainty;
    state0.heading_update_source = filter_msg->heading_update_source;
    state0.heading_flags = filter_msg->heading_flags;
    state0.linear_acceleration = filter_msg->linear_acceleration;
    state0.linear_acceleration_status = filter_msg->linear_acceleration_status;
    state0.angular_velocity = filter_msg->angular_velocity;
    state0.angular_velocity_status = filter_msg->angular_velocity_status;

    //Convert angular values from radians to degrees
    state0.raw_euler_rpy.x *= (180.0/PI);
    state0.raw_euler_rpy.y *= (180.0/PI);
    state0.raw_euler_rpy.z *= (180.0/PI);
    state0.euler_rpy.x *= (180.0/PI);
    state0.euler_rpy.y *= (180.0/PI);
    state0.euler_rpy.z *= (180.0/PI);
    state0.gyro_bias.x *= (180.0/PI);
    state0.gyro_bias.y *= (180.0/PI);
    state0.gyro_bias.z *= (180.0/PI);
    state0.heading *= (180/PI);
    state0.heading_uncertainty *= (180/PI);
    state0.angular_velocity.x *= (180.0/PI);
    state0.angular_velocity.y *= (180.0/PI);
    state0.angular_velocity.z *= (180.0/PI);

    //Set euler_rpy.z equal to heading
    state0.euler_rpy.z = state0.heading;

    //Adjust direction/sign of Euler Angles to be consistent with AUV's axes
    if(state0.euler_rpy.x > -180 && state0.euler_rpy.x < 0) {
      state0.euler_rpy.x += 180;
      state0.raw_euler_rpy.x += 180;
    }
    else if(state0.euler_rpy.x > 0 && state0.euler_rpy.x < 180) {
      state0.euler_rpy.x -= 180;
      state0.raw_euler_rpy.x -= 180;
    }
    else if(state0.euler_rpy.x == 0) {
      state0.euler_rpy.x = 180;
      state0.raw_euler_rpy.x = 180;
    }
    else if(state0.euler_rpy.x == 180 || state0.euler_rpy.x == -180) {
      state0.euler_rpy.x = 0;
      state0.raw_euler_rpy.x = 0;
    }
    state0.euler_rpy.z *= -1; //Negate Euler yaw angle
    state0.raw_euler_rpy.z *= -1; //Negate raw Euler yaw angle

    //Set new delta time (convert nano seconds to seconds)
    if(state1.dt == 0) { //If dt[0] not set yet, set to 0.01s, which is roughly what it would be
      state0.dt = 0.01;
    }
    else { //dt has already been set, so find actual dt
      state0.dt = (double)state0.header.stamp.sec + state0.header.stamp.nsec/(1.0e9) -
                    (double)state1.header.stamp.sec - state1.header.stamp.nsec/(1.0e9);
    }

    //Process Drift
    //Check if angular velocity about any axis is approximately 0
    if(abs(state0.angular_velocity.x) <= zero_ang_vel_thresh) {
      state0.local_drift.x = state0.euler_rpy.x - state1.euler_rpy.x;
    }
    else {
      state0.local_drift.x = 0;
    }
    state0.cumulative_drift.x += (double)abs(state0.local_drift.x);
    state0.local_drift_rate.x = state0.local_drift.x / state0.dt;

    if(abs(state0.angular_velocity.y) <= zero_ang_vel_thresh) {
      state0.local_drift.y = state0.euler_rpy.y - state1.euler_rpy.y;
    }
    else {
      state0.local_drift.y = 0;
    }
    state0.cumulative_drift.y += (double)abs(state0.local_drift.y);
    state0.local_drift_rate.y = state0.local_drift.y / state0.dt;

    if(abs(state0.angular_velocity.z) <= zero_ang_vel_thresh) {
      state0.local_drift.z = state0.euler_rpy.z - state1.euler_rpy.z;    }
    else {
      state0.local_drift.z = 0;
    }
    state0.cumulative_drift.z += (double)abs(state0.local_drift.z);
    state0.local_drift_rate.z = state0.local_drift.z / state0.dt;

    //Process linear acceleration (Remove centrifugal and tangential components)


    //Process angular acceleration (Use 3-pt backwards rule to approximate angular acceleration)
    if(cycles >= 2) { //Need 3 data points (state2 must have values)
      //ROS_INFO("Cycles: %i", cycles);
      float a, b; //Coefficients for backwards-diff rule

      float c = state0.dt, d = state1.dt;
      //ROS_INFO("state1.dt: %f", c);
      //ROS_INFO("state2.dt: %f", d);

      a = -1.0/c - 1/(2*d-c);
      b = c/(4*d*d-2*c*d);

      //Estimated angular acceleration to error order (dt)^2
      state0.angular_acceleration.x = a*state1.angular_velocity.x + b*state2.angular_velocity.x - (a+b)*state0.angular_velocity.x;
      state0.angular_acceleration.y = a*state1.angular_velocity.y + b*state2.angular_velocity.y - (a+b)*state0.angular_velocity.y;
      state0.angular_acceleration.z = a*state1.angular_velocity.z + b*state2.angular_velocity.z - (a+b)*state0.angular_velocity.z;
    }

    //Publish message for current state, adjust states
    state2 = state1;
    state1 = state0;
    imu_state_pub.publish(state0);

    if(cycles < 2) {
      cycles += 1;
    }
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
