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

 #include "riptide_hardware/imu_drift_logger.h"

 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "imu_drift_logger");
   IMUDriftLogger imu_drift_logger(argv);
   imu_drift_logger.loop();
   ROS_INFO("IMU drift logger looping");
 }

//Converts std::string to char*
char* IMUDriftLogger::convert(const std::string& str) {
   char* result = new char[str.length()+1];
   strcpy(result,str.c_str());
   return result;
 }

//Constructor
 IMUDriftLogger::IMUDriftLogger(char **argv) : nh()
 {
   imu_state_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1, &IMUDriftLogger::callback, this);

   //Create a new file for logging IMU drift data
   bool found_new_file_name = false;
   int num = 1; //Begin file counting here
   std::string file_suffix = boost::lexical_cast<std::string>(num); //Convert int to string
   std::string file_prefix = "//home//tsender//osu-uwrt//imu_drift_logger_"; //File path included in prefix
   std::string file_type = ".csv";

   //Create file name
   char *file_name = strcat(convert(file_prefix),convert(file_suffix)); //Combine prefix and suffix
   file_name = strcat(file_name,convert(file_type)); //Append file type
   file_name_c = file_name; //Convert to const char*

   //If unable to open for reading, then the file does not exist --> new file_name
   while(!found_new_file_name) {
     ROS_INFO("Drift Logger File Name:");
     ROS_INFO("\t%s", file_name);
     fid = fopen(file_name_c,"r");
      if(fid) {
        //Increase num, and create new file name
        num++;
        file_suffix = boost::lexical_cast<std::string>(num);
        file_name = strcat(convert(file_prefix),convert(file_suffix));
        file_name = strcat(file_name,convert(file_type));
        file_name_c = file_name;
      }
      else {
        found_new_file_name = true;
      }
   }
 }

 void IMUDriftLogger::callback(const riptide_msgs::Imu::ConstPtr& imu_msg) {
   //Store appropriate values from message
   dt = imu_msg->dt;
   angular_vel[0] = imu_msg->angular_velocity.x;
   angular_vel[1] = imu_msg->angular_velocity.y;
   angular_vel[2] = imu_msg->angular_velocity.z;
   angular_accel[0] = imu_msg->angular_acceleration.x;
   angular_accel[1] = imu_msg->angular_acceleration.y;
   angular_accel[2] = imu_msg->angular_acceleration.z;
   drift[0] = imu_msg->local_drift.x;
   drift[1] = imu_msg->local_drift.y;
   drift[2] = imu_msg->local_drift.z;
   drift_rate[0] = imu_msg->local_drift_rate.x;
   drift_rate[1] = imu_msg->local_drift_rate.y;
   drift_rate[2] = imu_msg->local_drift_rate.z;

   //Open file and print values
   fid = fopen(file_name_c,"a"); //Open file for "appending"
   if(!fid) {
     ROS_INFO("Drift Logger: file not opened");
   }
   fprintf(fid,"%f,%f,%f,%f,", dt,angular_vel[0],angular_vel[1],angular_vel[2]);
   fprintf(fid,"%f,%f,%f,", angular_accel[0],angular_accel[1],angular_accel[2]);
   fprintf(fid,"%f,%f,%f,", drift[0],drift[1],drift[2]);
   fprintf(fid,"%f,%f,%f\n", drift_rate[0],drift_rate[1],drift_rate[2]);
   fclose(fid);
 }

 //ROS loop function
  void IMUDriftLogger::loop()
  {
    ros::Rate rate(1000);
    while (!ros::isShuttingDown())
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
