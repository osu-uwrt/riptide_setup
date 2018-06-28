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
//YOU MUST CHANGE THE "file_prefix" BEFORE USING


#include "riptide_hardware/imu_logger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_logger");
  IMULogger imu_logger(argv);
  imu_logger.loop();
  ROS_INFO("IMU logger looping");
}

//Converts std::string to char*
char* IMULogger::convert(const std::string& str) {
   char* result = new char[str.length()+1];
   strcpy(result,str.c_str());
   return result;
 }

//Constructor
 IMULogger::IMULogger(char **argv) : nh()
 {
   mag_sub = nh.subscribe<imu_3dm_gx4::MagFieldCF>("imu/magnetic_field", 1, &IMULogger::magLogger, this);
   initialized = false;
   tStart = 0, tNow = 0;

   //Create a new file for logging IMU data
   bool found_new_file_name = false;
   int num = 1; //Begin file counting here
   std::string file_suffix = boost::lexical_cast<std::string>(num); //Convert int to string
   std::string file_prefix = "//home//tsender//osu-uwrt//imu_mag_components"; //File path included in prefix
   std::string file_type = ".csv";

   //Create file name
   char *file_name = strcat(convert(file_prefix),convert(file_suffix)); //Combine prefix and suffix
   file_name = strcat(file_name,convert(file_type)); //Append file type
   file_name_c = file_name; //Convert to const char*

   //If unable to open for reading, then the file does not exist --> new file_name
   while(!found_new_file_name) {
     ROS_INFO("IMU Logger File Name:");
     ROS_INFO("\t%s", file_name);
     fid = fopen(file_name_c,"r");
      if(fid) { //File already exists
        //Increase num, and create new file name
        num++;
        file_suffix = boost::lexical_cast<std::string>(num);
        file_name = strcat(convert(file_prefix),convert(file_suffix));
        file_name = strcat(file_name,convert(file_type));
        file_name_c = file_name;
      }
      else { //File does not exist
        found_new_file_name = true;
      }
   }
 }

//Log magnetometer vector components
 void IMULogger::magLogger(const imu_3dm_gx4::MagFieldCF::ConstPtr& mag) {

   //Open file and print values
   fid = fopen(file_name_c,"a"); //Open file for "appending"
   if(!fid) {
     ROS_INFO("IMU Logger: file not opened");
   }

   if(!initialized) {
     tStart = ros::Time::now().toSec();
     fprintf(fid,"%f,", 0.0);
     initialized = true;
   } else {
     tNow = ros::Time::now().toSec();
     fprintf(fid,"%f,", tNow - tStart);
   }
   fprintf(fid,"%f,", mag->mag_field_components.x);
   fprintf(fid,"%f,", mag->mag_field_components.y);
   fprintf(fid,"%f\n", mag->mag_field_components.z);
   fclose(fid);
 }

 //ROS loop function
  void IMULogger::loop()
  {
    ros::Rate rate(1000);
    while (!ros::isShuttingDown())
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
