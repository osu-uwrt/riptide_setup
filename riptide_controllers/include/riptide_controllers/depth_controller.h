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

#ifndef DEPTH_CONTROLLER_H
#define DEPTH_CONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "riptide_msgs/Depth.h"
#include "dynamic_reconfigure/server.h"
#include "riptide_controllers/DepthControllerConfig.h"

class DepthController
{
 private:
  // Comms
  ros::NodeHandle nh;
  ros::Subscriber depth_sub;
  ros::Subscriber cmd_sub;
  ros::Publisher cmd_pub;
  geometry_msgs::Accel accel;
  dynamic_reconfigure::Server<riptide_controllers::DepthControllerConfig> config_server;
  dynamic_reconfigure::Server<riptide_controllers::DepthControllerConfig>::CallbackType cb;

  //PID
  float depth_error;
  float current_depth;
  float cmd_depth;
  float error_sum;
  float d_error;
  float last_error;
  float dt;
  float kP;
  float kI;
  float kD;

  bool pid_initialized;
  bool clear_enabled;

  ros::Time sample_start;
  ros::Duration sample_duration;

  void UpdateError();

 public:
   DepthController();
   void CommandCB(const riptide_msgs::Depth::ConstPtr &depth);
   void DepthCB(const riptide_msgs::Depth::ConstPtr &cmd);
   void ConfigureCB(riptide_controllers::DepthControllerConfig &config, int level);
   void Loop();

};

#endif
