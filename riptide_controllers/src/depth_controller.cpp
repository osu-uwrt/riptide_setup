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

#include "riptide_controllers/depth_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_controller");
  DepthController dc;
  dc.Loop();
}

void DepthController::ConfigureCB(riptide_controllers::DepthControllerConfig &config, int level) {
  kP = config.P;
  kI = config.I;
  kD = config.D;
  clear_enabled = config.Reset;
}

void DepthController::UpdateError() {
  if (clear_enabled) {
    last_error = 0;
    error_sum = 0;
  }

  sample_duration = (ros::Time::now() - sample_start);
  dt = sample_duration.toSec();

  depth_error = cmd_depth - current_depth ;
  error_sum += depth_error * dt;
  d_error = (depth_error - last_error) / dt;
  last_error = depth_error;

  ROS_INFO("PID: %f %f %f", kP, kI, kD);
  ROS_INFO("Measured Depth: %f", current_depth);
  ROS_INFO("Target Depth: %f\n", cmd_depth);

  accel.linear.x = 0;
  accel.linear.y = 0;
  accel.linear.z = kP * depth_error + kI * error_sum + kD * d_error;
  accel.angular.x = 0;
  accel.angular.y = 0;
  accel.angular.z = 0;

  cmd_pub.publish(accel);
  sample_start = ros::Time::now();
}

DepthController::DepthController() {
    cmd_sub = nh.subscribe<riptide_msgs::Depth>("command/depth", 1000, &DepthController::CommandCB, this);
    depth_sub = nh.subscribe<riptide_msgs::Depth>("state/depth", 1000, &DepthController::DepthCB, this);

    cmd_pub = nh.advertise<geometry_msgs::Accel>("command/accel", 1);

    sample_start = ros::Time::now();
}

// Subscribe to command/depth
void DepthController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth) {
  current_depth = depth->depth;
  if (!pid_initialized) {
    cmd_depth = current_depth;
  }

  DepthController::UpdateError();
}

// Subscribe to state/depth
void DepthController::CommandCB(const riptide_msgs::Depth::ConstPtr &cmd) {
  cmd_depth = cmd->depth;

  if (!pid_initialized)
    pid_initialized = true;

  DepthController::UpdateError();
}

void DepthController::Loop() {
  cb = boost::bind(&DepthController::ConfigureCB, this, _1, _2);
  config_server.setCallback(cb);
  ros::spin();
}
