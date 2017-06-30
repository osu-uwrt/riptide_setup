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

#ifndef THRUSTER_CONTROLLER_H
#define THRUSTER_CONTROLLER_H

#include <math.h>
#include <vector>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Accel.h"
#include "sensor_msgs/Imu.h"
#include "imu_3dm_gx4/FilterOutput.h"

#include "riptide_msgs/ThrustStamped.h"

class ThrusterController
{
 private:
  // Comms
  ros::NodeHandle nh;
  ros::Subscriber state_sub;
  ros::Subscriber cmd_sub;
  ros::Publisher cmd_pub;
  riptide_msgs::ThrustStamped thrust;
  // Math
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  // Results
  double surge_stbd_hi, surge_port_hi, surge_port_lo, surge_stbd_lo;
  double sway_fwd, sway_aft;
  double heave_port_aft, heave_stbd_aft, heave_stbd_fwd, heave_port_fwd;
  // TF
  tf::TransformListener *listener;
  tf::StampedTransform tf_surge[4];
  tf::StampedTransform tf_sway[2];
  tf::StampedTransform tf_heave[4];

 public:
  ThrusterController(char **argv, tf::TransformListener *listener_adr);
  void state(const sensor_msgs::Imu::ConstPtr &msg);
  void callback(const geometry_msgs::Accel::ConstPtr &a);
  void loop();
};

#endif
