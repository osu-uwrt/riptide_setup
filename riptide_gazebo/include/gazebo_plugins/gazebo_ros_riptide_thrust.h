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

#ifndef RIPTIDE_THRUST_HH
#define RIPTIDE_THRUST_HH

#include <string>

#include "boost/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "ros/ros.h"
#include "riptide_msgs/ThrustStamped.h"
#include "ros/subscribe_options.h"
#include "ros/callback_queue.h"

namespace gazebo
{
class RiptideThrust : public ModelPlugin
{
 public:
  RiptideThrust();
  virtual ~RiptideThrust();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void UpdateChild();

 private:
  event::ConnectionPtr update_connection_;
  physics::WorldPtr world_;
  // Thrusters
  physics::LinkPtr surge_port_hi_;
  physics::LinkPtr surge_stbd_hi_;
  physics::LinkPtr surge_port_lo_;
  physics::LinkPtr surge_stbd_lo_;
  physics::LinkPtr sway_fwd_;
  physics::LinkPtr sway_aft_;
  physics::LinkPtr heave_port_fwd_;
  physics::LinkPtr heave_stbd_fwd_;
  physics::LinkPtr heave_port_aft_;
  physics::LinkPtr heave_stbd_aft_;
  // Msg
  std::string robot_namespace_;
  std::string topic_name_;
  riptide_msgs::ThrustStamped thrust_;
  // ROS
  ros::NodeHandle* rosnode_;
  ros::Subscriber sub_;
  ros::CallbackQueue queue_;
  // Tep...
  boost::thread callback_queue_thread_;
  boost::mutex lock_;
  // Functions?!
  void QueueThread();
  void UpdateObjectForce(const riptide_msgs::ThrustStamped::ConstPtr& _msg);
};
}
#endif
