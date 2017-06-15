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

#include "gazebo_plugins/gazebo_ros_depth_sensor.h"

#include <assert.h>
#include <algorithm>
#include <string>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(DepthSensor);

DepthSensor::DepthSensor()
{
  this->depth_.depth = 0;
  this->depth_.pressure = 0;
  this->depth_.temp = 0;
  this->depth_.altitude = 0;
}

DepthSensor::~DepthSensor()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  this->rosnode_->shutdown();

  delete this->rosnode_;
}

void DepthSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->world_ = _model->GetWorld();
  this->gravity_ = this->world_->Gravity().Z();

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  this->sensor_link_name_ = _sdf->GetElement("sensorLink")->Get<std::string>();
  this->sensor_link_ = _model->GetLink(sensor_link_name_);

  this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  this->depth_pub_ = this->rosnode_->advertise<riptide_msgs::Depth>(this->topic_name_, 1000);

  this->sea_level_ = _sdf->GetElement("seaLevel")->Get<double>();
  this->fluid_density_ = _sdf->GetElement("fluidDensity")->Get<double>();

  this->update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&DepthSensor::Update, this));
}

  void DepthSensor::Update()
  {
    double z_pos = this->sensor_link_->GetWorldPose().pos.z;
    this->depth_.depth = this->sea_level_ - z_pos;
    this->depth_.pressure = this->fluid_density_ * this->gravity_ * this->depth_.depth;
    this->depth_pub_.publish(this->depth_);
  }
}
