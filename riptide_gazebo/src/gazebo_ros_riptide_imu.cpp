#include "gazebo_plugins/gazebo_ros_riptide_imu.h"
#include <cmath>
#include <assert.h>
#include <algorithm>
#include <string>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(IMU);
IMU::IMU()
{
  this->IMU_.euler_rpy.x = 0;
  this->IMU_.euler_rpy.y = 0;
  this->IMU_.euler_rpy.z = 0;
  this->IMU_.ang_vel.x = 0;
  this->IMU_.ang_vel.y = 0;
  this->IMU_.ang_vel.z = 0;
  this->IMU_.linear_accel.x = 0;
  this->IMU_.linear_accel.y = 0;
  this->IMU_.linear_accel.z = 0;
  this->IMU_.ang_accel.x = 0;
  this->IMU_.ang_accel.y = 0;
  this->IMU_.ang_accel.z = 0;
}

IMU::~IMU()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

void IMU::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

  this->world_ = _model->GetWorld();
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
    _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  this->sensor_link_name_ = _sdf->GetElement("sensorLink")->Get<std::string>();
  sensor_link_ = _model->GetLink(sensor_link_name_);

  this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->IMU_pub_ = this->rosnode_->advertise<riptide_msgs::Imu>(this->topic_name_, 1);

  // this->angularVelocity = _model->GetWorldAngularVel();
  // this->angularAccel = _model->GetWorldAngularAccel();
  // this->linearAccel = _model->GetWorldLinearAccel();
  // this->modelPose = _model->GetWorldPose();

  this->update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&IMU::Update, this));

}

  void IMU::Update()
  {
    // TODO: FIX!

    angularVelocity = sensor_link_->GetWorldAngularVel();
    angularAccel = sensor_link_->GetWorldAngularAccel();
    linearAccel = sensor_link_->GetWorldLinearAccel();
    modelPose = sensor_link_->GetWorldPose();

    this->IMU_.euler_rpy.x = this->modelPose.rot.GetRoll()*180/M_PI;
    this->IMU_.euler_rpy.y = this->modelPose.rot.GetPitch()*180/M_PI;
    this->IMU_.euler_rpy.z = this->modelPose.rot.GetYaw()*180/M_PI;

    this->IMU_.ang_vel.x = angularVelocity.x;
    this->IMU_.ang_vel.y = angularVelocity.y;
    this->IMU_.ang_vel.z = angularVelocity.z;

    this->IMU_.ang_accel.x = angularAccel.x;
    this->IMU_.ang_accel.y = angularAccel.y;
    this->IMU_.ang_accel.z = angularAccel.z;

    this->IMU_.linear_accel.x = linearAccel.x;
    this->IMU_.linear_accel.y = linearAccel.y;
    this->IMU_.linear_accel.z = linearAccel.z;

    this->IMU_.header.stamp = ros::Time::now();

    this->IMU_pub_.publish(this->IMU_);

  }

}
