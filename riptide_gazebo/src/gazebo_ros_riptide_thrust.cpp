#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_riptide_thrust.h>

namespace gazebo
{
  GZ_REGISTER_MODEL_PLUGIN(RiptideThrust);

  RiptideThrust::RiptideThrust()
  {
    this->thrust_.force.surge_port_hi = 0;
    this->thrust_.force.surge_stbd_hi = 0;
    this->thrust_.force.surge_port_lo = 0;
    this->thrust_.force.surge_stbd_lo = 0;
    this->thrust_.force.sway_fwd = 0;
    this->thrust_.force.sway_aft = 0;
    this->thrust_.force.heave_port_fwd = 0;
    this->thrust_.force.heave_stbd_fwd = 0;
    this->thrust_.force.heave_port_aft = 0;
    this->thrust_.force.heave_stbd_aft = 0;
  }

  RiptideThrust::~RiptideThrust()
  {
    event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

    this->queue_.clear();
    this->queue_.disable();
    this->rosnode_->shutdown();
    this->callback_queue_thread_.join();

    delete this->rosnode_;
  }

  void RiptideThrust::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->world_ = _model->GetWorld();

    this->robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    this->surge_port_hi_ = _model->GetLink("surge_port_hi_link");
    this->surge_stbd_hi_ = _model->GetLink("surge_stbd_hi_link");
    this->surge_port_lo_ = _model->GetLink("surge_port_lo_link");
    this->surge_stbd_lo_ = _model->GetLink("surge_stbd_lo_link");
    this->sway_fwd_ = _model->GetLink("sway_fwd_link");
    this->sway_aft_ = _model->GetLink("sway_aft_link");
    this->heave_port_fwd_ = _model->GetLink("heave_port_fwd_link");
    this->heave_stbd_fwd_ = _model->GetLink("heave_stbd_fwd_link");
    this->heave_port_aft_ = _model->GetLink("heave_port_aft_link");
    this->heave_stbd_aft_ = _model->GetLink("heave_stbd_aft_link");

    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ros::SubscribeOptions so = ros::SubscribeOptions::create<riptide_msgs::ThrustStamped>(
      this->topic_name_,1, boost::bind( &RiptideThrust::UpdateObjectForce,this,_1),
      ros::VoidPtr(), &this->queue_);
    this->sub_ = this->rosnode_->subscribe(so);

    this->callback_queue_thread_ = boost::thread( boost::bind( &RiptideThrust::QueueThread,this ) );

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&RiptideThrust::UpdateChild, this));
  }

  void RiptideThrust::UpdateObjectForce(const riptide_msgs::ThrustStamped::ConstPtr& _msg)
  {
    this->thrust_.force.surge_port_hi = _msg->force.surge_port_hi;
    this->thrust_.force.surge_stbd_hi = _msg->force.surge_stbd_hi;
    this->thrust_.force.surge_port_lo = _msg->force.surge_port_lo;
    this->thrust_.force.surge_stbd_lo = _msg->force.surge_port_lo;
    this->thrust_.force.sway_fwd = _msg->force.sway_fwd;
    this->thrust_.force.sway_aft = _msg->force.sway_aft;
    this->thrust_.force.heave_port_fwd = _msg->force.heave_port_fwd;
    this->thrust_.force.heave_stbd_fwd = _msg->force.heave_stbd_fwd;
    this->thrust_.force.heave_port_aft = _msg->force.heave_port_aft;
    this->thrust_.force.heave_stbd_aft = _msg->force.heave_stbd_aft;
  }

  void RiptideThrust::UpdateChild()
  {
    this->lock_.lock();

    math::Vector3 zero(0, 0, 0);

    // +z is "forward" thrust
    math::Vector3 sph(0, 0, this->thrust_.force.surge_port_hi);
    math::Vector3 ssh(0, 0, this->thrust_.force.surge_stbd_hi);
    math::Vector3 spl(0, 0, this->thrust_.force.surge_port_lo);
    math::Vector3 ssl(0, 0, this->thrust_.force.surge_stbd_lo);
    math::Vector3 sf(0, 0, this->thrust_.force.sway_fwd);
    math::Vector3 sa(0, 0, this->thrust_.force.sway_aft);
    math::Vector3 hpf(0, 0, this->thrust_.force.heave_port_fwd);
    math::Vector3 hsf(0, 0, this->thrust_.force.heave_stbd_fwd);
    math::Vector3 hpa(0, 0, this->thrust_.force.heave_port_aft);
    math::Vector3 hsa(0, 0, this->thrust_.force.heave_stbd_aft);
    this->surge_port_hi_->SetForce(zero);
    this->surge_stbd_hi_->SetForce(zero);
    this->surge_port_lo_->SetForce(zero);
    this->surge_stbd_lo_->SetForce(zero);
    this->sway_fwd_->SetForce(zero);
    this->sway_aft_->SetForce(zero);
    this->heave_port_fwd_->SetForce(zero);
    this->heave_stbd_fwd_->SetForce(zero);
    this->heave_port_aft_->SetForce(zero);
    this->heave_stbd_aft_->SetForce(zero);
    // TODO: Simplify this giant block
    this->surge_port_hi_->AddLinkForce(sph);
    this->surge_stbd_hi_->AddLinkForce(ssh);
    this->surge_port_lo_->AddLinkForce(spl);
    this->surge_stbd_lo_->AddLinkForce(ssl);
    this->sway_fwd_->AddLinkForce(sf);
    this->sway_aft_->AddLinkForce(sa);
    this->heave_port_fwd_->AddLinkForce(hpf);
    this->heave_stbd_fwd_->AddLinkForce(hsf);
    this->heave_port_aft_->AddLinkForce(hpa);
    this->heave_stbd_aft_->AddLinkForce(hsa);

    this->lock_.unlock();
  }

  void RiptideThrust::QueueThread()
  {
    static const double timeout = 0.01;

    while (this->rosnode_->ok())
    {
      this->queue_.callAvailable(ros::WallDuration(timeout));
    }
  }
}
