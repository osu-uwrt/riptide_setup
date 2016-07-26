#ifndef RIPTIDE_THRUST_HH
#define RIPTIDE_THRUST_HH

#include <string>

#include <ros/ros.h>
#include <riptide_msgs/ThrustStamped.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

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
