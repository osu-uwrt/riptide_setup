#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>


class DepthControlActionServer
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<riptide_controllers::DepthControlActionServer> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  riptide_controllers::depth_commandfeedback feedback_;
  riptide_controllers::depth_commandresult result_;

public:

  DepthControlActionServer(std::string name) :
    as_(nh_, name, boost::bind(&DepthControlActionServer::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();

  }

  ~DepthControlActionServer(void)
  {
      ros::Publisher depth_pub = nh_.<riptide_msgs::DepthCommand>advertise("/command/depth",1000);
  }

  void executeCB(const riptide_controllers::depth_commandGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // start executing the action
    
    

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_controller_cmd");

  DepthControlActionServer depth("depth_controller_cmd");
  ros::spin();

  return 0;
}