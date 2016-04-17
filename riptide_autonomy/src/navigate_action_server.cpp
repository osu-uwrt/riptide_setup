/******************************
 * ROS Action Server Template *
 *****************************/

/************
 * INCLUDES *
 ***********/
 
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vector>
#include <riptide_msgs/NavigateAction.h>

/**************************
 * LIGHTS, CAMERA, ACTION *
 *************************/

class NavigateAction
{
	protected:
		
		ros::NodeHandle nodeHandle;
		
		actionlib::SimpleActionServer<riptide_msgs::NavigateAction> actionServer;
		std::string actionName;
		
		// If reading data from another node, create a subscriber here
		ros::Subscriber subscriber;
		
		// Declare messages used to publish action result and feedback
		riptide_msgs::NavigateFeedback feedbackMsg;
		riptide_msgs::NavigateResult resultMsg;
		
		// Declare any other variables needed during action execution
		
	public:
	
		// Action constructor
		NavigateAction(std::string name) :
			actionServer(nodeHandle, name, boost::bind(&NavigateAction::executeCB, this, _1), false),
			actionName(name)
			{
				actionServer.start();
			}
		
		~NavigateAction(void) {}
		
		// Define callback functions (execute, analysis, goal, etc.)
		
		void executeCB(const riptide_msgs::NavigateGoalConstPtr &goal)
		{
			ros::Time startTime = ros::Time::now();
			ros::Time currentTime = ros::Time::now();
			
			while (currentTime - startTime < goal->timeout && actionServer.isActive())
			{
				if (currentTime - startTime > ros::Duration(5))
				{
					resultMsg.foundGoal = true;
					resultMsg.realPosition.x = 4;
					resultMsg.realPosition.y = 2;
					resultMsg.realPosition.z = 7;
					ROS_INFO("Goal: <%f, %f, %f>", goal->positionGoal.x, goal->positionGoal.y, goal->positionGoal.z);
					actionServer.setSucceeded(resultMsg);
				}
				
				currentTime = ros::Time::now();
			}
			
			if (actionServer.isActive())
			{
				resultMsg.foundGoal = false;
				resultMsg.realPosition.x = 2;
				resultMsg.realPosition.y = 3;
				resultMsg.realPosition.z = 5;
				actionServer.setAborted(resultMsg);
			}
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "NavigateAction");
	
	NavigateAction NavigateAction(ros::this_node::getName());
	ros::spin();
	
	return 0;
}
	
	
	
