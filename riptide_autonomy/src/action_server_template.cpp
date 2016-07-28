/******************************
 * ROS Action Server Template *
 *****************************/

/************
 * INCLUDES *
 ***********/
 // The following are required includes. Replace "TestAction" with your action name.
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <riptide_msgs/TestAction.h>

/**************************
 * LIGHTS, CAMERA, ACTION *
 *************************/

class TestAction
{
	protected:
		
		ros::NodeHandle nodeHandle;
		
		actionlib::SimpleActionServer<riptide_msgs::TestAction> actionServer;
		std::string actionName;
		
		// If reading data from another node, create a subscriber here
		ros::Subscriber subscriber;
		
		// Declare messages used to publish action result and feedback
		riptide_msgs::TestActionFeedback feedbackMsg;
		riptide_msgs::TestActionResult resultMsg;
		
		// Declare any other variables needed during action execution
		
	public:
	
		// Action constructor
		TestAction(std::string name) :
			actionServer(nodeHandle, name, boost::bind(&TestAction::executeCB, this, _1), false),
			actionName(name)
			{
				actionServer.start();
                isTaskCompleted = false;
				//subscriber = nodeHandle.subscribe("TestNode", 1000, subscriberCB);  
			}
		
		// Define callback functions (execute, analysis, goal, etc.)
		
		void executeCB(const riptide_msgs::TestGoalConstPtr &goal)
		{
			while (actionServer.isActive())
			{
				// If the state machine times out an action, it will request the server to be preempted. The server must then preempt itself.
				// First check this. Otherwise, you can run the action code.
				if (actionServer.isPreemptRequested())
				{
					// Add appropriate information to resultMsg
					actionServer.setPreempted();
				}
				else
				{
					// First check for subscriber callback by calling ros::spin(). Determine whether task completed based on callback
					ros::spinOnce();
					if (isTaskCompleted)
					{
						// Populate resultMsg and set action server completed.
						actionServer.setSucceeded();
					}
					
					// Then run the action code! If the task runs into issues, call actionServer.setAborted()
				}
			}
		}
/* Subscriber callback:		
		void subscriberCB(data)
		{
			isTaskCompleted = true;
		}
*/
	private:
		bool isTaskCompleted;
};

// Entry point for action server, simply creates and starts Action Server node.
int main(int argc, char** argv)
{
	// Init sets the node name in rostopic. Filename sets node name.
	ros::init(argc, argv, "TestAction");
	
	// Class name must match above code.
	TestAction TestAction(ros::this_node::getName());
	ros::spin();
	
	return 0;
}
