#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>

class ClawAction {
	
protected:
	ros::NodeHandle nh_;

	actionlib::SimpleActionServer<control_msgs::GripperCommandAction> action_server_;
	std::string action_name_;
	
	control_msgs::GripperCommandFeedback feedback_;
	control_msgs::GripperCommandResult result_;
public:
	ClawAction(std::string name):action_server_(nh_, name, boost::bind(&ClawAction::executeCB,this,_1), false),action_name_(name)
	{
		action_server_.start();			
	}

	~ClawAction(void){};
	
	void executeCB(const control_msgs::GripperCommandGoalConstPtr &goal){

		// TODO : EXECUTION SHOULD CALL THE JOINT ACTION FOR THE GRIPPER;
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "claw_action_server");

  ClawAction claw(ros::this_node::getName());
  ros::spin();

  return 0;
}
