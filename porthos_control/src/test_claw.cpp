#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

int main (int argc, char **argv){

    ros::init(argc, argv, "test_claw");	

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client("/claw_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    action_client.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    control_msgs::FollowJointTrajectoryGoal goal;

    // set trajectory for the claw
    goal.trajectory.header.seq=0;
    // set trajectory header
    goal.trajectory.header.frame_id="/odom_combined";

    // set joints
    goal.trajectory.joint_names.push_back("claw_left");
    goal.trajectory.joint_names.push_back("claw_right");

    // set points
    trajectory_msgs::JointTrajectoryPoint aux;
    aux.positions.push_back(-0.017);
    aux.positions.push_back(0.017);
    aux.velocities.push_back(0.0);
    aux.velocities.push_back(0.0);

    ros::Duration aux_dur(1,13414823);
    aux.time_from_start=aux_dur;

    goal.trajectory.points.push_back(aux);

    action_client.sendGoal(goal);
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = action_client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");
    return 0;
}
