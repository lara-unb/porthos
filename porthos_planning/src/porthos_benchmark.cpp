#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("arm");

    moveit::planning_interface::PlanningSceneInterface psc;  
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    system("rosrun dynamic_reconfigure dynparam set /move_group/trajectory_execution allowed_execution_duration_scaling 5.0");

    //	0.28454; -0.15722; 0.70554
    //pose:
    //0.7071; -0.000503; -0.00041502; 0.70712
    geometry_msgs::Pose target_pose1, inicial, final;
    target_pose1.position.x=0.28454;
    target_pose1.position.y= -0.15722;
    target_pose1.position.z=0.70554;

    target_pose1.orientation.x=0.707106781;
    target_pose1.orientation.y=0;	
    target_pose1.orientation.z=0;	
    target_pose1.orientation.w=0.707106781;	

    inicial.position.x=0.45406;
    inicial.position.y= -0.18;
    inicial.position.z=0.4447;

    inicial.orientation.x=-0.5;
    inicial.orientation.y=-0.5;	
    inicial.orientation.z=0.5;	
    inicial.orientation.w=-0.5;	

    final.position.x=0.45406;
    final.position.y= 0.18;
    final.position.z=0.4447;

    final.orientation.x=-0.5;
    final.orientation.y=-0.5;	
    final.orientation.z=0.5;	
    final.orientation.w=-0.5;	

    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseTarget(inicial);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    success=group.execute(my_plan);


    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    //Planning with path constraints...

    //first u set an orientation constraint
    moveit_msgs::OrientationConstraint ocm;  
    //then set params
    ocm.link_name = "link7";  
    ocm.header.frame_id = "odom_combined";
    ocm.orientation.x = -0.5 ;
    ocm.orientation.y = -0.5 ;
    ocm.orientation.z = 0.5 ;
    ocm.orientation.w = -0.5 ;

    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    //now u should instantiate a generalized constrain
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    //set constraints to ur group
    group.setPathConstraints(test_constraints);

    group.setPoseTarget(final);
    success=group.plan(my_plan);//plan and put the new plan inside the variable my_plan

    success=group.execute(my_plan);//Given a plan, execute it without waiting for completion. Return true on success.
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED"); 
    sleep(5.0);

    return 0;
}
