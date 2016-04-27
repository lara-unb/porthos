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

    geometry_msgs::Pose target_pose1;
    target_pose1.position.x=0.30618;
    target_pose1.position.y=-0.23027;
    target_pose1.position.z=0.69202;

    target_pose1.orientation.x=0.707106781;
    target_pose1.orientation.y=0;	
    target_pose1.orientation.z=0;	
    target_pose1.orientation.w=0.707106781;	

    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    success=group.execute(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_object.id = "can";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.12;
    primitive.dimensions[1] = 0.012;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose cyl_pose;
    cyl_pose.orientation.w = 1.0;
    cyl_pose.position.x =  0.48;
    cyl_pose.position.y = -0.20;
    cyl_pose.position.z =  0.44;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cyl_pose);
    collision_object.operation = collision_object.ADD;

    moveit_msgs::CollisionObject co2;
    co2.header.frame_id=group.getPlanningFrame();
    co2.id="table";

    shape_msgs::SolidPrimitive prim2;
    prim2.type=prim2.BOX;
    prim2.dimensions.resize(3);
    prim2.dimensions[0]=2;
    prim2.dimensions[1]=0.75;
    prim2.dimensions[2]=0.38;

    geometry_msgs::Pose box_pose;

    box_pose.orientation.w=0.707106781;
    box_pose.orientation.z=0.707106781;
    box_pose.position.x=0.70;
    box_pose.position.y=0;
    box_pose.position.z=0.19;

    co2.primitives.push_back(prim2);
    co2.primitive_poses.push_back(box_pose);
    co2.operation=co2.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;  
    collision_objects.push_back(collision_object);  
    collision_objects.push_back(co2);

    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");  
    psc.addCollisionObjects(collision_objects);
    sleep(4.0);

    group.setSupportSurfaceName("table");

    std::vector<moveit_msgs::Grasp> grasps;
    moveit_msgs::Grasp grasp1;

    geometry_msgs::PoseStamped p1;
    p1.header.frame_id=group.getPlanningFrame();
    p1.pose.position.x=0.41065;
    p1.pose.position.y=-0.189451;
    p1.pose.position.z=0.472776;
    p1.pose.orientation.x=0.707106781;
    p1.pose.orientation.y=0.707106781;
    p1.pose.orientation.z=0;
    p1.pose.orientation.w=0;
    /*FRUSTRATION POSE! LOL!
    p1.pose.position.x=0.410516;
    p1.pose.position.y=-0.191781;
    p1.pose.position.z=0.472776;
    p1.pose.orientation.x=0.707106781;
    p1.pose.orientation.y=0.707106781;
    p1.pose.orientation.z=0;
    p1.pose.orientation.w=0;
    */

    grasp1.grasp_pose=p1; // EEF parent's link pose ---> where the object actually is
    grasp1.pre_grasp_approach.direction.vector.y=1.0; // how you approach the object
    grasp1.pre_grasp_approach.direction.header.frame_id=group.getEndEffectorLink();
    grasp1.pre_grasp_approach.min_distance = 0.04;
    grasp1.pre_grasp_approach.desired_distance = 0.06;

    grasp1.post_grasp_retreat.direction.header.frame_id=group.getPlanningFrame(); // how you retreat the gripper (now with the picked object)
    grasp1.post_grasp_retreat.direction.vector.z=1.0;
    grasp1.post_grasp_retreat.min_distance = 0.025;
    grasp1.post_grasp_retreat.desired_distance = 0.5;
    // chronologically the sequence is PRE-GRASP, GRASP, POST-GRASP

    ros::Duration aux_dur(1,13414823);
    // open the gripper
    grasp1.pre_grasp_posture.joint_names.resize(1, "claw_left");
    grasp1.pre_grasp_posture.joint_names.resize(2, "claw_right");
    grasp1.pre_grasp_posture.points.resize(1);
    grasp1.pre_grasp_posture.points[0].positions.resize(2);
    grasp1.pre_grasp_posture.points[0].positions[0] = -0.017;
    grasp1.pre_grasp_posture.points[0].positions[1] = 0.017;
    grasp1.pre_grasp_posture.points[0].time_from_start=aux_dur;
    // close the gripper

    grasp1.grasp_posture.joint_names.resize(1, "claw_left");
    grasp1.grasp_posture.joint_names.resize(2, "claw_right");
    grasp1.grasp_posture.points.resize(1);
    grasp1.grasp_posture.points[0].positions.resize(2);
    grasp1.grasp_posture.points[0].positions[0] = 0;
    grasp1.grasp_posture.points[0].positions[1] = 0;
    grasp1.grasp_posture.points[0].time_from_start=aux_dur;

    grasps.push_back(grasp1);

    group.pick("can",grasps);

    // group.attachObject(collision_object.id);
    // sleep(4.0);

    return 0;
}
