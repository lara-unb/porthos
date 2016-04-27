#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

std::vector<double> joint_state(4);



int main(int argc, char* argv[]){

    ros::init(argc,argv,"wheel_publisher");
    ros::NodeHandle n;
    ros::Publisher pub1 = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    if( !pub1 ) {
      ROS_WARN("Publisher1 invalid!");
    }

  ros::Rate loop_rate(20);
    while(ros::ok()){
    sensor_msgs::JointState js;

                                                
      js.name.push_back(std::string("p3at_front_left_wheel_joint"));
      js.velocity.push_back(0);
      js.position.push_back(0);
      js.name.push_back(std::string("p3at_front_right_wheel_joint"));
      js.velocity.push_back(0);
      js.position.push_back(0);
      js.name.push_back(std::string("p3at_back_left_wheel_joint"));
      js.velocity.push_back(0);
      js.position.push_back(0);
      js.name.push_back(std::string("p3at_back_right_wheel_joint"));
      js.velocity.push_back(0);
      js.position.push_back(0);

js.header.stamp = ros::Time::now();

    pub1.publish(js);
        ros::spinOnce();
    loop_rate.sleep();
    }

return 0;
}