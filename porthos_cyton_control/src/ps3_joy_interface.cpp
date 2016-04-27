#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist joystick;

void cb(const sensor_msgs::JoyConstPtr& msg){
	joystick.linear.x=msg->axes[1];
	joystick.angular.z=msg->axes[2];
}

int main(int argc, char **argv){

	ros::init(argc,argv,"ps3_joy_interface");
	ros::NodeHandle n;

	ros::Subscriber sub=n.subscribe("/joy",2,cb);

	ros::Publisher pub1=n.advertise<geometry_msgs::Twist>("/cmd_vel",2);

	ros::Rate r(1000.0);

	while(ros::ok()){
		pub1.publish(joystick);
		ros::spinOnce();
		r.sleep();

	}

	
	return 0;
}
