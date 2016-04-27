#include <math.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <valarray>
#include <string>
#include <limits>
#include <ros/ros.h>

int main(int argc, char **argv){
	ros::init(argc,argv,"measureEEF");
	ROS_INFO("testing nodes...");
	ros::NodeHandle nh;
	tf::TransformListener listener;
	float y=0,y_old=0,x=0,x_old=0,z=0,z_old=0,length=0,acc_length=0;

		 tf::StampedTransform transform;
		 ros::Rate r(10);
		 sleep(2);
		 while(ros::ok()){

			    try{
			      listener.lookupTransform("base_link","link7",ros::Time(0), transform);

			    }
			    catch (tf::TransformException ex){
			      ROS_ERROR("%s",ex.what());
			      ROS_ERROR("POSE VALUES WERE NOT UPDATED!");
			      continue;
			    }

			    y=transform.getOrigin().y();
			    x=transform.getOrigin().x();
			    z=transform.getOrigin().z();
			    length=sqrt(pow((x-x_old),2)+pow((y-y_old),2)+pow((z-z_old),2));
			    acc_length+=length;
			    ROS_INFO("Distancia percorrida pelo EEF %f", acc_length);
			    y_old=y;
			    x_old=x;
			    z_old=z;
			    r.sleep();
			    ros::spinOnce();
		}
	return 0;
}