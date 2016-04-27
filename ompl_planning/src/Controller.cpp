#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <math.h>

using namespace std;

ros::Publisher robot;
float xI, yI, theta0;//initial states
float v, tcx, tcy;//linear and angular velocity and the terms of corrections
float x, d1X, y, d1Y;//Current positions and velocities
float U, d1V;//Control outputs angular and linear velocities

//PID controller variables
float err_old, err_int;
float Xd, Yd;
float w_lim;

void PID_control(const nav_msgs::Odometry &pose){
	geometry_msgs::Twist cmd;
	float thetaD;
	float x, y, theta;
	float err, err_der;
	float w;
	float kp, ki, kd;
	
	//Set control constants
	kp = 1;
	ki = 0;
	kd = 0;
	
	//Get position and orientation
	x = pose.pose.pose.position.x;
	y = pose.pose.pose.position.y;
	theta = tf::getYaw(pose.pose.pose.orientation);
	if(theta > M_PI){
		theta -= M_PI;
	}
	if(theta < -M_PI){
		theta += M_PI;
	}
	
	//Get desired theta
	thetaD = atan2(Yd-y, Xd-x);
	
	//Get error
	err = thetaD - theta;
	
	//Calculate derivative and integral error
	err_int += err;
	err_der = err - err_old;
	
	w = kp*err + ki*err_int + kd*err_der;
	
	if(w > w_lim){
		w = w_lim;
	}
	else{
		if(w < -w_lim){
			w = -w_lim;
		}
	}
			
	
	cout << "w : 		" << w << endl;
	cout << "x : 		" << x << endl;
	cout << "y :		" << y << endl;
	cout << "theta :	" << theta*180/M_PI << endl;
	cout << "err : 		" << err << endl;
	
	float dist = sqrt(pow(Xd-x,2)+pow(Yd-y,2));
	cout << "dist :		" << dist << endl;
	
	//Set command values;
	if(dist < 0.2){
		cmd.linear.x = 0;
		w = 0;
	}
	else
		cmd.linear.x = v;
	cmd.linear.y = 0;
	cmd.linear.z = 0;
	cmd.angular.x = 0;
	cmd.angular.y = 0;
	cmd.angular.z = w;
	
	robot.publish(cmd);	
}

void flatPIDControl(const nav_msgs::Odometry &pose){
	geometry_msgs::Twist cmd;
	
	float Xd, d1Xd, d2Xd;//desired X and its first and second derivative
	float Yd, d1Yd, d2Yd;//desired Y and its first and second derivative
	float a,b;//Objectives
	float qx,qy,qz,qw;//Quaternions
	float theta, thetaD;//current theta and desired theta
	float Vd, Ud;//desired linear and angular velocity
	float k0, k1;//control paramters, impirically decided
	float aux;
	double teste;
	
	
	//Calculate quaternion to euler
	qx = pose.pose.pose.orientation.x;
	qy = pose.pose.pose.orientation.y;
	qz = pose.pose.pose.orientation.z;
	qw = pose.pose.pose.orientation.w;
	
	theta = tf::getYaw(pose.pose.pose.orientation);
	
	/*theta = asin(2*qx*qy +2*qz*qw);
	
	if(qz >= 0 && qw >= 0 || qz <= 0 && qw <= 0){
		if(fabs(qz) >  fabs(qw))
			theta = M_PI-theta;
	}
	else{
		if(fabs(qz) >  fabs(qw))
			theta = -M_PI-theta;
	}*/
	
		
	//debug that might still be usefull
	/*cout << "theta = " << theta << endl;
	 * theta = theta*180/M_PI;
	 * cout << "qz = " << qz << endl;
	 * cout << "acos(qz)" << acos(qz)*180/M_PI << endl;
	 * cout << "asin(qz)" << asin(qz)*180/M_PI << endl;
	 * cout << "qw = " << qw << endl;
	 * cout << "acos(qw)" << acos(qw)*180/M_PI << endl;
	 * cout << "asin(qw)" << asin(qw)*180/M_PI << endl << endl;*/
	
	//Get Position
	x = pose.pose.pose.position.x;
	y = pose.pose.pose.position.y;
	
	//Set trajectory
	a = -8.95; Xd = a + 0; d1Xd = 0; d2Xd = 0;
	b = -16.4; Yd = b + 0; d1Yd = 0; d2Yd = 0;
	
	//Calculate desired theta, linear and angular velocities
	thetaD = atan2(d1Yd, d1Xd);
	Vd = sqrt(pow(d1Xd,2)+pow(d1Yd,2));
	Ud = (d2Yd*d1Xd - d2Xd*d1Yd)/(pow(d1Xd,2) + pow(d1Yd,2));
	
	//Set control parameters
	k1 = 1;
	k0 = 1/4;
	
	//Calculate current velocities
	d1X = v*cos(theta);
	d1Y = v*sin(theta);
	
	//Control
	tcx = d2Xd - k1*(d1X-d1Xd) - k0*(x-Xd);
	tcy = d2Yd - k1*(d1Y-d1Yd) - k0*(y-Yd);
	
	aux = pow(d1X,2)+pow(d1Y,2);
	
	if(aux == 0){
		d1V = d1V+0.1;
		U = U+0.05;
	}
	else{
		d1V = (d1X*tcx + d1Y*tcy)/(sqrt(aux));
		U = (d1X*tcy - d1Y*tcx)/(aux);
	}
	v = d1V + v;
	if(v >= 2)
		v = 2;
	if(v <= -2)
		v = -2;
		
	if(U >= 0.5)
		U = 0.5;
	if(U <= -0.5)
		U = -0.5;
	
		
	cmd.linear.x = v;
	cmd.linear.y = 0;
	cmd.linear.z = 0;
	cmd.angular.x = 0;
	cmd.angular.y = 0;
	cmd.angular.z = U	;
	
	robot.publish(cmd);
	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");

	ros::NodeHandle n;
	
	//Set initial values
	xI = -9.;
	yI = -22.;
	theta0 = 116*M_PI/180;
	v = 0;
	x = xI;
	y = yI;
	U = 0;
	d1V = 0;
	
	//Set PID values
	Xd = -9.46;
	Yd = -23.98;
	err_old = 0;
	err_int = 0;
	v = 0.5;
	w_lim = M_PI_2;
	
	ros::Subscriber odom = n.subscribe("odom", 1000, PID_control);
	robot = n.advertise<geometry_msgs::Twist>("/Pioneer3AT/cmd_vel", 1000);
	
	ros::Rate loop_rate(10);

	
	while (ros::ok())
	{

		
		loop_rate.sleep();
			

		ros::spinOnce();
	}


	return 0;
}
