#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace std;

double width_robot = 0.175;
double vl = 0.0;
double vr = 0.0;
ros::Time last_time;	
double right_forward_enc = 0.0;
double right_backward_enc = 0.0;
double left_forward_enc = 0.0;
double left_backward_enc = 0.0;
double right_forward_enc_old = 0.0;
double right_backward_enc_old = 0.0;
double left_forward_enc_old = 0.0;
double left_backward_enc_old = 0.0;
double distance_left = 0.0;
double distance_right = 0.0;
//change this value accordingly
double ticks_per_meter = 280/0.1885;
double x = 0.0;
double y = 0.0;
double th = 0.0;
geometry_msgs::Quaternion odom_quat;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;	
	double vel_x = twist_aux.linear.x;
	double vel_th = twist_aux.angular.z;
	double right_vel_forward = 0.0;
	double right_vel_backward = 0.0;
	double left_vel_forward = 0.0;
	double left_vel_backward = 0.0;

	if(vel_x == 0){  // left turning when vel_th is positive
		right_vel_forward = vel_th * width_robot / 2.0;
		right_vel_backward= vel_th * width_robot / 2.0;
		left_vel_forward = (-1) * right_vel_forward;
		left_vel_backward = (-1) * right_vel_backward;
	}else if(vel_th == 0){ // forward / backward
		left_vel_forward = right_vel_forward = vel_x;
        left_vel_backward = right_vel_backward = vel_x;
	}else{
		 left_vel_forward=0;
	     left_vel_backward=0;
		 right_vel_forward=0;
		 right_vel_backward=0;	
	 }
		 /*else{ // moving during arcs
		left_vel = vel_x - vel_th * width_robot / 2.0;
		right_vel = vel_x + vel_th * width_robot / 2.0;*/
	
	vl_1 = left_vel_forward;
	vl_2 = left_vel_backward;
	vr_1 = right_vel_forward;
	vr_2 = right_vel_backward;	
}


int main(int argc, char** argv){
	ros::init(argc, argv, "base_controller");
	ros::NodeHandle n;
	ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, cmd_velCallback);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{

		double dxy = 0.0;
		double dth = 0.0;
		ros::Time current_time = ros::Time::now();
		double dt;
		double velxy = dxy / dt;
		double velth = dth / dt;

		ros::spinOnce();
		dt =  (current_time - last_time).toSec();;
		last_time = current_time;

		// calculate odomety
		if(right_enc == 0.0){
			distance_left = 0.0;
			distance_right = 0.0;
		}else{
			distance_left = (left_forward_enc - left_forward_enc_old) / ticks_per_meter;
			distance_right = (right_forward_enc - right_forward_enc_old) / ticks_per_meter;
		} 

		left_forward_enc_old = left_forward_enc;
		right_forward_enc_old = right_forward_enc;

		dxy = (distance_left + distance_right) / 2.0;
		dth = (distance_right - distance_left) / width_robot;

		if(dxy != 0){
			x += dxy * cosf(dth);
			y += dxy * sinf(dth);
		}	

		if(dth != 0){
			th += dth;
		}
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);
		loop_rate.sleep();
	}
}
