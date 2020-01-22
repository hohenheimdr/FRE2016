#include "ros/ros.h"
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/TwistStamped.h"
#include "differential_ifk_lib/DifferentialIfk.hpp"
#include <iostream>

ros::Publisher pub_joystick_cmd_vel,pub_joystick_cmd_vel_frame;
ros::Publisher pub_joystick_speed_left;
ros::Publisher pub_joystick_speed_right;
ros::Publisher pub_joystick_button_A;
ros::Publisher pub_joystick_button_B;
ros::Publisher pub_joystick_button_X;
ros::Publisher pub_joystick_button_Y;
ros::Publisher pub_joystick_button_LB,	pub_joystick_button_RB ,pub_joystick_button_LT,	pub_joystick_button_RT;
//publisher msgs for motorcontroller speed
std_msgs::Int64 joystick_msg_left;
std_msgs::Int64 joystick_msg_right;	
geometry_msgs::TwistStamped cmd_vel,cmd_vel_frame;
std_msgs::Bool False;
std_msgs::Bool True;

DifferentialIfk differential;

void Joystick_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
False.data=false;
True.data=true;
	joystick_msg_right.data=0;
	joystick_msg_left.data=0;
	int fwd=0;
	int turn=0;
	cmd_vel.twist.linear.x=0;
	cmd_vel.twist.linear.y=0;
	cmd_vel.twist.angular.z=0;	

		if (msg->buttons[4]==1)
		{
		pub_joystick_button_LB.publish(True);
		pub_joystick_button_A.publish(False);
		pub_joystick_button_B.publish(False);
		pub_joystick_button_X.publish(False);
		pub_joystick_button_Y.publish(False);	
		pub_joystick_button_RB.publish(False);
		pub_joystick_button_LT.publish(False);
		pub_joystick_button_RT.publish(False);

		fwd=(1000*msg->axes[4]);
		turn=1000*msg->axes[3];
				
			//define ancle for turning...
		joystick_msg_left.data=-turn+fwd;
		joystick_msg_right.data=turn+fwd;
		
			if(joystick_msg_left.data>1000)
			{
				joystick_msg_left.data=1000;
			}
			
			if(joystick_msg_left.data<-1000)
			{
				joystick_msg_left.data=-1000;
			}
				if(joystick_msg_right.data>1000)
			{
				joystick_msg_right.data=1000;
			}
			if(joystick_msg_right.data<-1000)
			{
				joystick_msg_right.data=-1000;
			}
					
		
		//ROS_INFO( "%d %d", joystick_msg_right.data,joystick_msg_left.data);
		pub_joystick_speed_left.publish(joystick_msg_left);
		pub_joystick_speed_right.publish(joystick_msg_right);
		
		}else
		{
			pub_joystick_button_LB.publish(False);
		}
		//toggle button states when pressed
		if (msg->buttons[0]==1)
		{
			pub_joystick_button_A.publish(True);
			//pub_joystick_button_LB.publish(False);
		}
		if (msg->buttons[1]==1)
		{	
			pub_joystick_button_B.publish(True);
			//pub_joystick_button_LB.publish(False);
		}
		if (msg->buttons[2]==1)
		{
			
			pub_joystick_button_X.publish(True);
			//pub_joystick_button_LB.publish(False);
		}
		if (msg->buttons[3]==1)
		{	
			pub_joystick_button_Y.publish(True);
			//pub_joystick_button_LB.publish(False);
		}
		
		if (msg->buttons[5]==1)
		{	
			pub_joystick_button_RB.publish(True);
			//pub_joystick_button_LB.publish(False);
		}else{
			pub_joystick_button_RB.publish(False);
		}
		
		if (msg->axes[2]<1)
		{
			pub_joystick_button_LT.publish(True);
			//pub_joystick_button_LB.publish(False);
		}else{
			pub_joystick_button_LT.publish(False);
		}
		
		if (msg->axes[5]<1)
		{
			pub_joystick_button_RT.publish(True);
			//pub_joystick_button_LB.publish(False);
		}else{
			pub_joystick_button_RT.publish(False);

		}
		
		
		
		
//sett joystick cmd_vel
		cmd_vel_frame.twist.linear.x=msg->axes[1];
		cmd_vel_frame.twist.linear.y=msg->axes[0];
		cmd_vel_frame.twist.linear.z=msg->axes[7];
		
		cmd_vel.twist.linear.x=msg->axes[4];//(joystick_msg_left.data+joystick_msg_right.data)/2.0;
		cmd_vel.twist.angular.z=msg->axes[3];//(joystick_msg_right.data-joystick_msg_left.data)/0.4;
		
		cmd_vel_frame.header.stamp=ros::Time::now();
		cmd_vel.header.stamp=ros::Time::now();
		pub_joystick_cmd_vel.publish(cmd_vel);
		pub_joystick_cmd_vel_frame.publish(cmd_vel_frame);		

}

//starting joy

std::string joy_sub_str, cmd_vel_frame_str, cmd_vel_str;

int main (int argc, char** argv)
{
	ros::init(argc,argv,"joystick_core");
	
	ros::NodeHandle n("~");
	//read params of launch file
	if(!n.hasParam("joy_sub")) ROS_WARN("Used default parameter for joystick subscriber");
	n.param<std::string>("joy_sub", joy_sub_str, "/joy"); 
	if(!n.hasParam("cmd_vel")) ROS_WARN("Used default parameter for joystick cmd_vel");
	n.param<std::string>("cmd_vel", cmd_vel_str, "/cmd_vel_joy"); 
	if(!n.hasParam("cmd_vel_frame")) ROS_WARN("Used default parameter for joystick cmd_vel_frame");
	n.param<std::string>("cmd_vel_frame", cmd_vel_frame_str, "/cmd_vel_frame"); 
	cmd_vel.header.frame_id=cmd_vel_frame_str.c_str();
	//sett publishers for joystick speedcommands...
	pub_joystick_speed_left = n.advertise<std_msgs::Int64>("joystick_speed_left",100);
	pub_joystick_speed_right = n.advertise<std_msgs::Int64>("joystick_speed_right",100);
	
	//sett publishers for joystick buttons for toggle states...
	pub_joystick_button_A = n.advertise<std_msgs::Bool>("button_A",100);
	pub_joystick_button_B = n.advertise<std_msgs::Bool>("button_B",100);
	pub_joystick_button_X = n.advertise<std_msgs::Bool>("button_X",100);
	pub_joystick_button_Y = n.advertise<std_msgs::Bool>("button_Y",100);
	pub_joystick_button_LB = n.advertise<std_msgs::Bool>("button_LB",100);
	pub_joystick_button_RB = n.advertise<std_msgs::Bool>("button_RB",100);
	pub_joystick_button_LT = n.advertise<std_msgs::Bool>("button_LT",100);
	pub_joystick_button_RT = n.advertise<std_msgs::Bool>("button_RT",100);
	
	//set publisher for cmd_vel with baud rate of 150
	pub_joystick_cmd_vel = n.advertise<geometry_msgs::TwistStamped>(cmd_vel_str.c_str(),150);
	pub_joystick_cmd_vel_frame = n.advertise<geometry_msgs::TwistStamped>(cmd_vel_frame_str.c_str(),150);
	
	
	ros::Subscriber joy_sub= n.subscribe(joy_sub_str.c_str(),1,Joystick_callback);
		
	//take care that node is spinning...
	ros::spin();	
	
return 0;

}

