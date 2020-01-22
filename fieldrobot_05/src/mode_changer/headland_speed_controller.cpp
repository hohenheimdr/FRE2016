#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Bool.h>

#include <msgs/IntStamped.h>
#include "msgs/row.h"

#define USER_INPUT 0;	
#define OBSTACLE 1;
#define ROW_NAVIGATION 2;
#define HEADLAND_TURN 3;
#define STATIC 4;

//reads the signals from joystick topics and send them to the Motorcontroller command topic


class Sender_Node
{
private:
geometry_msgs::TwistStamped cmd_vel_row;
bool start_button, stop_button, action_button_1,action_button_2;
	//internal variable to save the actual mode
msgs::IntStamped mode;
msgs::row row;	



public:
	// public variables
	ros::Publisher pub_cmd_vel, mode_pub;
	std::string frame_id;
	double speed_max,angle_max, angle_ignore_val,row_distance_max;
	bool invert_angle;
	
	
	

	Sender_Node()
	{
		//set start mode to user input...
		
	}
	
	~Sender_Node()
	{
	}
	
	//-------continuous publisher called by the node to garante a constant output---------------
	void PublishSpeed(const ros::TimerEvent& e)
	{
		
		if (row.headland)
		{mode.data=HEADLAND_TURN;
			mode.header.stamp=ros::Time::now();
			mode_pub.publish(mode);
		 }
		/*
		else
		{
			if(abs_d(row.leftangle)<angle_ignore_val && abs_d(row.leftdistance)<row_distance_max)
			{
				if(abs_d(row.rightangle)<angle_ignore_val && abs_d(row.rightdistance)<row_distance_max)
				{
				mode.data= ROW_NAVIGATION;
				mode.header.stamp=ros::Time::now();	
				} 
			}
		 }
		*/
		
		
		
		ROS_INFO("row_speed %f, %f",cmd_vel_row.twist.linear.x,cmd_vel_row.twist.angular.z);
		pub_cmd_vel.publish(cmd_vel_row);
		
			
	}
	//subscribers
	void Row(const msgs::row::ConstPtr& msg)
	{
		row=*msg;
		double angle_left=0;
		double angle_right=0;
		//check if the row values are valid:
		if(abs_d(row.leftangle)<angle_ignore_val && abs_d(row.leftdistance)<row_distance_max)
		{
			//set angle 
			angle_left=row.leftangle;
			if(abs_d(row.rightangle)<angle_ignore_val && abs_d(row.rightdistance)<row_distance_max)
			{
			//set angle
			angle_right=row.rightangle;	
			}
			else
			{
				angle_right=0.0;
				angle_left=0.0;
			}
		}else
		{
				angle_right=0.0;
				angle_left=0.0;
		}
		
		//invert if angle turns in the wrong direction...
		//speed can be changed same way...
		if (invert_angle)
		{
			cmd_vel_row.twist.angular.z=-(angle_right+angle_left);
		}else
		{
			cmd_vel_row.twist.angular.z=(angle_right+angle_left);//*180.0/M_PI/2.0;
		}
		
		
		if (abs_d(angle_right+angle_left)>angle_max)
		{
			//slow down....
			cmd_vel_row.twist.linear.x=speed_max/3.0;
		}
		else
		{
			if(!row.headland)
			{
				cmd_vel_row.twist.linear.x=speed_max;
			}
			else
			{
				//set speed to zero....
				cmd_vel_row.twist.linear.x=0;
				cmd_vel_row.twist.angular.z=0;
			}
		}
		
	}

double abs_d(double t)
{
	return sqrt(t*t);
}
	

};
//-------------------------------------------------------------------------


int main (int argc, char** argv)
{
	std::string mode_str,row_sub_str, cmd_vel_str;
	
	ros::init(argc,argv,"row_speed_controller");
	ros::NodeHandle n("~");
	
	//read params of launch file
	
	Sender_Node s;

	n.param<std::string>("mode_pub", mode_str, "/mode"); 
	n.param<std::string>("row_sub", row_sub_str, "/row"); 
	n.param<std::string>("frame_id", s.frame_id, "/base_link"); 
	n.param<std::string>("cmd_vel", cmd_vel_str, "/cmd_vel_row");
	n.param<double>("speed_max", s.speed_max, 1.0); 
	n.param<double>("angle_max", s.angle_max, 1.0);
	n.param<double>("angle_ignore_val", s.angle_ignore_val, 1.0); 
	n.param<double>("row_distance_max", s.row_distance_max, 1.0); 
	n.param<bool>("invert_angle", s.invert_angle, false); 

	//subscribers for the error stop handling
	ros::Subscriber row=n.subscribe(row_sub_str,5,&Sender_Node::Row,&s);	
	
	//publishers
	s.pub_cmd_vel = n.advertise<geometry_msgs::TwistStamped>(cmd_vel_str.c_str(),10);
	s.mode_pub = n.advertise<msgs::IntStamped>(mode_str.c_str(),10);
	
	//define the update rate with a seperate function...
	ros::Timer t;
	t = n.createTimer(ros::Duration(0.05), &Sender_Node::PublishSpeed,&s);
	
	ros::spin();
		
}
