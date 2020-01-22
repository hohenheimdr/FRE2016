#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
	std::string frame_id,row_tf;
	double speed_max,angle_max, angle_ignore_val,row_distance_max, goal_dist, time_delay;
	bool inverse_angle,mode_changer_active;
	
	tf::TransformBroadcaster br;
	tf::TransformListener li;
	geometry_msgs::TransformStamped goal_trans;
	
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
		else
		{
			if(mode_changer_active)
			{
			//case when there is an obstacle....need to get out in the oposite direction..., need change to obstacle mode....
			if(abs_d(row.leftangle)>angle_ignore_val)
			{
				if(abs_d(row.rightangle)>angle_ignore_val)
				{
				mode.data= OBSTACLE;
				mode.header.stamp=ros::Time::now();	
				mode_pub.publish(mode);
				} 
			}
			}
		 }
		 
		// publish the transform message to follow new goalpoint

			 goal_trans.header.stamp = ros::Time::now();
			 geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
			 goal_trans.header.frame_id = "/base_link";
			 goal_trans.child_frame_id =  row_tf;
			 goal_trans.transform.translation.x = goal_dist;
			 goal_trans.transform.translation.y = row.leftdistance-row.rightdistance;//cmd_vel_row.twist.angular.z;
			 goal_trans.transform.translation.z = 0.0;
			 goal_trans.transform.rotation = odom_quat;
			 br.sendTransform(goal_trans); 
				
		ROS_INFO("row_speed %f, %f",cmd_vel_row.twist.linear.x,cmd_vel_row.twist.angular.z);
		
		pub_cmd_vel.publish(cmd_vel_row);
		
			
	}
	//subscribers
	void Row(const msgs::row::ConstPtr& msg)
	{
		row=*msg;
		//just change the value with the given time delay...
		sleep(time_delay);
		double angle_left=0.0;
		double angle_right=0.0;
		//check if the row values are valid and set to zero if not!
		if(abs_d(row.leftangle)<angle_ignore_val && abs_d(row.leftdistance)<row_distance_max)
		{
			//set angle 
			angle_left=row.leftangle;

		}else
		{
				angle_left=0.0;
		}
		
		if(abs_d(row.rightangle)<angle_ignore_val && abs_d(row.rightdistance)<row_distance_max)
			{
			//set angle
			angle_right=row.rightangle;	
			}
			else
			{
				angle_right=0.0;
			}
		
		
		if(inverse_angle)
		{
			cmd_vel_row.twist.angular.z=-(angle_right+angle_left);//*180.0/M_PI/2.0;
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
	n.param<bool>("inverse_angle", s.inverse_angle,true);
	n.param<bool>("mode_changer_active", s.mode_changer_active,true);
	n.param<double>("time_delay", s.time_delay,1.0);
	n.param<double>("goal_dist", s.goal_dist,1.0);
	n.param<std::string>("tf", s.row_tf,"row");

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
