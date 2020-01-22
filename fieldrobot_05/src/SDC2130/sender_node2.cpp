#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Bool.h>


//reads the signals from joystick topics and send them to the Motorcontroller command topic


class Sender_Node
{
	
public:
	// public variables
	ros::Publisher pub_cmd_vel;
	geometry_msgs::TwistStamped cmd_vel_point,cmd_vel_joy,cmd_vel_error;
	geometry_msgs::TwistStamped cmd_vel_static;
	//this part comes later maybee not needed right now
	//geometry_msgs::TwistStamped cmd_vel_laser;
	bool obstacle_front, obstacle_back,obstacle_front_left,obstacle_front_right, obstacle_back_left,obstacle_back_right;
	bool start_button, stop_button, action_button_1,action_button_2;
	
	int error_points;

	Sender_Node()
	{
		
		obstacle_front=false;
		obstacle_back=false;
		obstacle_front_left=false;
		obstacle_front_right=false;
		obstacle_back_left=false;
		obstacle_back_right=false;
		
	}
	
	~Sender_Node()
	{
	}
	
	//-------publisher---------------
	void PublishSpeed()
	{
		if (start_button || action_button_1)
		{
			//when obstacle, get error cmd, else use point navigation cmd
			if(obstacle_front || obstacle_back)
			{
				//no turning, when error cloud at the back!
				if(obstacle_back)
					{
							cmd_vel_error.twist.angular.z=0.0;
					}else
					{
				
						if(obstacle_front_left && !obstacle_front_right)
						{
							//check if backwards is enough space
							cmd_vel_error.twist.linear.x=0.00;
							cmd_vel_error.twist.angular.z=-0.3;
						}
						if(!obstacle_front_left && obstacle_front_right)
						{
							cmd_vel_error.twist.linear.x=0.00;
							cmd_vel_error.twist.angular.z=0.3;
						}
						if(obstacle_front_left && obstacle_front_right)
						{
							cmd_vel_error.twist.linear.x=-0.10;
							cmd_vel_error.twist.angular.z=0.0;
						}
					}
				pub_cmd_vel.publish(cmd_vel_error);
				 //ROS_INFO("ErrorSpeed info: [%f %f]",cmd_vel_error.twist.linear.x,cmd_vel_error.twist.angular.z);
					//pub_cmd_vel.publish(cmd_vel_joy);		
					
			}else
			{
				// ROS_INFO("PointSpeed info: [%f %f]",cmd_vel_point.twist.linear.x,cmd_vel_point.twist.angular.z);
					if (start_button)
					{
					pub_cmd_vel.publish(cmd_vel_point);
					}
					if (action_button_1)
					{
						//publish static command:
						pub_cmd_vel.publish(cmd_vel_static);
					}
			}
		
		}else
		{
			//ROS_INFO("JoySpeed info: [%f %f]",cmd_vel_joy.twist.linear.x,cmd_vel_joy.twist.angular.z);
			//when not autonomous mode just use joystickvalue
			pub_cmd_vel.publish(cmd_vel_joy);
		}
		
		if(action_button_2)
		{
			//ROS_INFO("started following algorithm");
			
		}
		
		
	}
	//------subscribers----------
	void PointSpeed(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		 cmd_vel_point=*msg;
		 PublishSpeed();
	}
	
	void StaticSpeed(const geometry_msgs::Twist::ConstPtr& msg)
	{
		 cmd_vel_static.twist.linear=msg->linear;
		 cmd_vel_static.twist.angular=msg->angular;
		 PublishSpeed();
	}
	
	
	void JoystickSpeed(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		cmd_vel_joy=*msg;
			if (cmd_vel_joy.twist.linear.x!=0 || cmd_vel_joy.twist.angular.z!=0)
			{
				 pub_cmd_vel.publish(cmd_vel_joy);
			}
		PublishSpeed();

	}
	//read direct obstacles commands
	void Obstacle_front(const std_msgs::Bool::ConstPtr& msg)
	{
		obstacle_front=msg->data;	
	}
	
	void Obstacle_front_left(const std_msgs::Bool::ConstPtr& msg)
	{
		obstacle_front_left=msg->data;	
	}
	
	void Obstacle_front_right(const std_msgs::Bool::ConstPtr& msg)
	{
		obstacle_front_right=msg->data;	
	}
	
	void Obstacle_back(const std_msgs::Bool::ConstPtr& msg)
	{
		obstacle_back=msg->data;	
	}
	
	void Obstacle_back_left(const std_msgs::Bool::ConstPtr& msg)
	{
		obstacle_back_left=msg->data;	
	}
	
	void Obstacle_back_right(const std_msgs::Bool::ConstPtr& msg)
	{
		obstacle_back_right=msg->data;	
	}
	//button subscribers
	void Start_Button(const std_msgs::Bool::ConstPtr& msg)
	{
		start_button=msg->data;
	}
	
	void Stop_Button(const std_msgs::Bool::ConstPtr& msg)
	{
		stop_button=msg->data;
	}
	void Action_Button_1(const std_msgs::Bool::ConstPtr& msg)
	{
		action_button_1=msg->data;
	}
	
	void Action_Button_2(const std_msgs::Bool::ConstPtr& msg)
	{
		action_button_2=msg->data;
	}

};
//-------------------------------------------------------------------------


int main (int argc, char** argv)
{
	std::string cmd_vel_pub_str,cmd_vel_joy_str,cmd_vel_point_str,cmd_vel_static_str;
	std::string start_button_str,stop_button_str,action_button_1_str,action_button_2_str;
	std::string obstacle_front_str,obstacle_front_left_str,obstacle_front_right_str;
	std::string obstacle_back_str,obstacle_back_left_str,obstacle_back_right_str;

	ros::init(argc,argv,"sender_controller");
	ros::NodeHandle n("~");
	
	//read params of launch file
	if(!n.hasParam("cmd_vel_joy")) ROS_WARN("Used default parameter for  cmd_vel_joy");
	n.param<std::string>("joy_sub_left", cmd_vel_joy_str, "/cmd_vel_joy"); 
	if(!n.hasParam("cmd_vel_point")) ROS_WARN("Used default parameter for  cmd_vel_point");
	n.param<std::string>("cmd_vel_point", cmd_vel_point_str, "/cmd_vel_point"); 
	n.param<std::string>("cmd_vel_static", cmd_vel_static_str, "/cmd_vel_static"); 
	if(!n.hasParam("cmd_vel_pub")) ROS_WARN("Used default parameter for cmd_vel_pub");
	n.param<std::string>("cmd_vel_pub", cmd_vel_pub_str, "/cmd_vel"); 
	//not neccessary but useful :)
	n.param<std::string>("start_button", start_button_str, "/start_button"); 
	n.param<std::string>("stop_button", stop_button_str, "/stop_button"); 
	n.param<std::string>("action_button_1", action_button_1_str, "/action_button_1"); 
	n.param<std::string>("action_button_2", action_button_2_str, "/action_button_2"); 
	//subscribers for the error stop handling
	n.param<std::string>("obstacle_front", obstacle_front_str, "/obstacle_front"); 
	n.param<std::string>("obstacle_front_left", obstacle_front_left_str, "/obstacle_front_left"); 
	n.param<std::string>("obstacle_front_right", obstacle_front_right_str, "/obstacle_front_right"); 
	n.param<std::string>("obstacle_back", obstacle_back_str, "/obstacle_back"); 
	n.param<std::string>("obstacle_back_left", obstacle_back_left_str, "/obstacle_back_left"); 
	n.param<std::string>("obstacle_back_right", obstacle_back_right_str, "/obstacle_back_right"); 
	
	Sender_Node s;

	//speed cmd subscribers
	ros::Subscriber joy=n.subscribe(cmd_vel_joy_str,5,&Sender_Node::JoystickSpeed,&s);
	ros::Subscriber point=n.subscribe(cmd_vel_point_str,5,&Sender_Node::PointSpeed,&s);
	ros::Subscriber _static=n.subscribe(cmd_vel_static_str,5,&Sender_Node::StaticSpeed,&s);
	//joystick button subscribers
	ros::Subscriber a=n.subscribe(start_button_str,10,&Sender_Node::Start_Button,&s);	
	ros::Subscriber b=n.subscribe(stop_button_str,10,&Sender_Node::Stop_Button,&s);
	ros::Subscriber x=n.subscribe(action_button_1_str,10,&Sender_Node::Action_Button_1,&s);
	ros::Subscriber y=n.subscribe(action_button_2_str,10,&Sender_Node::Action_Button_2,&s);
	//error laser subscribers
	ros::Subscriber o1=n.subscribe(obstacle_front_str,10,&Sender_Node::Obstacle_front,&s);
	ros::Subscriber o2=n.subscribe(obstacle_front_left_str,10,&Sender_Node::Obstacle_front_left,&s);
	ros::Subscriber o3=n.subscribe(obstacle_front_right_str,10,&Sender_Node::Obstacle_front_right,&s);
	ros::Subscriber o4=n.subscribe(obstacle_back_str,10,&Sender_Node::Obstacle_back,&s);
	ros::Subscriber o5=n.subscribe(obstacle_back_right_str,10,&Sender_Node::Obstacle_back_right,&s);
	ros::Subscriber o6=n.subscribe(obstacle_back_left_str,10,&Sender_Node::Obstacle_back_left,&s);
	//publisher
	s.pub_cmd_vel = n.advertise<geometry_msgs::TwistStamped>(cmd_vel_pub_str.c_str(),10);
	
	ros::spin();
		
}
