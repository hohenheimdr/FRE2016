#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <msgs/IntStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
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
	geometry_msgs::Twist cmd_vel_static, cmd_motor,cmd_light,cmd_magnet;
	//internal variable to save the actual mode
	msgs::IntStamped mode;
	
public:
	// public variables
	ros::Publisher pub_cmd_vel, mode_pub,pub_cmd_magnet, pub_cmd_light,pub_cmd_motor,pub_goal;
	std::string frame_id,row_tf;
	double speed_max,angle_max, angle_ignore_val,row_distance_max, goal_dist, time_delay;
	bool inverse_angle,mode_changer_active,metal_detected,goal_published;
	geometry_msgs::PoseStamped trash_goal;
	
	
	Sender_Node()
	{
		//set start mode to user input...
		goal_published=false;
		
		
	}
	
	~Sender_Node()
	{
	}  
	
	void init()
	{
		turn_lights_on();
		turn_magnets_off();
		move_motor_up(6.0);
		//cmd_motor.linear.x=1;
		move_motor_down(2.0);
		stop_motor();
		turn_magnets_on();
		turn_lights_off();
		
	}
	void Mode_Changer(const ros::TimerEvent& b)
	{
		
		stop_motor();
	}
	
	//-------continuous publisher called by the node to garante a constant output---------------
	void PublishSpeed(const ros::TimerEvent& e)
	{
		//now turn back to the old mode...
		//first set mode to point navigation algorithm....
		mode.data=HEADLAND_TURN;
		mode_pub.publish(mode);

		if(metal_detected)
		{
		//start routine for the searching of the areal.
		//first set mode to static
		mode.data=STATIC;
		//then start the routine for the movement....and publish the message all time 	
		cmd_vel_static.linear.x=1.0;
		sleep(1);
		cmd_vel_static.linear.x=-1.0;
		pub_cmd_vel.publish(cmd_vel_static);
		sleep(1);
		cmd_vel_static.linear.x=1.0;
		pub_cmd_vel.publish(cmd_vel_static);
		sleep(1);
		cmd_vel_static.linear.x=-1.0;
		cmd_vel_static.angular.z=0.1;
		pub_cmd_vel.publish(cmd_vel_static);
		sleep(1);
		cmd_vel_static.linear.x=1.0;
		cmd_vel_static.angular.z=-0.1;
		pub_cmd_vel.publish(cmd_vel_static);
		sleep(1);
		cmd_vel_static.linear.x=-1.0;
		cmd_vel_static.angular.z=0.1;
		pub_cmd_vel.publish(cmd_vel_static);
		sleep(1);
		cmd_vel_static.linear.x=1.0;
		cmd_vel_static.angular.z=-0.1;
		cmd_vel_static.linear.x=0.0;
		cmd_vel_static.angular.z=0.0;	
		//assuming that the metal was collected, set the goal to zero
		//in the odom grid to move ther and set the
		//trash_goal.pose.position.x=0;
		//trash_goal.pose.position.y=0;
		//trash_goal.header.frame_id="odom";
		//trash_goal.header.stamp=ros::Time::now();
		//pub_goal.publish(trash_goal);
		//metal_detected=false;
		//goal_published=true;
		ROS_INFO("set new goal");
		}
		pub_cmd_vel.publish(cmd_vel_static);		 
		
		ROS_INFO("pub speed");	
	}
	
	void move_motor_down(double t)
	{
		cmd_motor.linear.x=1;
		ros::Time start_time=ros::Time::now();
		
		ros::Duration time_out(t);
		while ((ros::Time::now()- start_time)<time_out)
		{
			pub_cmd_motor.publish(cmd_motor);
		}
	}
	void stop_motor()
	{
		cmd_motor.linear.x=0;
		pub_cmd_motor.publish(cmd_motor);
	}
	
	void move_motor_up(double t)
	{
		cmd_motor.linear.x=-1;
		ros::Time start_time=ros::Time::now();
		
		ros::Duration time_out(t);
		while ((ros::Time::now()- start_time)<time_out)
		{
			pub_cmd_motor.publish(cmd_motor);
		}
	}
	
	void turn_magnets_on()
	{
		cmd_magnet.linear.x=1;
		pub_cmd_magnet.publish(cmd_magnet);	
	}
	
	void turn_magnets_off()
	{
		cmd_magnet.linear.x=0;
		pub_cmd_magnet.publish(cmd_magnet);	
	}
	
	void turn_lights_on()
	{
		cmd_light.linear.x=1;
		pub_cmd_light.publish(cmd_light);	
	}
	
	void turn_lights_off()
	{
			cmd_light.linear.x=0;
			pub_cmd_light.publish(cmd_light);	
	}
	
	
	//subscribers
	void Metal_Detected(const std_msgs::Bool::ConstPtr& msg)
	{
		//...
		metal_detected=msg->data;
		
	}
	
	void Goal_Dist(const std_msgs::Float64::ConstPtr& msg)
	{
		//...
		double goal_distance=msg->data;
		
		if (goal_published)
		{
			if(goal_distance<0.3)
			{
			//change mode when the distance was to goal is lower than thresh...
			mode.data=HEADLAND_TURN;
			mode_pub.publish(mode);
			//move_motor_up(6.0);
			turn_magnets_off();
			sleep(2.0);
			//move_motor_down(3.0);
			turn_magnets_on();
			//moving on to next goalpoint....
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
	std::string mode_str,cmd_vel_str, cmd_motor_str,cmd_light_str,cmd_magnet_str,goal_str,goal_dist_str,metal_detected_str;
	
	ros::init(argc,argv,"metal_speed_controller");
	ros::NodeHandle n("~");
	
	//read params of launch file
	
	Sender_Node s;
	
	n.param<std::string>("mode_pub", mode_str, "/mode"); 
	n.param<std::string>("frame_id", s.frame_id, "/base_link"); 
	n.param<std::string>("cmd_vel_static", cmd_vel_str, "/cmd_vel_static");
	n.param<std::string>("cmd_motor", cmd_motor_str, "/cmd_motor");
	n.param<std::string>("cmd_light", cmd_light_str, "/cmd_light");
	n.param<std::string>("cmd_magnet", cmd_magnet_str, "/cmd_magnet");
	n.param<std::string>("goal", goal_str, "/goal");
	n.param<std::string>("goal_dist", goal_dist_str, "/fmControllers/distance");
	n.param<bool>("mode_changer_active", s.mode_changer_active,true);
	n.param<std::string>("metal_detected_sub", metal_detected_str,"metal_detected");
	
	n.param<double>("time_delay", s.time_delay,1.0);
	
	//subscribers for the error stop handling
	ros::Subscriber goal_d=n.subscribe(goal_dist_str,5,&Sender_Node::Goal_Dist,&s);
	ros::Subscriber metal=n.subscribe(metal_detected_str,5,&Sender_Node::Metal_Detected,&s);	
	
	//publishers
	s.pub_goal = n.advertise<geometry_msgs::PoseStamped>(goal_str.c_str(),10);
	s.pub_cmd_vel = n.advertise<geometry_msgs::Twist>(cmd_vel_str.c_str(),10);
	s.pub_cmd_motor =n.advertise<geometry_msgs::Twist>(cmd_motor_str.c_str(),10);
	s.pub_cmd_light = n.advertise<geometry_msgs::Twist>(cmd_light_str.c_str(),10);
	s.pub_cmd_magnet = n.advertise<geometry_msgs::Twist>(cmd_magnet_str.c_str(),10);

	s.mode_pub = n.advertise<msgs::IntStamped>(mode_str.c_str(),10);
	
	s.init();
	
	//define the update rate with a seperate function...
	ros::Timer t;
	t = n.createTimer(ros::Duration(0.01), &Sender_Node::PublishSpeed,&s);
	
	//ros::Timer t2;
	//t2 = n.createTimer(ros::Duration(0.01), &Sender_Node::Mode_Changer,&s);
	
	
	ros::spin();
		
}
