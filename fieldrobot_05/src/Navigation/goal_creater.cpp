#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "pathParser.h"
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include <nav_msgs/Path.h>

using namespace std;

class SimpleGoal
{
private:	

	int actual_wp;
	double frequency;
	bool read_mission_file;
	bool publish_goals;	
	double  min_distance;
	string filename;
	bool wait_for_userstart;
	bool reached_last_waypoint;
	bool usergoal_received;
	double wp_approximation;
	double waittime;
	double path_frequency;
	double actual_distance;
	bool distance_received;
	string fixed_frame;
	string rabbit_frame;
	string robot_frame;
	
	pathParser waypoints;
	nav_msgs::Path path;
	
	// public variables
public:
	
	tf::TransformBroadcaster br;

	ros::Publisher goal_pub, path_pub;
	geometry_msgs::TransformStamped goal_trans;
	geometry_msgs::PoseStamped goal, mission_goal, usergoal;
	
	std_msgs::Float64 goal_distance;
	bool slow_down;
	
	//start with normal constructor
	SimpleGoal(double min_distance, pathParser waypoints, double frequency)
	{
		//set up dynamic reconfigure server
		distance_received=false;
	    usergoal_received=false;
		//set variables local to object
		this->min_distance=min_distance;
		this->waypoints=waypoints;
		this->frequency=frequency;
		//counter for waypoint list
		actual_wp = 0;
		
		//set first goal to actual position:
		goal.header.stamp=ros::Time::now();
		goal.header.frame_id="robot_frame";
		goal.pose.position.x=0;
		goal.pose.position.y=0;
		ROS_INFO("start endless loop");
		//publishNextGoal();	
		goal_pub.publish(goal);	
		
		sleep(10);
		readPath();
		//for visualisation publish it
		publish_Path();
		get_next_wp();
		goal_pub.publish(goal);	
		//start lop with frequency out of dynamic reconfigure
		//start endless loop until node ends
		goal_creater_loop();

	}
	
	~SimpleGoal()
	{
	}
	
	void goal_creater_loop()
	{
	
		ROS_INFO("goal schleife");
		
			if(read_mission_file)
			{
			//overwrite the path file...
			ROS_INFO("reading new file...");
			pathParser new_waypoints(filename);
			waypoints=new_waypoints;
			readPath();
			}
			//check distance to goal
			if(!reached_last_waypoint)
			{
				ROS_INFO("actual_distance: %f",actual_distance);
					//ROS_INFO("Distance to waypoint % f", msg->data);
					if (actual_distance<min_distance && actual_distance!=0)
					{
					get_next_wp();
					}	
			}
			if (usergoal_received)
			{
				//this has prio A so move this goal fast to the output goal
				//scip first the pathplaning.. maybe make this later a little bit smarter..
				reached_last_waypoint=true;
				goal=usergoal;	
			}
			
			publishNextGoal();	
			goal_pub.publish(goal);				
			
			publish_Path();
		
	}
	
	void readPath()
	{
		reached_last_waypoint=false;
		path.header.frame_id=fixed_frame;
		path.header.stamp=ros::Time::now();
		
		for (int i=0;i<waypoints.path.size();i++)
		{
			//ROS_INFO("create goalpath");
			geometry_msgs::PoseStamped point;
			//point.header.stamp=ros::Time::now();
			point=waypoints.path[i];
			//point.pose.position.y=waypoints.path[i].pose.orientation.y;
			path.poses.push_back(point);
		}	
			
	}
	
			
	void publish_Path()
	{
		//publish the received path
		path.header.stamp=ros::Time::now();
		path_pub.publish(path);
	}
	
	void get_next_wp()
	{
		ROS_INFO("get next waypoint!");
		//check if actual wp is the last waypoint...
		if(actual_wp<waypoints.path.size()-1)
		{
			goal=waypoints.path[actual_wp];
			actual_wp++;
		} else
		{
			ROS_INFO("reached last waypoint!");
			reached_last_waypoint=true;
		}
	
	}
	
	void check_usergoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		usergoal_received=true;
		//write user goal input 
		usergoal=*msg;	
		ROS_INFO("got usergoal");
	}
	
	void check_for_next_waypoint(const std_msgs::Float64::ConstPtr& msg)
	{
		distance_received=true;
		actual_distance=msg->data;
		//start the whole procedure
		goal_creater_loop();
	}
	
	void publishNextGoal()
	{
				//li.waitForTransform(fixed_frame,goal.header.frame_id,goal.header.stamp,ros::Duration(1.0));
				//li.transformPose(fixed_frame,goal.header.stamp,usergoal, goal.header.frame_id,goal);	
				ROS_INFO("got trafo");
				 // publish the transform of the next goal to follow
				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(goal.pose.orientation.z);
				goal_trans.header.stamp = ros::Time::now();
				goal_trans.header.frame_id =fixed_frame;
				goal_trans.child_frame_id = rabbit_frame;
				goal_trans.transform.translation.x = goal.pose.position.x;
				goal_trans.transform.translation.y = goal.pose.position.y;
				goal_trans.transform.translation.z = 0.0;
				goal_trans.transform.rotation = odom_quat;
				br.sendTransform(goal_trans);
				
	}
		

};
//-------------------------------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_creater");

ros::NodeHandle n("~");

ros::Publisher pub_goal;
ros::Subscriber sub_goal_distance,sub_usergoal;
std::string wp_str,wp_list_str,pose_str,path_pub_str, usergoal_str;
double m_distance,f;
bool slow_down;

	  if(!n.hasParam("next_waypoint")) ROS_WARN("Used default parameter for next_waypoint");
	  n.param<std::string>("next_waypoint", wp_str, "/goal"); 
	  if(!n.hasParam("distance")) ROS_WARN("Used default parameter for distance");
	  n.param<std::string>("distance", pose_str, "/goal_distance"); 
	  if(!n.hasParam("waypoint_list")) ROS_WARN("Used default parameter for waypoint_list");
	  n.param<std::string>("waypoint_list", wp_list_str, "waypoint.txt"); 
	  //if(!n.hasParam("min_distance")) ROS_WARN("Used default parameter for min_distance");
	  n.param<double>("min_distance", m_distance, 0.5);
	  if(!n.hasParam("path_publisher")) ROS_WARN("Used default parameter for path_publisher");
	  n.param<std::string>("path_publisher", path_pub_str, "/goalpath"); 
	  n.param<double>("frequency", f, 5); 
	   if(!n.hasParam("user_goal_input")) ROS_WARN("Used default parameter for user_goal input");
	  n.param<std::string>("user_goal_input", usergoal_str, "/move_base_simple/goal"); 
		  
	  pathParser waypoints(wp_list_str.c_str());
      //create object for waypoint management
      SimpleGoal g(m_distance,waypoints,f);
      g.slow_down=slow_down;
      
      
      sub_goal_distance = n.subscribe(pose_str,15,&SimpleGoal::check_for_next_waypoint,&g);
      sub_usergoal= n.subscribe(usergoal_str,15, &SimpleGoal::check_usergoal, &g);
	 
	  g.goal_pub=n.advertise<geometry_msgs::PoseStamped>(wp_str.c_str(), 100);
	  g.path_pub=n.advertise<nav_msgs::Path>(path_pub_str.c_str(), 100);
	
	ros::spin();
  

}
