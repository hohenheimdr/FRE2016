#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "pathParser.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <ros/subscriber.h>
#include <nav_msgs/Path.h>
#include <goal_manager/FrameGoalManagerConfig.h>
#include <dynamic_reconfigure/server.h>

/* Goal Manager optimized for the 3D Frame of the phoenix
 * autor: david
 * date: 20.1.16
 * function: 	reads the inputfile provided in the launch file settings "waypoint_list" as string, 
 * 				This coordinates are ordered in rows like x,y,z,waittime
 * 				coordinates get represented in the "fixed_frame" Frame
 * 				when the file should wait at each goal point for a topic this could be defined also
 * 				in "wait_for_topic" und wait_topic_name as std_msgs::Bool
 * 				also its possible to add configure mode, means, that the goal can be defined by the 
 * 				configure slides in dynamic reconfiguration 
 * 
 * topics out:	PointStamped. 
 * 
 * */

using namespace std;

class SimpleGoal
{
private:	

	int actual_wp;
	ros::Duration goal_waittime; //time to wait at actual goalpoint
	bool read_mission_file, publish_goals, wait_for_userstart, reached_last_waypoint;
	bool usergoal_received, distance_received, frequency,topic_status,configure_received;
	double wp_approximation, path_frequency;
	double actual_distance;
	string filename,lastfilename;
	string fixed_frame, rabbit_frame, robot_frame;
	pathParser waypoints;
	nav_msgs::Path path;
	
	// public variables
public:
	dynamic_reconfigure::Server<goal_manager::FrameGoalManagerConfig> server_;
	tf::TransformListener li;
	tf::TransformBroadcaster br;
	double waittime;
	bool wait_for_topic;

	ros::Publisher goal_pub, path_pub;
	geometry_msgs::TransformStamped goal_trans, goal_transform;
	geometry_msgs::PoseStamped goal, mission_goal, usergoal;
	geometry_msgs::Point frame_goal;
	
	//start with normal constructor
	SimpleGoal(double min_distance, pathParser waypoints)
	{
	    distance_received=false;
	    usergoal_received=false;
	    configure_received=false;
		//set variables local to object
		wp_approximation=min_distance;
		this->waypoints=waypoints;
		//counter for waypoint list
		actual_wp = 0;
		//reset topic status!
		topic_status=false;
		//check if there is already a valid path
		readPath();
		//set first goal to actual position:
		goal.header.stamp=ros::Time::now();
		goal.header.frame_id=robot_frame;
		goal.pose.position.x=0;
		goal.pose.position.y=0;
		goal.pose.position.z=0;
		frame_goal.x=0;
		frame_goal.y=0;
		frame_goal.z=0;
		
		//set up dynamic reconfigure server
		dynamic_reconfigure::Server<goal_manager::FrameGoalManagerConfig>::CallbackType f;
	    f = boost::bind(&SimpleGoal::configure,this, _1, _2);
	    server_.setCallback(f);
		
		//finished start up
		
	}
	
	~SimpleGoal()
	{
	}
	
	//void goal_creater_loop(const ros::TimerEvent& e)
	void goal_creater_loop()
	{
		while(!reached_last_waypoint)
		{
			get_next_wp();
			
			if (usergoal_received)
			{
				//this has prio A so move this goal fast to the output goal
				//scip first the pathplaning.. maybe make this later a little bit smarter..
				reached_last_waypoint=true;
				//check if it is a new usergoal
				if(goal.pose.position.x!=usergoal.pose.position.x)
				{
					if(goal.pose.position.y!=usergoal.pose.position.y)
					{ 
						goal=usergoal;
						goal.header.stamp=ros::Time::now();			
					}
				}	
			}
			//just write goal constant if no configure was received!
			while(configure_received)
			{
				//in this case program will never end!
				goal_pub.publish(frame_goal);
				TransformGoal();				
				publish_Path();
				ros::spinOnce();
				ros::Rate r(waittime);
				r.sleep();
			}
			frame_goal=goal.pose.position;
			goal_pub.publish(frame_goal);
			TransformGoal();				
			publish_Path();
			if(wait_for_topic)
			{
				//wait until topic status is true or configure was received
				while(!topic_status && !configure_received)
				{
					TransformGoal();	
					//read all callbacks since last call
					ros::spinOnce();
				}
				//reset topic status!
				topic_status=false;
			}
			ROS_INFO("sleeping now for %f seconds",goal_waittime.toSec());
			goal_waittime.sleep();
			TransformGoal();	
			//read all callbacks since last call
			ros::spinOnce();
			
		}
			
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
			point.header.stamp=ros::Time::now();
			point.header.frame_id=fixed_frame;
			point=waypoints.path[i];
			//point.pose.position.y=waypoints.path[i].pose.orientation.y;
			path.poses.push_back(point);
		}
		
	}
	
	void configure(goal_manager::FrameGoalManagerConfig &config, uint32_t level) {
		ROS_INFO("reading dynamic reconfigure files");
		if(frame_goal.x!=0 || frame_goal.y!=0 || frame_goal.z!=0)
		{
			//recieved input of user! send goal....
			configure_received=true;
			ROS_INFO("program in configure mode! moving to (%f,%f,%f) No break",frame_goal.x,frame_goal.y,frame_goal.z);
		}
		frame_goal.x=config.x_position;
		frame_goal.y=config.y_position;
		frame_goal.z=config.z_position;
		goal.pose.position=frame_goal;
	
	    waittime=config.waittime;
	    fixed_frame=config.fixed_frame;
	    rabbit_frame=config.rabbit_frame;
	    robot_frame=config.robot_frame;
	}
		
	void publish_Path()
	{
		//publish the received path
		path.header.stamp=ros::Time::now();
		path.header.frame_id=fixed_frame;
		path_pub.publish(path);
	}
	
	void get_next_wp()
	{
		
		ROS_INFO("get next waypoint!");
		//check if actual wp is the last waypoint...
		if(actual_wp<waypoints.path.size()-1)
		{
			goal=waypoints.path[actual_wp];
			goal.header.frame_id=robot_frame;//fixed_frame;
			//read time to wait what was provided from the goal list
			goal_waittime=ros::Duration(goal.header.stamp.toSec());
			goal.header.stamp=ros::Time::now();
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
		frame_goal=msg->pose.position;
		//usergoal.header.frame_id=fixed_frame;
		usergoal.header.stamp=ros::Time::now();	
		ROS_INFO("got usergoal");	
	}
	
	void check_topic(const std_msgs::Bool::ConstPtr& msg)
	{
		topic_status=msg->data;
		//got ready status from topic
		ROS_INFO("got new status");	
	}
	
	void TransformGoal()
	{
		
		try{
				//check if transform is neccessary
				if(goal.header.frame_id!=robot_frame)
				{
				tf::StampedTransform transform;
				//ros:Time(0) provides the last available transform!
				li.waitForTransform(fixed_frame,robot_frame,ros::Time(0),ros::Duration(3.0));		
				li.lookupTransform(fixed_frame,robot_frame,ros::Time(0), transform);	
				//li.transformPose(fixed_frame,ros::Time(0),goal,robot_frame,goal);
				goal.header.frame_id=robot_frame;
				}
				// publish the transform of the next goal to follow to the rabbit frame
			}
			catch(tf::TransformException ex)
			{
				ROS_INFO("error with transform from %s to %s",robot_frame.c_str(),fixed_frame.c_str());				
			}
			//solve transform...maybe with wrong values?
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(goal.pose.orientation.z);
			
			goal_trans.header.stamp = ros::Time::now();
			goal_trans.header.frame_id =robot_frame;
			goal_trans.child_frame_id =rabbit_frame;
			goal_trans.transform.translation.x = goal.pose.position.x;
			goal_trans.transform.translation.y = goal.pose.position.y;
			goal_trans.transform.translation.z = goal.pose.position.z;
			goal_trans.transform.rotation = odom_quat;
			br.sendTransform(goal_trans);				
	}
			
};
//--------------------main-----------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_creater");

ros::NodeHandle n("~");

std::string wp_str,wp_list_str,pose_str,path_pub_str, usergoal_str,topic_name_str;
double m_distance;
bool slow_down;

	  if(!n.hasParam("next_waypoint")) ROS_WARN("Used default parameter for next_waypoint");
	  n.param<std::string>("next_waypoint", wp_str, "/goal"); 
	  if(!n.hasParam("distance")) ROS_WARN("Used default parameter for distance");
	  n.param<std::string>("distance", pose_str, "/goal_distance"); 
	  if(!n.hasParam("waypoint_list")) ROS_WARN("Used default parameter for waypoint_list");
	  n.param<std::string>("waypoint_list", wp_list_str, "waypoint.txt"); 
	  n.param<std::string>("path_publisher", path_pub_str, "/goalpath");
	  n.param<std::string>("user_goal_input", usergoal_str, "/move_base_simple/goal"); 
	  n.param<std::string>("wait_topic_name",topic_name_str, "test");
	  
	  n.param<double>("min_distance", m_distance, 0.5);
	  //read actual points
	  pathParser waypoints(wp_list_str.c_str());
      //create object for waypoint management
	  SimpleGoal g(m_distance,waypoints);
	  
	  n.param<bool>("wait_for_topic", g.wait_for_topic,false); 
     //publishers:
      g.goal_pub=n.advertise<geometry_msgs::Point>(wp_str.c_str(), 100);
	  g.path_pub=n.advertise<nav_msgs::Path>(path_pub_str.c_str(), 100);
	  //subscribers:
      //sub_goal_distance = n.subscribe(pose_str,15,&SimpleGoal::check_for_next_waypoint,&g);
      ros::Subscriber sub_usergoal= n.subscribe(usergoal_str,15, &SimpleGoal::check_usergoal, &g);
      ros::Subscriber sub_topic= n.subscribe(topic_name_str,15, &SimpleGoal::check_topic, &g);
		
	  g.goal_creater_loop();
	 //stop program after last goal reached.
	 
}
