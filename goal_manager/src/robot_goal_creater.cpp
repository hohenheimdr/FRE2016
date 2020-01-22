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
#include <tf/transform_datatypes.h>

/* Goal Manager optimized for the 3D navigation of robot talos
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
 * dr	added relative goals as 
 * 
 * */

using namespace std;

class SimpleGoal
{
private:	

	int actual_wp;
	ros::Duration goal_waittime; //time to wait at actual goalpoint
	bool wait_for_userstart, reached_last_waypoint;
	bool usergoal_received, config_goal_received, topic_status,time_break;
	string fixed_frame, rabbit_frame, robot_frame;
	geometry_msgs::TransformStamped goal_trans;
	double waittime;
	pathParser waypoints;
	nav_msgs::Path path;
	ros::Time starttime;
	
	// public variables
public:
	dynamic_reconfigure::Server<goal_manager::FrameGoalManagerConfig> server_;
	tf::TransformListener li;
	tf::TransformBroadcaster br;
	bool wait_for_topic,relative_goals;
	ros::Publisher pose_goal_pub, point_goal_pub, path_pub;
	geometry_msgs::PoseStamped pose_goal,usergoal, config_goal, robot_pose;
	geometry_msgs::Point point_goal;
	double break_after_seconds;
	
	//start with normal constructor
	SimpleGoal(pathParser waypoints)
	{
		starttime=ros::Time::now();
	    usergoal_received=false;
	    time_break=false;
	    config_goal_received=false;
		//set variables local to object
		this->waypoints=waypoints;
		//counter for waypoint list
		actual_wp = 0;
		//reset topic status (checking if topic was received)
		topic_status=false;
		wait_for_topic=false;	
		//set first goal to actual position:
		pose_goal.header.stamp=ros::Time::now();
		pose_goal.header.frame_id=robot_frame;
		pose_goal.pose.position.x=0;
		pose_goal.pose.position.y=0;
		pose_goal.pose.position.z=0;
		
		//set up dynamic reconfigure server
		dynamic_reconfigure::Server<goal_manager::FrameGoalManagerConfig>::CallbackType f;
	    f = boost::bind(&SimpleGoal::configure,this, _1, _2);
	    server_.setCallback(f);
		//finished start up
		//read planed path for visualisation purpose
		readPath();
		
	}
	
	~SimpleGoal()
	{
	}
	
	void read_robot_pose()
	{
		robot_pose.header.stamp=ros::Time::now();
		robot_pose.header.frame_id=robot_frame;
		robot_pose.pose.position.x=0;
		robot_pose.pose.position.y=0;
		robot_pose.pose.position.z=0;
		robot_pose.pose.orientation.x=0;
		robot_pose.pose.orientation.y=0;
		robot_pose.pose.orientation.z=0;
		robot_pose.pose.orientation.w=1;
		tf::StampedTransform transform;
		//try{
			//reading the actual robot pose relative to fixed frame
			li.waitForTransform(fixed_frame,robot_frame,robot_pose.header.stamp,ros::Duration(3.0));		
			li.lookupTransform(fixed_frame,robot_frame,ros::Time(0), transform);	
			li.transformPose(fixed_frame,ros::Time(0),robot_pose,robot_frame,robot_pose);
			
			ROS_INFO("robot pose: x:%f,y:%f,z:%f",robot_pose.pose.position.x,robot_pose.pose.position.y,robot_pose.pose.position.z);
		
		//}catch(...)
		//{
			//ROS_INFO("error getting robot transform");
		//}
	}
	
	//void goal_creater_loop(const ros::TimerEvent& e)
	void goal_creater_loop()
	{
		
		
		if(relative_goals)
		{
			read_robot_pose();
		}
		//run this loop until last waypoint was reached
		while(!reached_last_waypoint || !time_break)
		{
			
			
			
			get_next_wp();
			
			//if status should wait for topic...move in this loop
			if(wait_for_topic)
			{
				//wait until topic status is true or configure was received
				while(!topic_status && !config_goal_received && !usergoal_received &&!time_break )
				{
					
					double time_duration=(ros::Time::now()-starttime).toSec();
					std::cout<<time_duration<<std::endl;
					if(time_duration>break_after_seconds)
					{
						if(break_after_seconds>0)
						{
						ROS_INFO("stoped program");
						time_break=true;
						//break;
						}
					}
					
					//read all callbacks since last call asnd stay in this loop until topic 
					ros::spinOnce();
					publish_goals();
					publish_goals();	
					ros::spinOnce();
					//ROS_INFO("normal loop");
				}
				//reset topic status!
				topic_status=false;
			}
			
			//just write goal constant if no configure was received!
			while(config_goal_received || usergoal_received)
			{
				//this has prio A so move this goal fast to the output goal
				//scip first the pathplaning.. maybe make this later a little bit smarter..
				if(config_goal_received && !pose_equal(config_goal,pose_goal))
				{
					ROS_INFO("setting config goal");
					pose_goal=config_goal;
					TransformGoal();
					config_goal=pose_goal;
				}
				if(usergoal_received &&!pose_equal(usergoal,pose_goal))
				{
					ROS_INFO("setting user goal");
					pose_goal=usergoal;
					TransformGoal();
					usergoal=pose_goal;
				}
				//take care, that also here the goal and transform get published...
				publish_goals();	
				ros::spinOnce();
				publish_goals();	
				ros::spinOnce();
				//in this case waittime gets set by dynamic config server
				//ros::Rate r(waittime);
				//r.sleep();	
				//as soon as this loop had been entered program will never end!	/does this need a change?
			}
			
			//if everything goes normal, we end here...
			publish_goals();	
			ros::spinOnce();
			publish_goals();	
			ros::spinOnce();	
			ROS_INFO("sleeping now for %f seconds",goal_waittime.toSec());
			goal_waittime.sleep();
			//read all callbacks since last call bevore continue
			ros::spinOnce();
				
		}//end while loop	
	}
	
	void readPath()
	{
		//check what this part is doing here! (do I need it?)
		path.header.frame_id=fixed_frame;
		path.header.stamp=ros::Time::now();
		
		for (int i=0;i<waypoints.path.size();i++)
		{
			//ROS_INFO("create goalpath");
			geometry_msgs::PoseStamped point;
			point.header.stamp=ros::Time::now();
			point.header.frame_id=fixed_frame;
			point=waypoints.path[i];
			path.poses.push_back(point);
		}	
	}
	
	void configure(goal_manager::FrameGoalManagerConfig &config, uint32_t level) {
		ROS_INFO("reading dynamic reconfigure files");
		config_goal.header.frame_id=fixed_frame;
		config_goal.header.stamp=ros::Time::now();
		config_goal.pose.position.x=config.x_position;
		config_goal.pose.position.y=config.y_position;
		config_goal.pose.position.z=config.z_position;
		
		if(!pose_equal(config_goal,pose_goal))
		{	
			//recieved input of user! send goal....
			config_goal_received=true;
			//to be shure that config goal gets sended..
			usergoal_received=false;
			ROS_INFO("program in configure mode! moving to (%f,%f,%f) No break",config_goal.pose.position.x,config_goal.pose.position.y,config_goal.pose.position.z);
			pose_goal=usergoal;
			publish_goals();
		}
		waittime=config.waittime;
	    fixed_frame=config.fixed_frame;
	    rabbit_frame=config.rabbit_frame;
	    robot_frame=config.robot_frame;
	}
	
	void publish_goals()
	{			
			//transform goal, when frame is not fixed frame
			pose_goal.header.stamp=ros::Time::now();
			PublishTransform();	
			//PublishTransform();	
			//publish the path you shoud go to...
			publish_Path();	
			//publish goal as point 
			point_goal_pub.publish(pose_goal.pose.position);
			//publish goal as pose
			pose_goal_pub.publish(pose_goal);		
	}
	
	bool pose_equal(geometry_msgs::PoseStamped a,geometry_msgs::PoseStamped b)
	{ //return true if input a and b are the same
		if (a.pose.position.x==b.pose.position.x && a.pose.position.y==b.pose.position.y && a.pose.position.z==b.pose.position.z)
		{
			if (a.pose.orientation.x==b.pose.orientation.x && a.pose.orientation.y==b.pose.orientation.y && a.pose.orientation.z==b.pose.orientation.z)
			return true;
		}
			return false;
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
			pose_goal=waypoints.path[actual_wp];
			pose_goal.header.frame_id=fixed_frame;//fixed_frame is the reference frame!
			//read time to wait what was provided from the goal list
			goal_waittime=ros::Duration(pose_goal.header.stamp.toSec());
			pose_goal.header.stamp=ros::Time::now();
			actual_wp++;
			
			if(relative_goals)
				{
					try{
						PublishTransform();
						pose_goal.header.frame_id=robot_frame+"_old";
						tf::StampedTransform transform;
						//need to transform the goals to the robot starting position...
						ROS_INFO("adding a relative transform to goal!");
						li.waitForTransform(fixed_frame,robot_frame+"_old",ros::Time(0),ros::Duration(3.0));		
						li.lookupTransform(fixed_frame,robot_frame+"_old",ros::Time(0), transform);	
						li.transformPose(fixed_frame,ros::Time(0),pose_goal,robot_frame+"_old",pose_goal);
					}catch(...)
					{
						ROS_INFO("error trying relative goal transform");
					}
					//ROS_INFO("robot pose: x:%f,y:%f,z:%f",pose_goal.pose.position.x,pose_goal.pose.position.y,robot_pose.pose.position.z);
				}
			
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
		usergoal.header.stamp=ros::Time::now();	
	}
	
	void check_topic(const std_msgs::Bool::ConstPtr& msg)
	{
		topic_status=msg->data;
		//got ready status from topic
		ROS_INFO("got new status");	
	}
	
	void PublishTransform()
	{
			goal_trans.header.stamp=ros::Time::now();
			goal_trans.header.frame_id =pose_goal.header.frame_id;
			goal_trans.child_frame_id =rabbit_frame;
			goal_trans.transform.translation.x = pose_goal.pose.position.x;
			goal_trans.transform.translation.y = pose_goal.pose.position.y;
			goal_trans.transform.translation.z = pose_goal.pose.position.z;
			goal_trans.transform.rotation = pose_goal.pose.orientation;
			br.sendTransform(goal_trans);
			
			if(relative_goals)
			{
				//publish start transform
				goal_trans.header.stamp=ros::Time::now();
				goal_trans.header.frame_id =fixed_frame;
				goal_trans.child_frame_id =robot_frame+"_old";
				goal_trans.transform.translation.x = robot_pose.pose.position.x;
				goal_trans.transform.translation.y = robot_pose.pose.position.y;
				goal_trans.transform.translation.z = robot_pose.pose.position.z;
				goal_trans.transform.rotation = robot_pose.pose.orientation;
				br.sendTransform(goal_trans);
			}			
	}
	
		
	void TransformGoal()
	{	
		try{
				
				//check if transform is neccessary
				if(pose_goal.header.frame_id!=fixed_frame)
				{
					ROS_INFO("transforming goal now");
					tf::StampedTransform transform;
					//ros:Time(0) provides the last available transform!
					li.waitForTransform(fixed_frame,pose_goal.header.frame_id,ros::Time(0),ros::Duration(3.0));		
					li.lookupTransform(fixed_frame,pose_goal.header.frame_id,ros::Time(0), transform);	
					li.transformPose(fixed_frame,ros::Time(0),pose_goal,pose_goal.header.frame_id,pose_goal);
					//li.transformPoint(fixed_frame,ros::Time(0),
					//tf::Vector3 position=transform.getOrigin();
					//tf::Quaternion a=transform.getRotation();
					//tf::Quaternion b(pose_goal.pose.orientation.x,pose_goal.pose.orientation.y,pose_goal.pose.orientation.z,pose_goal.pose.orientation.w);
					//tf::Quaternion c=a*b.inverse();
					//pose_goal.pose.orientation.x=c[0];
					//pose_goal.pose.orientation.y=c[1];
					//pose_goal.pose.orientation.z=-c[2];
					//pose_goal.pose.orientation.w=c[3];
					
					//tf::Vector3 position=transform.transformPoint(pose_goal.pose.position.x,pose_goal.pose.position.y,pose_goal.pose.position.z);
					//pose_goal.pose.position.x=pose_goal.pose.position.x;//position[0];//+pose_goal.pose.position.x;
					//pose_goal.pose.position.y=pose_goal.pose.position.y;//position[1];//+pose_goal.pose.position.y;
					//pose_goal.pose.position.z=pose_goal.pose.position.z;//position[2];//+pose_goal.pose.position.z;
					
					pose_goal.header.frame_id=fixed_frame;
					
				}
			}
			catch(tf::TransformException ex)
			{
				ROS_INFO("error with transform from %s to %s",robot_frame.c_str(),fixed_frame.c_str());				
			}		
	}
			
};
//--------------------main-----------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_creater");

ros::NodeHandle n("~");

std::string point_str,pose_str,wp_list_str,path_pub_str, usergoal_str,topic_name_str;
double m_distance;
bool slow_down;

	  if(!n.hasParam("next_waypoint")) ROS_WARN("Used default parameter for next_waypoint");
	  n.param<std::string>("next_pose", pose_str, "/goal_pose");
	   n.param<std::string>("next_point", point_str, "/goal_point");  
	  if(!n.hasParam("waypoint_list")) ROS_WARN("Used default parameter for waypoint_list");
	  n.param<std::string>("waypoint_list", wp_list_str, "waypoint.txt"); 
	  n.param<std::string>("path_publisher", path_pub_str, "/goalpath");
	  n.param<std::string>("user_goal_input", usergoal_str, "/move_base_simple/goal"); 
	  n.param<std::string>("wait_topic_name",topic_name_str, "test");
	  
	  //read actual points
	  pathParser waypoints(wp_list_str.c_str());
      //create object for waypoint management
	  SimpleGoal g(waypoints);
	  
	  n.param<bool>("wait_for_topic", g.wait_for_topic,false);
	  n.param<bool>("relative_goals", g.relative_goals,false);  
	  	  n.param<double>("break_after_seconds",g.break_after_seconds, 0);

     //publishers:
      g.point_goal_pub=n.advertise<geometry_msgs::Point>(point_str.c_str(), 100);
      g.pose_goal_pub=n.advertise<geometry_msgs::PoseStamped>(pose_str.c_str(), 100);
	  g.path_pub=n.advertise<nav_msgs::Path>(path_pub_str.c_str(), 100);
	  //subscribers:
      //sub_goal_distance = n.subscribe(pose_str,15,&SimpleGoal::check_for_next_waypoint,&g);
      ros::Subscriber sub_usergoal= n.subscribe(usergoal_str,15, &SimpleGoal::check_usergoal, &g);
      ros::Subscriber sub_topic= n.subscribe(topic_name_str,15, &SimpleGoal::check_topic, &g);
		
	  g.goal_creater_loop();
	 //stop program after last goal reached.
	 
}
