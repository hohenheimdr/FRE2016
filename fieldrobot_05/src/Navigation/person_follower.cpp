
/*
 * person_follower.cpp
 *
 *  Created on: 28.1.2015
 *   Author: dreiser
 * reading the topic of leg_detector and write back 
 * the position as qa new goal where the robot should follow
 */

//#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include <iostream>
#include <fstream>

using namespace std;

class PersonFollower
{
public:
ros::Publisher position_publisher;
geometry_msgs::PoseStamped goal,dummy;
tf::TransformListener li;
tf::TransformBroadcaster br;
geometry_msgs::TransformStamped goal_trans;

	PersonFollower()
	{
	}
	
	~PersonFollower()
	{
	}
	
	void check_Position(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
	{
		if(msg->people.size()>0)
		{
		ros::Time now = ros::Time::now();
		goal.header.stamp=now;
		goal.header.frame_id="lmslaser"; //msg->people[0].header.frame_id;
		goal.pose.position.x=msg->people[0].pos.x;
		goal.pose.position.y=msg->people[0].pos.y;
	
		//now publish this position to the goal publisher
		position_publisher.publish(goal);
		//add transform to follow by rabbit_follower	
		
		// publish the transform message to follow new goalpoint
	       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(goal.pose.orientation.z);
			goal_trans.header.stamp = ros::Time::now();
			goal_trans.header.frame_id = goal.header.frame_id;
			goal_trans.child_frame_id =  "person_goal";
			goal_trans.transform.translation.x = goal.pose.position.x;
			goal_trans.transform.translation.y = goal.pose.position.y;
			goal_trans.transform.translation.z = 0.0;
			goal_trans.transform.rotation = odom_quat;
			br.sendTransform(goal_trans);
		
		}	
	}

};



int main(int argc, char **argv)
{
	ros::init(argc, argv,"person_follower");
	ros::NodeHandle n("~");
	ROS_INFO("person_follower_started");
	
	std::string sub_topic,pub_topic;

	n.param<std::string>("sub_topic",sub_topic,"/people_tracker_measurements");
	n.param<std::string>("pub_topic",pub_topic,"/person_position");
	
	PersonFollower p;
	ros::Subscriber person_sub = n.subscribe(sub_topic,15,&PersonFollower::check_Position,&p);
		
	p.position_publisher=n.advertise<geometry_msgs::PoseStamped>(pub_topic.c_str(), 100);
	 
	ros::spin();

	return 0;
}
