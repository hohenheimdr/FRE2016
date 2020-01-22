/*
 * path_writer.cpp
 *
 *  Created on: 2.12, 2014
 *      Author: dreiser
 * writing back a path with given frequency out of odometry information
 */

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include <iostream>
#include <fstream>


using namespace std;

class PathWriter
{
public:

//global object variables
ros::Publisher path_publisher;
std::string frame_id,file;
nav_msgs::Path path;
//not used at the moment
double frequency,odometry_error;
bool record_file;
geometry_msgs::PoseStamped last_point, point;

ofstream myfile;

	PathWriter(double frequency,std::string frame_id,bool record_file)
	{
		//set variables local to object
		this->record_file=record_file;
		this->frequency=frequency;
		path.header.stamp=ros::Time::now();
		path.header.frame_id=frame_id;

	}
	~PathWriter()
	{
		myfile.close();
	}
	
	void check_Position(const nav_msgs::Odometry::ConstPtr& msg)
	{
		ROS_INFO("reading pose");	
		point.pose=msg->pose.pose;
		point.header.stamp=ros::Time::now();
		point.header.frame_id =frame_id;
		if(last_point.pose.position.x!= point.pose.position.x || last_point.pose.position.y!=point.pose.position.y)
		{
			//define the moved distance to the last point
			double delta_x=abs(last_point.pose.position.x-point.pose.position.x);
			double delta_y=abs(last_point.pose.position.y-point.pose.position.y);
			double dist=sqrt(delta_x*delta_x+delta_y*delta_y);
			if (dist>odometry_error)
			{
				path.header.stamp=ros::Time::now();
				point.header.stamp=ros::Time::now();
				point.header.frame_id=frame_id;
				path.poses.push_back(point);
				//write back the point...
				last_point=point;
				if (record_file && !myfile.is_open())
				{
					myfile.open (file.c_str());
				}	
				if (myfile.is_open())
				{
					ROS_INFO("writing to file");
					//myfile <<"test \n";
					double waittime=0;
					//printf(myfile,"%f,%f \n",point.pose.position.x,point.pose.position.y);
					//myfile <<std::fixed << std::setprecision(15)<<point.pose.position.x <<"," << point.pose.position.y <<"\n";
					myfile <<std::fixed << std::setprecision(15)<<point.pose.position.x <<"," << point.pose.position.y <<","<<point.pose.position.z<<","<<waittime
					<<","<<point.pose.orientation.x <<","<<point.pose.orientation.y<<","<<point.pose.orientation.z<<"\n";
					
				}
			}	
			
		}
	
		path_publisher.publish(path);
		//sleep for a while...
		ros::Rate r(frequency);
		r.sleep();
				
	}
		
};



int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "path_writer");
	ros::NodeHandle n("~");
	double f;
	bool record_file;
	
	std::string odom_sub_topic,path_pub_topic,frame, file;

	n.param<std::string>("odom_subscriber_topic",odom_sub_topic,"/fmKnowledge/odometry");
	n.param<std::string>("path_publisher_topic",path_pub_topic,"/fmExecutors/path");
	n.param<double>("frequency", f, 0.5);
	n.param<std::string>("frame_id",frame,"robot_frame");
	n.param<std::string>("output_file",file,"/home/talos/test.txt");
	n.param<bool>("record_file",record_file,false);
	
	PathWriter p(f,frame,record_file);
	p.file=file;
	p.path_publisher=n.advertise<nav_msgs::Path>(path_pub_topic.c_str(), 100);
	n.param<double>("odometry_error",p.odometry_error,0.1);
	
	ros::Subscriber odometry_sub = n.subscribe(odom_sub_topic,15,&PathWriter::check_Position,&p);
	sleep(1);
	//overwrite last point with actual odometry value recieved
	p.last_point=p.point;
		
	  
	ros::spin();

	return 0;

}
