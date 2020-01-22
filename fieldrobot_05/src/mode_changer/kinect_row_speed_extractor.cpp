#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <ros/subscriber.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Bool.h>

#include <msgs/IntStamped.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>




/* read the filtered 2D data of the kinect2 and create out of it a moving command for the robot..
 * can be used indoors or also for row navigation
 * gives back also if the headland was detected?
 * */

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;

#define USER_INPUT 0;	
#define OBSTACLE 1;
#define ROW_NAVIGATION 2;
#define HEADLAND_TURN 3;
#define STATIC 4;


class KinectObstacle
{
public:

	msgs::IntStamped mode;

// public variables
	geometry_msgs::TwistStamped cmd_vel;
	
	ros::Publisher pub_cmd_vel, mode_pub;
	std::string frame_id;
	double speed_max,angle_max, stop_radius,save_radius,action_radius;
	bool headland;
	
	
	KinectObstacle()
	{
		headland=false;
		
	}
	
	~ KinectObstacle()
	{
	}
	
	//-------continuous publisher called by the node to garante a constant output---------------
	void PublishSpeed(const ros::TimerEvent& e)
	{
		
		if (headland)
		{mode.data=HEADLAND_TURN;
			mode.header.stamp=ros::Time::now();
		 }
		else
		{
		mode.data= ROW_NAVIGATION;
		mode.header.stamp=ros::Time::now();	 
		 }
		
		mode_pub.publish(mode);
		//sleep(3.0);
		
			
	}
	//subscribers
	void Row(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		
		//create a pcl point cloud first...and transform
		 pcl::PointCloud<pcl::PointXYZRGB>::Ptr input (new pcl::PointCloud<pcl::PointXYZRGB>);
		 pcl::fromROSMsg(*msg,*input);	
		 //collect all points with a minimum distance defined.
		 
		 double orientation;
		 int count=0;
		 double dist_min=action_radius;
		 for (int i; i<input->points.size();i++)
		 {
			 double distance=sqrt(input->points[i].x*input->points[i].x+input->points[i].y*input->points[i].y);
			 
			 //just use point when...
			 if(distance<action_radius)
			 {
				 //save min distance
				 if (dist_min>distance && distance>0.31)
				 {
					 dist_min=distance;
				 }
				 //set orientation
				 if (distance>stop_radius && distance < action_radius)
				 {
				 orientation=orientation+input->points[i].y;
				 count++; 
				}
				//check if slow down is neccessary? 
			 
			}
		 }
		 
		 orientation=orientation/float(count);
		 
		 
		 //double angle=orientation;
		 //angle=angle/angle_max;
		 cmd_vel.twist.angular.z=orientation;
		 double speed=(dist_min-stop_radius)/(action_radius-stop_radius);
		 //speed=speed_maxspeed;
		 cmd_vel.twist.linear.x=speed;
		
		if(dist_min<stop_radius)
		{
				 cmd_vel.twist.linear.x=0;
				 //cmd_vel.twist.angular.z=0;
		 }
	
		ROS_INFO("orientation %f", orientation);
		ROS_INFO("row_speed %f, %f, min_dist %f ",cmd_vel.twist.linear.x,cmd_vel.twist.angular.z,dist_min);
		pub_cmd_vel.publish(cmd_vel);
		
	}
	
	
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

std::string mode_str,kinect_sub_str, cmd_vel_str;
	
	ros::init(argc,argv,"kinect_row_speed_controller");
	ros::NodeHandle n("~");
	
	//read params of launch file
	
	KinectObstacle s;

	n.param<std::string>("mode_pub", mode_str, "/mode"); 
	n.param<std::string>("kinect_data", kinect_sub_str, "/kinect_filtered"); 
	n.param<std::string>("frame_id", s.frame_id, "/base_link"); 
	n.param<std::string>("cmd_vel", cmd_vel_str, "/cmd_vel_row");
	n.param<double>("speed_max", s.speed_max, 1.0); 
	n.param<double>("angle_max", s.angle_max, 1.0); 
	n.param<double>("stop_radius", s.stop_radius, 1.0); 
	n.param<double>("save_radius", s.save_radius, 1.0);
	n.param<double>("action_radius", s.action_radius, 1.0);

	//subscribers for the error stop handling
	ros::Subscriber row=n.subscribe(kinect_sub_str,5,&KinectObstacle::Row,&s);	
	
	//publishers
	s.pub_cmd_vel = n.advertise<geometry_msgs::TwistStamped>(cmd_vel_str.c_str(),10);
	s.mode_pub = n.advertise<msgs::IntStamped>(mode_str.c_str(),10);
	
	//define the update rate with a seperate function...
	ros::Timer t;
	t = n.createTimer(ros::Duration(0.02), &KinectObstacle::PublishSpeed,&s);
	
	ros::spin();
  

}
