#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include "pathParser.h"
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


/* reading the front and back cloud and give back at what side there is 
 * space to turn clouds just get published when there are points in
 * defined range of laser_pcd_node
 * this signal has priority A when it  comes to adaptive obstacle
 * avoidance
 * by David Reiser 3.12.14
 * */

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;


class LaserObstacle
{
public:

// public variables
	ros::Publisher front_left_pub,front_right_pub,back_left_pub,back_right_pub;
	geometry_msgs::TwistStamped cmd_vel_error;
	geometry_msgs::TwistStamped cmd_vel;
	bool obstacle_front_left,obstacle_front_right, obstacle_back_left,obstacle_back_right;
	
	int error_points;

	LaserObstacle(int points)
	{
		this->error_points=points;	
		obstacle_front_left=false;
		obstacle_front_right=false;
		obstacle_back_left=false;
		obstacle_back_right=false;
		publish_data();
	}
	
	~ LaserObstacle()
	{
	}
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		obstacle_front_left=false;
		obstacle_front_right=false;
		//create a pcl point cloud first...
		 pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
		//check if obstacle was detected
		if(msg->width>error_points)
		{
				//now there is some processing neccessary...convert msg to pcl point cloud		
				pcl::fromROSMsg(*msg, *input);
				for (int i=0;i<input->points.size();i++)
				{
					
					double y=input->points[i].y;
					if(y<0)
					{
						obstacle_front_right=true;
					}
					if(y>=0)
					{
						obstacle_front_left=true;
					}  
					
				}
						
		}
		
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
		
				
		publish_data();
		
	}

	void readBackCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		obstacle_back_left=false;
		obstacle_back_right=false;
		
		if(msg->width>error_points)
		{
			obstacle_back_left=true;
			obstacle_back_right=true;						
		}
	publish_data();
	}
	
	void publish_data()
	{
		
		//ROS_INFO( "Publish the info..");
		front_left_pub.publish(obstacle_front_left);
		front_right_pub.publish(obstacle_front_right);
		back_left_pub.publish(obstacle_back_left);
		back_right_pub.publish(obstacle_back_right);
		ros::Rate f(10);
		f.sleep();
		
	}
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

std::string front_str,back_str,back_left_str,back_right_str,front_left_str,front_right_str;
int ep;

	  if(!n.hasParam("front_cloud")) ROS_WARN("Used default parameter for front_cloud");
	  n.param<std::string>("front_cloud", front_str, "/error_cloud_front"); 
	  if(!n.hasParam("back_cloud")) ROS_WARN("Used default parameter for back_cloud");
	  n.param<std::string>("back_cloud", back_str, "/error_cloud_back"); 
	  if(!n.hasParam("back_left")) ROS_WARN("Used default parameter for back_left");
	  n.param<std::string>("back_left", back_left_str, "/back_left"); 
	  if(!n.hasParam("back_right")) ROS_WARN("Used default parameter for back_right");
	  n.param<std::string>("back_right", back_right_str, "/back_right"); 
	  if(!n.hasParam("front_left")) ROS_WARN("Used default parameter for front_left");
	  n.param<std::string>("front_left", front_left_str, "/front_left"); 
	  if(!n.hasParam("front_right")) ROS_WARN("Used default parameter for front_right");
	  n.param<std::string>("front_right", front_right_str, "/front_right"); 
	  n.param<int>("point_ignore", ep, 5);

	  
	 LaserObstacle o(ep);	  
	 ros::Subscriber sub_laser_front = n.subscribe(front_str,1, &LaserObstacle::readFrontCloud, &o);
	 ros::Subscriber sub_laser_back  = n.subscribe(back_str,1, &LaserObstacle::readBackCloud, &o);
	  
      //create publishers
      o.front_left_pub=n.advertise<std_msgs::Bool>(front_left_str.c_str(), 1);
      o.front_right_pub=n.advertise<std_msgs::Bool>(front_right_str.c_str(), 1);
      o.back_left_pub=n.advertise<std_msgs::Bool>(back_left_str.c_str(), 1);
      o.back_right_pub=n.advertise<std_msgs::Bool>(back_right_str.c_str(), 1);
	
	ros::spin();
  

}
