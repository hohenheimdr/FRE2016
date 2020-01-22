#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



 //by Max Staiger & Markus Ullrich 17.3.16
 

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;


class LaserObstacle
{
public:														//Bestimmung der Variablen

	ros::Publisher front_pub;
	std_msgs::Bool obstacle_front;
	
	int error_points;

	LaserObstacle(int points)
	{
		this->error_points=points;	
		obstacle_front.data=false;
	    	
		
	}
	
	~ LaserObstacle()
	{
	}
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		obstacle_front.data=false;
		
		if(msg->width>error_points)																//Kontrolle ob Hindernis vorhanden	
		{	
				obstacle_front.data=true;					
		}
					
		front_pub.publish(obstacle_front);
	}	

	
};

int main(int argc, char** argv){

ros::init(argc, argv, "obstacle_filter");

ros::NodeHandle n("~");

std::string front_str, obstacle_front;
int ep;

	  n.param<std::string>("cloud_in", front_str, "/scan_mitte");									//Eingehende PCL
	  n.param<std::string>("info_out", obstacle_front, "/obstacle_detection_info"); 					//Ausgehende Info)
	  n.param<int>("point_ignore", ep, 5);

	  
	 LaserObstacle o(ep);	  
	 ros::Subscriber sub_laser_front = n.subscribe(front_str,50, &LaserObstacle::readFrontCloud, &o);	//Einlesen der FrontCloud
	
      //create publishers
      o.front_pub=n.advertise<std_msgs::Bool>(obstacle_front.c_str(), 10);						//Erstellen vom Publisher
   
	
	ros::spin();
  

}
