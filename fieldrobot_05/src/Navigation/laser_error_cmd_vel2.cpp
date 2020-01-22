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
#include <msgs/IntStamped.h>

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
 * adjusted for modechanger 3.7.15
 * */

#define USER_INPUT 0;	
#define OBSTACLE 1;
#define AUTOMATIC 2;
#define HEADLAND_TURN 3;
#define STATIC 4;

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;


class LaserObstacle
{
private:
msgs::IntStamped mode;
	
public:

// public variables
	ros::Publisher front_left_pub,front_right_pub,back_left_pub,back_right_pub;
	ros::Publisher pub_cmd_vel,mode_pub;
	geometry_msgs::TwistStamped cmd_vel_error;
	geometry_msgs::TwistStamped cmd_vel;
	bool obstacle_front_left,obstacle_front_right, obstacle_back_left,obstacle_back_right,obstacle_back,obstacle_front,mode_changer_active;
	bool was_active;
	int error_points;

	LaserObstacle(int points)
	{
		this->error_points=points;	
		was_active=false;
		obstacle_front=false;
		obstacle_front_left=false;
		obstacle_front_right=false;
		obstacle_back_left=false;
		obstacle_back_right=false;
		obstacle_back=false;
	}
	
	~ LaserObstacle()
	{
	}
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		obstacle_front_left=false;
		obstacle_front_right=false;
		obstacle_front=false;
		//create a pcl point cloud first...
		 pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
		//check if obstacle was detected
		if(msg->width>error_points)
		{
			obstacle_front=true;
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
		if(obstacle_front || obstacle_back)
			{
				if(obstacle_front)
					{
						if(cmd_vel_error.twist.linear.x>0.0)
						{
							cmd_vel_error.twist.linear.x=0.0;
						}
					}
				//no turning, when error cloud at the back!
				if(obstacle_back)
					{
						if(cmd_vel_error.twist.linear.x<0.0)
						{
							cmd_vel_error.twist.linear.x=0.0;
						}
						
				
					}else
					{
				
						if(obstacle_front_left && !obstacle_front_right)
						{
							cmd_vel_error.twist.linear.x=0.0;
							//check if backwards is enough space
							if(!obstacle_back_right)
							{
							cmd_vel_error.twist.angular.z=-1.0;
							}
							if(!obstacle_back)
							{
							cmd_vel_error.twist.linear.x=-0.1;
							
							}
						}
						if(!obstacle_front_left && obstacle_front_right)
						{	
							cmd_vel_error.twist.linear.x=0.0;
							if(!obstacle_back_left)
							{
							
							cmd_vel_error.twist.angular.z=1.0;
							
							}
							if(!obstacle_back)
							{
							cmd_vel_error.twist.linear.x=-0.1;
							
							
							}
						}
						if(obstacle_front_left && obstacle_front_right)
						{
							cmd_vel_error.twist.angular.z=0.0;
							if(!obstacle_back)
							{
							cmd_vel_error.twist.angular.z=0.5;
							cmd_vel_error.twist.linear.x=-0.4;
							
							}
						}
					}
				
			}
			
		
	}

	void readBackCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{
		obstacle_back_left=false;
		obstacle_back_right=false;
		obstacle_back=false;
		//create a pcl point cloud first...
		 pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
		//check if obstacle was detected
		if(msg->width>error_points)
		{
			obstacle_back=true;
				//now there is some processing neccessary...convert msg to pcl point cloud		
				pcl::fromROSMsg(*msg, *input);
				for (int i=0;i<input->points.size();i++)
				{
					
					double y=input->points[i].y;
					if(y<0)
					{
					
						obstacle_back_right=true;
					}
					if(y>=0)
					{
						obstacle_back_left=true;
					}  
					
				}
		}
	}
	
	void publish_data(const ros::TimerEvent& e)
	{
		
		//change mode if obstacle was detected at the front....
		if(mode_changer_active)
			{
			//case when there is an obstacle....need to get out in the oposite direction..., need change to obstacle mode....
			if(obstacle_front || obstacle_back)
			{
				was_active=true;
				mode.data= OBSTACLE;
				mode.header.stamp=ros::Time::now();	
				mode_pub.publish(mode);
				
				
			}else
			{
				double seconds=ros::Time::now().toSec()-mode.header.stamp.toSec();
				std::cout<<ros::Time::now().toSec()<<" / "<<mode.header.stamp.toSec()<<" / "<<seconds<<std::endl;
				if(was_active && seconds>4.0)
				{
				mode.data= AUTOMATIC;
				mode.header.stamp=ros::Time::now();	
				
				}else
				{
					mode.data= OBSTACLE;
				}
				mode_pub.publish(mode);
			}
			
			}
		//ROS_INFO( "Publish the info..");
		std_msgs::Bool d;
		d.data=obstacle_front_left;
		front_left_pub.publish(d);
		d.data=obstacle_front_right;
		front_right_pub.publish(d);
		d.data=obstacle_back_left;
		back_left_pub.publish(d);
		d.data=obstacle_back_right;
		back_right_pub.publish(d);
		pub_cmd_vel.publish(cmd_vel_error);
		//reset vel value;
		if(! obstacle_front)
		{
		cmd_vel_error.twist.linear.x= 0.3;
		cmd_vel_error.twist.angular.z=0.0;
		}
				
	}
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

std::string front_str,back_str,back_left_str,back_right_str,front_left_str,front_right_str;
std::string cmd_vel_pub_str, mode_str;
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
		if(!n.hasParam("cmd_vel_pub")) ROS_WARN("Used default parameter for cmd_vel_pub");
		n.param<std::string>("cmd_vel_pub", cmd_vel_pub_str, "/cmd_vel"); 
		n.param<std::string>("mode_pub", mode_str, "/mode"); 
	  
	 LaserObstacle o(ep);	
	 n.param<bool>("mode_changer_active", o.mode_changer_active,true);  
	 ros::Subscriber sub_laser_front = n.subscribe(front_str,1, &LaserObstacle::readFrontCloud, &o);
	 ros::Subscriber sub_laser_back  = n.subscribe(back_str,1, &LaserObstacle::readBackCloud, &o);
	  
      //create publishers
      o.front_left_pub=n.advertise<std_msgs::Bool>(front_left_str.c_str(), 1);
      o.front_right_pub=n.advertise<std_msgs::Bool>(front_right_str.c_str(), 1);
      o.back_left_pub=n.advertise<std_msgs::Bool>(back_left_str.c_str(), 1);
      o.back_right_pub=n.advertise<std_msgs::Bool>(back_right_str.c_str(), 1);
      //publisher
	  o.pub_cmd_vel = n.advertise<geometry_msgs::TwistStamped>(cmd_vel_pub_str.c_str(),10);
	  o.mode_pub = n.advertise<msgs::IntStamped>(mode_str.c_str(),10);
	  //define the update rate by a seperate function...
		ros::Timer t;
		t = n.createTimer(ros::Duration(0.05), &LaserObstacle::publish_data,&o);
	
	ros::spin();
  

}
