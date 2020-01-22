#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "pcl/common/distances.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>


/*
 * writen by David Reiser 17.03.15
 * this node checks the back and the front laser together with the actual cmd velocity 
 * and intervent when this is not possible to move there. 
 * Its not dependend to one position of the laserscanners, just take into account to have
 * one laser at the front, and one at the back.
 * It just take action, when a obstacle gets detected in the defined hight of the coordinates.
 * for that it takes already transformed and filtered point clouds in cartesian coordinates
 * It will try to not allow a movement, that will harm the robot (more ore less ;) )
 * */
 

typedef pcl::PointCloud<pcl::PointXYZI> PCLCloud;

class LaserGoal
{
private:	
	//actual raw and downsampled point cloud
	pcl::PointCloud<pcl::PointXYZI> cloud_front,cloud_back;
	//check if laser data received
	tf::TransformBroadcaster br;
	tf::TransformListener li;
	//one saved point of actual obstacle
	pcl::PointXYZI obstacle_point;
	//variables for position and goals...
	geometry_msgs::TwistStamped cmd_vel,cmd_vel_new;
	
		
public:
// public variables
	double robot_width,zmin,zmax,front_distance_min,back_distance_min;
	ros::Publisher pub_vel;
	
	LaserGoal()
	{
	
	}
	
	~LaserGoal()
	{
	}
	//subscribers

	void check_vel(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		cmd_vel=*msg;		
	}
	
	
	void readBackCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//write msg direct to cloud_raw
		pcl::fromROSMsg(*msg,cloud_front);	
	}
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//write msg direct to cloud_raw
		pcl::fromROSMsg(*msg,cloud_back);	
		
	}
	
//check if there is an obstacle between actual position and next goalpoint
	void check_for_Obstacles(const ros::TimerEvent& e)
	{
		
		//check if clouds are empty first
		if(cloud_back.points.size()==0 && cloud_front.points.size()==0)
		{
			//when clouds are empty dont do anything and publish the cmd_vel direct to the output
			pub_vel.publish(cmd_vel);	
		}else
		{
			//first define the twist direction status:
			int status=0;//not driving status
			if (cmd_vel.twist.linear.x>0 && cmd_vel.twist.angular.z==0)
			{
				//driving forward
				status =1;
			}
			if (cmd_vel.twist.linear.x<0 && cmd_vel.twist.angular.z==0)
			{
				//driving backwards
				status=2;
			}
			if (cmd_vel.twist.angular.z>0)
			{
				//turning left
				status=3;
			}
			if (cmd_vel.twist.angular.z<0)
			{
				//turning right+
				status=4;
			}
			
		check_if_speed_is_correct(status);	
		}
			
	}
	
  void check_if_speed_is_correct(int status)
  {
	  cmd_vel_new=cmd_vel;
	switch (status)
	{
		case 1:
		{
			ROS_INFO("moving forward");
			//just need to check front laser for obstacles depending on the speed
			double distance_min=cmd_vel.twist.linear.x*2;
			
		}
		break;
		case 2:
		{
			ROS_INFO("moving backwards");
			//just need to check back laser
			double distance_min=abs(cmd_vel.twist.linear.x)*2;
			
		}
		break;
		case 3:
		{
			ROS_INFO("turning left");
			
		}
		break;
		case 4:
		{
			ROS_INFO("turning right");
			
		}
		break;
		
	}
	
	
	
	
	  pub_vel.publish(cmd_vel_new);	
  }
 
};


//-------------------------------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_pcd_node");
	
	std::string source_front, source_back,cmd_vel_str,publisher_str;
	LaserGoal l;
	
	ros::NodeHandle n("~");
	n.param<std::string>("source_front", source_front, "/scan_cloud_front");
	n.param<std::string>("source_back", source_back, "/scan_cloud_back");
	n.param<std::string>("cmd_vel_sub", cmd_vel_str, "/cmd_vel");
	n.param<std::string>("cmd_vel_pub", publisher_str, "/cmd_vel_new");
		
    n.param<double>("robot_radius", l.robot_width, 0.4);
    n.param<double>("zmin", l.zmin, -0.2);
    n.param<double>("zmax", l.zmax,  0.2);
    n.param<double>("front_distance_min", l.front_distance_min,  0.2);
    n.param<double>("back_distance_min", l.back_distance_min,  0.2);
    
	ros::Subscriber o1=n.subscribe(cmd_vel_str,1,&LaserGoal::check_vel,&l);
	ros::Subscriber o2=n.subscribe(source_front,1,&LaserGoal::readFrontCloud,&l);
	ros::Subscriber o3=n.subscribe(source_back,1,&LaserGoal::readBackCloud,&l);
	
	//publisher
	l.pub_vel = n.advertise<geometry_msgs::TwistStamped>(publisher_str.c_str(),10);
	//define Hz for checking the clouds...
	ros::Timer t;
	t = n.createTimer(ros::Duration(0.01), &LaserGoal::check_for_Obstacles,&l);
	
	
	ros::spin();			
	
}
