#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64MultiArray.h>
#include "nav_msgs/Odometry.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>


/* reading the front and back cloud and give back at what side there is 
 * space to turn clouds just get published when there are points in
 * defined range of laser_pcd_node
 * this signal has priority A when it  comes to adaptive obstacle
 * avoidance
 * by David Reiser 7.7.15
 *  * */

typedef pcl::PointCloud<pcl::PointXYZI> PCLCloud;


class PCL_CLASS
{
public:
	
	int neigbour_nr;
	double radius, distance, xmin,xmax,ymin,ymax;
	std::string frame_id,cloud_frame;
	visualization_msgs::Marker row;
	geometry_msgs::PoseStamped row_pose;
	bool start_line_tracking;
	
	// public variables
	ros::Publisher row_out_pub,row_pose_pub;
	
	PCL_CLASS()
	{
		start_line_tracking=false;
				row.pose.orientation.w = 1.0;
				row.type = visualization_msgs::Marker::LINE_STRIP;

				row.id = 1;

				row.color.b = 1.0;
				row.color.r = 0;
				row.color.a = 0.6;
	}
	
	~ PCL_CLASS()
	{
	}
	
	void readOdometry(const nav_msgs::Odometry::ConstPtr& msg)
	{
		
		if(msg->pose.pose.position.x>5.75 && msg->pose.pose.position.x<5.8)
		{
			//ROS_INFO("read");
			start_line_tracking=true;
			
		
		}else
		{
			start_line_tracking=false;
		}
	}	
	
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		
		row.header.frame_id = msg->header.frame_id;
		row.header.stamp = msg->header.stamp;
		row.ns = "Ransac";
		row.scale.x = 1;
		
		cloud_frame=msg->header.frame_id;
		//create a pcl point cloud first...
		 pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
		//check if obstacle was detected
		//ROS_INFO("read");
		pcl::ModelCoefficients line_params;
		geometry_msgs::PoseStamped line;
		
		//now there is some processing neccessary...push info to 	
		if(msg->width>3 && start_line_tracking)
		{
		pcl::fromROSMsg(*msg, *input);
		line_params=apply_ransac(input);
		row.pose.position.x=line_params.values[0];
		row.pose.position.y=line_params.values[1];
		row.pose.position.z=line_params.values[2];
		
		row.pose.orientation.x=line_params.values[3];
		row.pose.orientation.y=line_params.values[4];
		row.pose.orientation.z=line_params.values[5];
		
		row_pose.pose.position=row.pose.position;
		row_pose.pose.orientation=row.pose.orientation;
		row_pose.header.frame_id=msg->header.frame_id;
		row_pose.header.stamp=msg->header.stamp;
		
		geometry_msgs::Point p;
		row.points.clear();

				p.x = 0; p.y = row.pose.orientation.y*p.x + row.pose.position.y; p.z = 0;

				//row.points.push_back(p);

				p.x = 1; p.y = row.pose.orientation.y*p.x + row.pose.position.y;

				//row.points.push_back(p);
		row_out_pub.publish(row);
		row_pose_pub.publish(row_pose);
		
		}else
		{
			//set row to zero
		row.pose.position.x=0;
		row.pose.position.y=0;
		row.pose.position.z=0;
		
		row.pose.orientation.x=0;
		row.pose.orientation.y=0;
		row.pose.orientation.z=0;
		
		row_pose.pose.position=row.pose.position;
		row_pose.pose.orientation=row.pose.orientation;
		row_pose.header.frame_id=msg->header.frame_id;
		row_pose.header.stamp=msg->header.stamp;
		
			
		}
		//just publish if there is the range...
				
		
	}
	pcl::ModelCoefficients apply_ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{	
		std::vector<pcl::ModelCoefficients> result;
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZI> seg;
		pcl::ModelCoefficients coefficients;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	
		/*Settings of RANSAC algorithm */
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_LINE);
		seg.setMethodType (pcl::SAC_RANSAC);// also possible to use alternative LMEDS algorithm...
		seg.setDistanceThreshold(distance);
		//use max iterations in correlance to point cloud size
		seg.setMaxIterations (cloud->points.size());
		
		seg.setInputCloud(cloud);
		seg.segment(*inliers, coefficients);
		ROS_INFO("y= %f + %f  x",coefficients.values[1],coefficients.values[4]);
		
		return coefficients;
		
	}
	
	//-----------------------------------------------------------------------
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

 PCL_CLASS o;
std::string front_str,row_out,odometry_str;

	  n.param<std::string>("cloud_in", front_str, "/error_cloud_front"); 
	  n.param<std::string>("row_out", row_out, "/error_cloud_front"); 
	  n.param<std::string>("frame_id",o.frame_id,"odom");
	   n.param<double>("radius", o.radius, 5);
	   n.param<int>("neigbour_nr", o.neigbour_nr, 5);
	   n.param<double>("distance", o.distance, 10);
	   n.param<double>("xmin", o.xmin, 10);
	   n.param<double>("xmax", o.xmax, 10);
	     n.param<double>("ymin", o.ymin, 10);
	   n.param<double>("ymax", o.ymax, 10);
	   n.param<std::string>("odometry",odometry_str,"/fmInformation/odometry");
	  	  
	  ros::Subscriber sub_odom = n.subscribe(odometry_str,50, &PCL_CLASS::readOdometry, &o); 
	  ros::Subscriber sub_laser_front = n.subscribe(front_str,50, &PCL_CLASS::readFrontCloud, &o);
	 
	  std::string row_pose=row_out + "_pose";
      //create publishers
      o.row_out_pub=n.advertise<visualization_msgs::Marker>(row_out.c_str(), 50);
      o.row_pose_pub=n.advertise<geometry_msgs::PoseStamped>(row_pose.c_str(), 50);
      
	
	ros::spin();
  

}
