
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>


/* reading 3 laserscanners assemble the data to one pointcloud and apply a voxel grid 
 * with the size x, y,z
 * by David Reiser 12.03.15
 *  * */

typedef pcl::PointCloud<pcl::PointXYZI> PCLCloud;


class ASSEMBLER
{
public:
	
	boost::mutex            lock1_;
    boost::mutex            lock2_;
    boost::mutex            lock3_;
    
   
    tf::TransformListener *tf_listener1; 
     tf::TransformListener *tf_listener2; 
      tf::TransformListener *tf_listener3; 
	
	int neigbour_nr;
	double radius, distance,frequency;
	double x,y,z;
	bool use_voxel;
	std::string baseframe;
	
// public variables
	ros::Publisher cloud_out_pub1,cloud_out_pub2,cloud_out_pub3;
	//complete cloud
	pcl::PointCloud<pcl::PointXYZI> assembled_cloud1,assembled_cloud2,assembled_cloud3;
	
	ASSEMBLER()
	{
		//applied voxel grid values
		x=0.1;
		y=0.1;
		z=0.1;
		tf_listener1=new tf::TransformListener();
		tf_listener2=new tf::TransformListener();
		tf_listener3=new tf::TransformListener();
	}
	
	~ ASSEMBLER()
	{
	}
	
	void readCloud1(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{	
		//create a pcl point cloud first...and transform
		 pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
		 sensor_msgs::PointCloud2 transformed_cloud; 
		 tf_listener1->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(5.0));
		 pcl_ros::transformPointCloud(baseframe,*msg,transformed_cloud,*tf_listener1);
		 pcl::fromROSMsg(transformed_cloud,*input);
		 				
		assembled_cloud1=assembled_cloud1+*input;
		pcl::copyPointCloud(assembled_cloud1,*input);
		if(use_voxel)
		input=apply_voxel_filter(input,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input,assembled_cloud1);
		//publish_data(assembled_cloud1,baseframe,cloud_out_pub1);
		sensor_msgs::PointCloud2 cloud;
		//direcly write back the information to the point cloud msg
		pcl::toROSMsg(assembled_cloud1,cloud);
		//set the new header and size tranforms
		cloud.header.frame_id=baseframe;
		cloud.header.stamp=ros::Time::now();
		cloud.width=assembled_cloud1.points.size();
		cloud_out_pub1.publish(cloud);
		ROS_INFO("read_data cloud1 with %d points",assembled_cloud1.points.size());
		
	}
	void readCloud2(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//create a pcl point cloud first...
		 pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
	
		sensor_msgs::PointCloud2 transformed_cloud; 
		 tf_listener2->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(5.0));
		 pcl_ros::transformPointCloud(baseframe,*msg,transformed_cloud,*tf_listener2);
		 pcl::fromROSMsg(transformed_cloud,*input);
		
		assembled_cloud2=assembled_cloud2+*input;
		pcl::copyPointCloud(assembled_cloud2,*input);
		if(use_voxel)
		input=apply_voxel_filter(input,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input,assembled_cloud2);
		//publish_data(assembled_cloud2,baseframe,cloud_out_pub2);
		sensor_msgs::PointCloud2 cloud;
		//direcly write back the information to the point cloud msg
		pcl::toROSMsg(assembled_cloud2,cloud);
		//set the new header and size tranforms
		cloud.header.frame_id=baseframe;
		cloud.header.stamp=ros::Time::now();
		cloud.width=assembled_cloud2.points.size();
		cloud_out_pub2.publish(cloud);
		ROS_INFO("read_data cloud2 with %d points",assembled_cloud2.points.size());
		
		
	}
	
	void readCloud3(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//create a pcl point cloud first...
		 pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);
		 
		sensor_msgs::PointCloud2 transformed_cloud; 
		 tf_listener3->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(5.0));
		 pcl_ros::transformPointCloud(baseframe,*msg,transformed_cloud,*tf_listener3);
		 pcl::fromROSMsg(transformed_cloud,*input);
		 
		assembled_cloud3=assembled_cloud3+*input;
		pcl::copyPointCloud(assembled_cloud3,*input);
		if(use_voxel)
		input=apply_voxel_filter(input,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input,assembled_cloud3);
		//publish_data(assembled_cloud3,baseframe,cloud_out_pub3);
		sensor_msgs::PointCloud2 cloud;
		//direcly write back the information to the point cloud msg
		pcl::toROSMsg(assembled_cloud3,cloud);
		//set the new header and size tranforms
		cloud.header.frame_id=baseframe;
		cloud.header.stamp=ros::Time::now();
		cloud.width=assembled_cloud3.points.size();
		cloud_out_pub3.publish(cloud);
		ROS_INFO("read_data cloud3 with %d points",assembled_cloud3.points.size());
		
		
	}
		
	//-----------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZI>::Ptr apply_voxel_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr input,float x, float y,float z)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr output (new pcl::PointCloud<pcl::PointXYZI>);
		 
		 pcl::VoxelGrid<pcl::PointXYZI> vox_grid;

		if(input->points.size()>0)
		{
		vox_grid.setLeafSize (x,y,z);
		// ... and downsampling the point cloud
		//std::cout << "PointCloud before filtering: " << input->width * input->height << " data points." <<endl;
		vox_grid.setInputCloud (input);
		vox_grid.filter (*output);
		//std::cout << "PointCloud after filtering: " << output->width * output->height << " data points." << endl;
		}
								
		return output;
		
	}
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

 ASSEMBLER o;
std::string front_str,cloud_out1,cloud_out2,cloud_out3,cloud1,cloud2,cloud3;

	  n.param<std::string>("cloud_in_1", cloud1, "/front_cloud"); 
	  n.param<std::string>("cloud_in_2", cloud2, "/front_tilt_cloud"); 
	  n.param<std::string>("cloud_in_3", cloud3, "/back_cloud"); 
	  n.param<std::string>("cloud_out1", cloud_out1, "/cloud1");
	  n.param<std::string>("cloud_out2", cloud_out2, "/cloud2");
	  n.param<std::string>("cloud_out3", cloud_out3, "/cloud3");
	  n.param<std::string>("base_frame", o.baseframe, "odom");  
	  
	   n.param<double>("x", o.x, 5);
	   n.param<double>("y", o.y, 5);
	   n.param<double>("z", o.z, 10);
	   n.param<double>("frequency", o.frequency, 10);
	   n.param<bool>("use_voxel", o.use_voxel, false);
	  	  
	  ros::Subscriber sub_cloud1 = n.subscribe(cloud1,1, &ASSEMBLER::readCloud1, &o);
	  ros::Subscriber sub_cloud2 = n.subscribe(cloud2,1, &ASSEMBLER::readCloud2, &o);
	  ros::Subscriber sub_cloud3 = n.subscribe(cloud3,1, &ASSEMBLER::readCloud3, &o);
	
      //create publishers
      o.cloud_out_pub1=n.advertise<sensor_msgs::PointCloud2>(cloud_out1.c_str(), 1);
       o.cloud_out_pub2=n.advertise<sensor_msgs::PointCloud2>(cloud_out2.c_str(), 1);
        o.cloud_out_pub3=n.advertise<sensor_msgs::PointCloud2>(cloud_out3.c_str(), 1);
      
	
	ros::spin();
  

}
