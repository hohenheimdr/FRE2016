
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include <msgs/FloatStamped.h>
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
       tf::TransformListener *tf_listener4; 
        tf::TransformListener *tf_listener5; 
         tf::TransformListener *tf_listener6; 
	
	int neigbour_nr;
	double radius, distance,frequency;
	double x,y,z;
	std::string baseframe;
	
// public variables
	ros::Publisher cloud_out_pub1;
	//complete cloud
	pcl::PointCloud<pcl::PointXYZI> assembled_cloud1;
	
	ASSEMBLER()
	{
		//applied voxel grid values
		x=0.1;
		y=0.1;
		z=0.1;
		tf_listener1=new tf::TransformListener();
		tf_listener2=new tf::TransformListener();
		tf_listener3=new tf::TransformListener();
		tf_listener4=new tf::TransformListener();
		tf_listener5=new tf::TransformListener();
		tf_listener6=new tf::TransformListener();
	}
	
	~ ASSEMBLER()
	{
	}
	
	void readSensor1(const msgs::FloatStamped::ConstPtr& msg)
	{	
		//create point
		pcl::PointXYZI input;
		//set values to zero.... first...
		input.x=0;
		input.y=0;
		input.z=0;
		input.intensity=msg->data;
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_point (new pcl::PointCloud<pcl::PointXYZI>);
		input_point->push_back(input);
		assembled_cloud1.push_back(input);
		//check if I done this right.... later!
		assembled_cloud1.width=assembled_cloud1.points.size();
		//input_point->width=1;
		//tf_listener1->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(1.0));
		//pcl_ros::transformPointCloud(baseframe,*input_point,*input_point,*tf_listener1);	
		assembled_cloud1=assembled_cloud1+*input_point;
		pcl::copyPointCloud(assembled_cloud1,*input_point);
		input_point=apply_voxel_filter(input_point,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input_point,assembled_cloud1);
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
	void readSensor2(const msgs::FloatStamped::ConstPtr& msg)
	{	
		//create point
		pcl::PointXYZI input;
		//set values to zero.... first...
		input.x=0;
		input.y=0;
		input.z=0;
		input.intensity=msg->data;
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_point (new pcl::PointCloud<pcl::PointXYZI>);
		input_point->push_back(input);
		//check if I done this right.... later!
		input_point->width=1;
		//tf_listener1->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(1.0));
		//pcl_ros::transformPointCloud(baseframe,*input_point,*input_point,*tf_listener1);	
		assembled_cloud1=assembled_cloud1+*input_point;
		pcl::copyPointCloud(assembled_cloud1,*input_point);
		input_point=apply_voxel_filter(input_point,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input_point,assembled_cloud1);
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
	void readSensor3(const msgs::FloatStamped::ConstPtr& msg)
	{	
		//create point
		pcl::PointXYZI input;
		//set values to zero.... first...
		input.x=0;
		input.y=0;
		input.z=0;
		input.intensity=msg->data;
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_point (new pcl::PointCloud<pcl::PointXYZI>);
		input_point->push_back(input);
		//check if I done this right.... later!
		input_point->width=1;
		//tf_listener1->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(1.0));
		//pcl_ros::transformPointCloud(baseframe,*input_point,*input_point,*tf_listener1);	
		assembled_cloud1=assembled_cloud1+*input_point;
		pcl::copyPointCloud(assembled_cloud1,*input_point);
		input_point=apply_voxel_filter(input_point,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input_point,assembled_cloud1);
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
	void readSensor4(const msgs::FloatStamped::ConstPtr& msg)
	{	
		//create point
		pcl::PointXYZI input;
		//set values to zero.... first...
		input.x=0;
		input.y=0;
		input.z=0;
		input.intensity=msg->data;
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_point (new pcl::PointCloud<pcl::PointXYZI>);
		input_point->push_back(input);
		//check if I done this right.... later!
		input_point->width=1;
		//tf_listener1->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(1.0));
		//pcl_ros::transformPointCloud(baseframe,*input_point,*input_point,*tf_listener1);	
		assembled_cloud1=assembled_cloud1+*input_point;
		pcl::copyPointCloud(assembled_cloud1,*input_point);
		input_point=apply_voxel_filter(input_point,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input_point,assembled_cloud1);
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
	void readSensor5(const msgs::FloatStamped::ConstPtr& msg)
	{	
		//create point
		pcl::PointXYZI input;
		//set values to zero.... first...
		input.x=0;
		input.y=0;
		input.z=0;
		input.intensity=msg->data;
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_point (new pcl::PointCloud<pcl::PointXYZI>);
		input_point->push_back(input);
		//check if I done this right.... later!
		input_point->width=1;
		//tf_listener1->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(1.0));
		//pcl_ros::transformPointCloud(baseframe,*input_point,*input_point,*tf_listener1);	
		assembled_cloud1=assembled_cloud1+*input_point;
		pcl::copyPointCloud(assembled_cloud1,*input_point);
		input_point=apply_voxel_filter(input_point,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input_point,assembled_cloud1);
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
	void readSensor6(const msgs::FloatStamped::ConstPtr& msg)
	{	
		//create point
		pcl::PointXYZI input;
		//set values to zero.... first...
		input.x=0;
		input.y=0;
		input.z=0;
		input.intensity=msg->data;
		pcl::PointCloud<pcl::PointXYZI>::Ptr input_point (new pcl::PointCloud<pcl::PointXYZI>);
		input_point->push_back(input);
		//check if I done this right.... later!
		input_point->width=input_point->points.size();
		//input_point->size=1;
		//input_point->size()=1;
		//tf_listener1->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(1.0));
		//pcl_ros::transformPointCloud(baseframe,*input_point,*input_point,*tf_listener1);	
		assembled_cloud1=assembled_cloud1+*input_point;
		pcl::copyPointCloud(assembled_cloud1,*input_point);
		input_point=apply_voxel_filter(input_point,x,y,z);
		//copy pointcloud to 
		pcl::copyPointCloud(*input_point,assembled_cloud1);
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
std::string cloud_out1,sensor1,sensor2,sensor3,sensor4,sensor5,sensor6;

	  n.param<std::string>("cloud_out1", cloud_out1, "/cloud1");
	  n.param<std::string>("base_frame", o.baseframe, "odom"); 
	  n.param<std::string>("sensor1", sensor1, "/sensor1"); 
	  n.param<std::string>("sensor2", sensor2, "/sensor2"); 
	  n.param<std::string>("sensor3", sensor3, "/sensor3"); 
	  n.param<std::string>("sensor4", sensor4, "/sensor4"); 
	  n.param<std::string>("sensor5", sensor5, "/sensor5"); 
	  n.param<std::string>("sensor6", sensor6, "/sensor6"); 
	  
	   n.param<double>("x", o.x, 5);
	   n.param<double>("y", o.y, 5);
	   n.param<double>("z", o.z, 10);
	   n.param<double>("frequency", o.frequency, 10);
	  	  
	  ros::Subscriber sub_sensor1 = n.subscribe(sensor1,1, &ASSEMBLER::readSensor1, &o);
	   ros::Subscriber sub_sensor2 = n.subscribe(sensor2,1, &ASSEMBLER::readSensor2, &o);
	    ros::Subscriber sub_sensor3 = n.subscribe(sensor3,1, &ASSEMBLER::readSensor3, &o);
	     ros::Subscriber sub_sensor4 = n.subscribe(sensor4,1, &ASSEMBLER::readSensor4, &o);
	      ros::Subscriber sub_sensor5 = n.subscribe(sensor5,1, &ASSEMBLER::readSensor5, &o);
	       ros::Subscriber sub_sensor6 = n.subscribe(sensor6,1, &ASSEMBLER::readSensor6, &o);

	 
	
      //create publishers
      o.cloud_out_pub1=n.advertise<sensor_msgs::PointCloud2>(cloud_out1.c_str(), 1);
      
      
	
	ros::spin();
  

}
