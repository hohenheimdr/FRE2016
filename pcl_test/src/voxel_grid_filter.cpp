
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
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>


/* reading data from kinect and apply a fast voxel grid to it
 * with the size x, y,z
 * by David Reiser 12.03.15
 *  * */

typedef pcl::PointCloud<pcl::PointXYZRGB> PCLCloud;

class ASSEMBLER
{
public:
	
	boost::mutex            lock1_;
    boost::mutex            lock2_;
    boost::mutex            lock3_;
    
   
    tf::TransformListener *tf_listener1; 
    tf::TransformListener *tf_listener2; 
   
	sensor_msgs::PointCloud2 cloud,cloud2;
	
	pcl::PointCloud<pcl::PointXYZRGB> assembled_cloud;
	
	int neigbour_nr;
	double radius, distance,frequency,sigma,height_max;
	double x,y,z;
	std::string baseframe, inputframe;
	bool use_voxel,assemble,use_ground_plane_filter, push_to_plane;
	
// public variables
	ros::Publisher cloud_out_pub1,cloud_out_pub2;
	
	
	ASSEMBLER()
	{
		//applied voxel grid values
		x=0.1;
		y=0.1;
		z=0.1;
		tf_listener1=new tf::TransformListener();
		tf_listener2=new tf::TransformListener();

	}
	
	~ ASSEMBLER()
	{
	}
	
	void readCloud1(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{	
		
		try{
		
		//create a pcl point cloud first...and transform
		 pcl::PointCloud<pcl::PointXYZRGB>::Ptr input (new pcl::PointCloud<pcl::PointXYZRGB>);
		 pcl::fromROSMsg(*msg,*input);				
		if(input->points.size()>0)
		{
			if(use_voxel)
			{
				input=apply_voxel_filter(input,x,y,z);
			}
			if (use_ground_plane_filter)
			{
				input=apply_ground_filter(input,sigma);
			}		
		//direcly write back the information to the point cloud msg
		
		
		
		try{
			//convert to ros clod...
               pcl::toROSMsg(*input,cloud);
			//transform to base
              tf_listener1->waitForTransform(baseframe,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(0.1));
			  pcl_ros::transformPointCloud(baseframe,cloud,cloud,*tf_listener1);
			  //convert to pcl cloud...
              pcl::fromROSMsg(cloud,*input);
					
				  if (push_to_plane)
				  {   
					  //push all points to the 2D level (when not allready correct...)
					  for (size_t i = 0; i < input->points.size(); i++) 
						{			
							input->points[i].z =  0.0;
						}		
				  }
				  
				  //convert to output...
				  pcl::toROSMsg(*input,cloud);
                  
                  if (assemble)
					{				
						assembled_cloud=assembled_cloud+*input;
						pcl::copyPointCloud(assembled_cloud,*input);
						input=apply_voxel_filter(input,x,y,z);
						pcl::copyPointCloud(*input,assembled_cloud);	
					}
					
             
                                
			}
			catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();	
		   }
		
		}//end if point cloud is valid...
		
		}//end try
		catch(tf::TransformException ex)
			{
			ROS_INFO("error ...maybe with tf? ");							
			}
		
		
	}
			
	//-----------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr apply_voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,float x, float y,float z)
	{
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;

		if(input->points.size()>0)
		{
			vox_grid.setLeafSize (x,y,z);
			// ... and downsampling the point cloud
			//std::cout << "PointCloud before filtering: " << input->width * input->height << " data points." <<std::endl;
			vox_grid.setInputCloud (input);
			vox_grid.filter (*output);
			//std::cout << "PointCloud after filtering: " << output->width * output->height << " data points." << std::endl;
		}
								
		return output;
		
	}
	
		//-----------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr apply_ground_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,double s)
	{
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;

		if(input->points.size()>0)
		{
			
			
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZRGB> seg;
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setDistanceThreshold (sigma);
			seg.setMaxIterations (100);
			seg.setInputCloud (input);
			seg.segment (*inliers, *coefficients);
		
			//pcl::copyPointCloud<pcl::PointXYZRGB>( *input, inliers->indices, *ground ); 
				
				// Extract fInliers from the input cloud
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                extract.setInputCloud (input);
                extract.setIndices (inliers);
                //extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud
                extract.setNegative (true); // Removes part_of_cloud from full cloud  and keep the rest
                extract.filter (*output); 
                
				
				//push all points to 2D in normal direction with the extracted plain parameters
					for (size_t i = 0; i < output->points.size(); i++) 
					{
						
						Eigen::Vector4f plane_coef(coefficients->values[0],coefficients->values[1],coefficients->values[2],1);
						double distance = pcl::pointToPlaneDistance(output->points[i],plane_coef);
						//ROS_INFO("%f",distance);
						if(abs_d(distance)<height_max)
						{
							if (push_to_plane)
							{	
							output->points[i].x = output->points[i].x+abs_d(coefficients->values[0]*distance);
							output->points[i].y = output->points[i].y+abs_d(coefficients->values[1]*distance);
							output->points[i].z =  output->points[i].z+abs_d(coefficients->values[2]*distance);
							}
							
						}else
						{
						output->points[i].x = 0;
						output->points[i].y = 0;
						output->points[i].z = 0;
						}
					}	
					
					//do a second time the voxel grid to remove double points
					
					output=apply_voxel_filter(output,x,y,z);		
		}
								
		return output;
		
	}
	
	double abs_d(double t)
	{
		return sqrt(t*t);
	}	
	
	
	void match_clouds(const ros::TimerEvent& e)
	{
		
		cloud_out_pub1.publish(cloud);
		
		if (assemble)
		{
			pcl::toROSMsg(assembled_cloud,cloud2);
			//set the new header and size tranforms
			cloud2.header.frame_id=baseframe;
			cloud2.header.stamp=cloud.header.stamp;
			cloud2.width=assembled_cloud.points.size();
			cloud_out_pub2.publish(cloud2);	
		}
		
	}
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

 ASSEMBLER o;
std::string front_str,cloud_out1,cloud_out2,cloud_out3,cloud1,cloud2,cloud3;

	  n.param<std::string>("cloud_in_1", cloud1, "/front_cloud");
	   n.param<std::string>("input_frame_id", o.inputframe, "odom");   
	  n.param<std::string>("cloud_out1", cloud_out1, "/cloud1");
	  n.param<std::string>("base_frame", o.baseframe, "odom"); 
	  n.param<bool>("use_voxel_filter", o.use_voxel, true);
	  n.param<bool>("assemble", o.assemble, true);    
	  n.param<bool>("use_ground_plane_filter", o.use_ground_plane_filter, true);
	  n.param<bool>("push_to_plane", o.push_to_plane, true);      
	 
	   n.param<double>("sigma", o.sigma, 0.1);
	    n.param<double>("height_max", o.height_max, 1.0);
	   
	   n.param<double>("x", o.x, 5);
	   n.param<double>("y", o.y, 5);
	   n.param<double>("z", o.z, 10);
	   n.param<double>("frequency", o.frequency, 10);
	  	  
	  ros::Subscriber sub_cloud1 = n.subscribe(cloud1,1, &ASSEMBLER::readCloud1, &o);
	 	
      //create publishers
      o.cloud_out_pub1=n.advertise<sensor_msgs::PointCloud2>(cloud_out1.c_str(), 1);
      o.cloud_out_pub2=n.advertise<sensor_msgs::PointCloud2>("kinect_test", 1);     
     
	
		ros::Timer t;
		t = n.createTimer(ros::Duration(double(1.0/o.frequency)),&ASSEMBLER::match_clouds,&o);
	
	ros::spin();
  

}
