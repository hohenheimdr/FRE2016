#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


/* reading the front and back cloud and give back at what side there is 
 * space to turn clouds just get published when there are points in
 * defined range of laser_pcd_node
 * this signal has priority A when it  comes to adaptive obstacle
 * avoidance
 * by David Reiser 25.02.15
 *  * */

typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;


class PCL_CLASS
{
public:
	
	int neigbour_nr;
	double radius, distance;
	
// public variables
	ros::Publisher cloud_out_pub;
	
	PCL_CLASS()
	{
		
	}
	
	~ PCL_CLASS()
	{
	}
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//create a pcl point cloud first...
		 pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
		//check if obstacle was detected
		
		//now there is some processing neccessary...convert msg to pcl point cloud		
		pcl::fromROSMsg(*msg, *input);
		
		publish_data(apply_filter(input, neigbour_nr,radius,distance),msg->header.frame_id);
		
	}
	//-----------------------------------------------------------------------
	void publish_data(pcl::PointCloud<pcl::PointXYZ>::Ptr input,std::string frame_id)
	{
		sensor_msgs::PointCloud2 cloud;
		//direcly write back the information to the point cloud msg
		pcl::toROSMsg(*input,cloud);
		//set the new header and size tranforms
		cloud.header.frame_id=frame_id;
		cloud.header.stamp=ros::Time::now();
		cloud.width=input->points.size();
		cloud_out_pub.publish(cloud);
				
	}
	
	//-----------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr apply_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int nr, double radius,double distance)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		// build the filter
		outrem.setInputCloud(cloud);
		outrem.setRadiusSearch(radius);
		outrem.setMinNeighborsInRadius (nr);
		// apply filter
		outrem.filter (*cloud_filtered);
  
		  std::cerr << "Cloud before filtering: " << std::endl;
		  for (size_t i = 0; i < cloud->points.size (); ++i)
			std::cerr << "    " << cloud->points[i].x << " "
								<< cloud->points[i].y << " "
								<< cloud->points[i].z << std::endl;
		  // display pointcloud after filtering
		  std::cerr << "Cloud after filtering: " << std::endl;
		  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
			std::cerr << "    " << cloud_filtered->points[i].x << " "
								<< cloud_filtered->points[i].y << " "
								<< cloud_filtered->points[i].z << std::endl;
								
		return cloud_filtered;
		
		
	}
	
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

 PCL_CLASS o;
std::string front_str,cloud_out;

	  n.param<std::string>("cloud_in", front_str, "/error_cloud_front"); 
	  n.param<std::string>("cloud_out", cloud_out, "/error_cloud_front"); 
	  
	   n.param<double>("radius", o.radius, 5);
	   n.param<int>("neigbour_nr", o.neigbour_nr, 5);
	   n.param<double>("distance", o.distance, 10);
	  	  
	  ros::Subscriber sub_laser_front = n.subscribe(front_str,1, &PCL_CLASS::readFrontCloud, &o);
	
	  
      //create publishers
      o.cloud_out_pub=n.advertise<sensor_msgs::PointCloud2>(cloud_out.c_str(), 1);
      
	
	ros::spin();
  

}
