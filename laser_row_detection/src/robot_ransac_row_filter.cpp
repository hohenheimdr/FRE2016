#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

//by Max Staiger & Markus Ullrich

typedef pcl::PointCloud<pcl::PointXYZI> PCLCloud;


class PCL_CLASS
{
public:
	
	int neigbour_nr;
	double radius, distance, xmin,xmax,ymin,ymax;
	std::string frame_id,cloud_frame;
	ros::Time t;
	ros::Publisher cloud_out_pub;
	
	PCL_CLASS()
	{
	}
		
	~ PCL_CLASS()
	{
	}
	
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		cloud_frame=msg->header.frame_id;
		t=msg->header.stamp;
	
		pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);		//Erstellung PointCloud	
		pcl::fromROSMsg(*msg, *input);
		input=apply_filter(input,xmin,xmax,ymin,ymax);
		publish_data(input);
	}
//-------------------------------------------------------------------------------------------
	void publish_data(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
	{
		sensor_msgs::PointCloud2 cloud;
		//direcly write back the information to the point cloud msg
		pcl::toROSMsg(*input,cloud);
		//set the new header and size tranforms
		cloud.header.frame_id=frame_id;
		//set old time stamp to avoid errors
		cloud.header.stamp=t;
		cloud.width=input->points.size();
		cloud_out_pub.publish(cloud);
				
	}
	pcl::PointCloud<pcl::PointXYZI>::Ptr apply_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,double xmin,double xmax, double ymin, double ymax)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);		//transform it to the baseframe 
	
			  for (int i=0; i<cloud->points.size();i++)									//Kontrollier ob Punkte im definierten Bereich sind
			  {
				  if(cloud->points[i].x>xmin && cloud->points[i].x<xmax)
				  {
					  if(cloud->points[i].y>ymin && cloud->points[i].y<ymax)
						{
							cloud_filtered->push_back(cloud->points[i]);	
						}
				  }
			  }
			  cloud_filtered->width=cloud_filtered->points.size();
					
		return cloud_filtered;
		
		
	}
	
};

int main(int argc, char** argv){

ros::init(argc, argv, "error_laser_cmd");

ros::NodeHandle n("~");

 PCL_CLASS o;
std::string front_str,cloud_out;

	n.param<std::string>("cloud_in", front_str, "/tilt_scan_cloud_filtered"); 
	n.param<std::string>("cloud_out", cloud_out, "/tilt_scan_cloud_filtered_new"); 
	n.param<std::string>("frame_id",o.frame_id,"base_link");
	n.param<double>("radius", o.radius, 200);							//Parameter des RANSACs, werden über Launch gefüllt
	n.param<int>("neigbour_nr", o.neigbour_nr, 1);						
	n.param<double>("distance", o.distance, 100);						
	n.param<double>("xmin", o.xmin, 0);    
	n.param<double>("xmax", o.xmax, 10);
	n.param<double>("ymin", o.ymin, -10);	
	n.param<double>("ymax", o.ymax, 0);		
	  
	ros::Subscriber sub_laser_front = n.subscribe(front_str,50, &PCL_CLASS::readFrontCloud, &o); //liest PointCloud ein
	
	o.cloud_out_pub=n.advertise<sensor_msgs::PointCloud2>(cloud_out.c_str(), 50); //publiziert Sensor Message in Topic
      
	ros::spin();
  

}
