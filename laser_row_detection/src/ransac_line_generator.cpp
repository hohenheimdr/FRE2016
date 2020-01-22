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
#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>


//RANSAC Filter zur Geradengenerierung und anschließender Pose Ausgabe 
//by Max Staiger & Markus Ullrich
//optimized by David Reiser for the fieldrobot Event 2016 
//date: 1.6.16

 

typedef pcl::PointCloud<pcl::PointXYZI> PCLCloud;


class PCL_CLASS
{
public:																				//Bestimmung der Variablen
	
	int iterations,mean_value,add_points;
	double distance, xmin,xmax,ymin,ymax;
	std::string frame_id,cloud_frame;

	geometry_msgs::PoseStamped row_pose;
	bool start_line_tracking,left_row;
	float row_pose_z[1000];

	ros::Publisher row_pose_pub;
	
	PCL_CLASS()
	{
		start_line_tracking=true; 
					
	}
	
	~ PCL_CLASS()
	{
	}
	
	double fabs(double in)
	{
		return sqrt(in*in);
	}
	
	float getMean (float input) 													//Bildung Mittelwert von Z um Schwankungen zu minimieren
	{
		float Mean=0;
		row_pose_z[mean_value]=input;
			for (int i=0; i<mean_value; i++)
			{
				row_pose_z[i]=row_pose_z[i+1];
				Mean=Mean+row_pose_z[i];
			}
		return float(Mean/mean_value);
	}
	
	void readFrontCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)				//Einlesen der PCL 
	{		
		row_pose.header.frame_id = msg->header.frame_id;
		row_pose.header.stamp = msg->header.stamp;
	
		pcl::PointCloud<pcl::PointXYZI>::Ptr input (new pcl::PointCloud<pcl::PointXYZI>);		//Erstellen der PointCloud
			
		if(msg->width>3 && start_line_tracking)
			{
			
			pcl::fromROSMsg(*msg, *input);
			pcl::ModelCoefficients line_params=apply_ransac(input);
			row_pose.pose.position.x=line_params.values[0];
			row_pose.pose.position.y=line_params.values[1];
			row_pose.pose.position.z=line_params.values[2];
			//Verwendung des Mittelwerts für die Winkel	
			double dx=line_params.values[3]-line_params.values[0];
			double dy=line_params.values[4]-line_params.values[1];
			double dz=line_params.values[5]-line_params.values[2];
			//as we work in 2D no roll and pitch neccessary..
			double r=getMean(0);
			double p=getMean(0);
			double y;
			y=line_params.values[4];//atan(dx/dy);//getMean(dx/dy);
			std::cout<<"yaw:"<<y<<" dx:"<<dx<<" dy:"<<dy<<" dz:"<<dz<<std::endl;
			/*
			if(dx>0)
			{			
				y=getMean(dx/-dy);
			}else
			{
				y=getMean(-dx/-dy);
			}*/
											
			geometry_msgs::Quaternion line_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,y);
			row_pose.pose.orientation=line_quat;
			row_pose_pub.publish(row_pose);
			}
		
		else  																	
		{
			row_pose.pose.position.x=1;
			row_pose.pose.position.y=0;
			row_pose.pose.position.z=0;
			row_pose.pose.orientation.x=0;
			row_pose.pose.orientation.y=0;
			row_pose.pose.orientation.z=0;
			row_pose.pose.orientation.w=0;
		
			row_pose.header.frame_id=msg->header.frame_id;
			row_pose.header.stamp=msg->header.stamp;
			row_pose_pub.publish(row_pose);							
		}			
		
	}
	
	pcl::ModelCoefficients apply_ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
	{	
		
		pcl::ModelCoefficients coefficients;
		if(cloud->points.size()>3)
		{
		try{
			std::vector<pcl::ModelCoefficients> result;
			pcl::SACSegmentation<pcl::PointXYZI> seg;									//Erstelle Segementierung Create the segmentation object
			
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());							
			seg.setOptimizeCoefficients (true);											//Einstellungen des RANSAC Algorithmus
			seg.setModelType (pcl::SACMODEL_LINE);
			//seg.setMethodType (pcl::SAC_RANSAC);
			//seg.setMethodType (pcl::SAC_LMEDS);
			//seg.setMethodType (pcl::SAC_MSAC);
			//seg.setMethodType (pcl::SAC_RRANSAC);
			//seg.setMethodType (pcl::SAC_RMSAC);
			seg.setMethodType (pcl::SAC_MLESAC);
			//seg.setMethodType (pcl::SAC_PROSAC);
			
			seg.setDistanceThreshold(distance);
			//define inline point!
			pcl::PointXYZI p;
			p.x=0;
			if(left_row){
				p.y=0.375;
			}else
			{	
				p.y=-0.375;
			}
			
			p.z=0;
			for(int i=0;i<add_points;i++)
			{
				cloud->points.push_back(p);
			}
			cloud->width=cloud->points.size();
			cloud->height=1;
			//inliers=cloud;
			seg.setMaxIterations (iterations);								//Verwende max. Anzahl von Iterationen für besseres Ergebnis	
			seg.setInputCloud(cloud);
			seg.segment(*inliers, coefficients);
			
			// cofficients are: [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z] 
			//ROS_INFO("y= %f + %f  x",coefficients.values[1],coefficients.values[4]);    //Ausgabe der Geradenfunktion im Terminal
			ROS_INFO("0:%2f,1:%2f,2:%2f,3:%2f,4:%2f,5:%2f",coefficients.values[0],coefficients.values[1],coefficients.values[2],coefficients.values[3],coefficients.values[4],coefficients.values[5]);
			//std::cout<<coefficients.size();
			
		}catch(...){
			ROS_INFO("error line generation");
		}
		}
		
		return coefficients;	
	}
};


int main(int argc, char** argv){

	ros::init(argc, argv, "error_cloud_cmd");

ros::NodeHandle n("~");

 PCL_CLASS o;
 
	std::string front_str,row_out,odometry_str;

	n.param<std::string>("cloud_in", front_str, "/error_cloud_front"); 								//Einstellen der einzelnen Parameter
	n.param<std::string>("row_out", row_out, "/error_cloud_front"); 
	n.param<std::string>("frame_id",o.frame_id,"odom");
	n.param<int>("mean_value", o.mean_value, 5);
	n.param<int>("iterations", o.iterations, 5);
	n.param<double>("distance", o.distance, 10);
	n.param<int>("add_points", o.add_points, 10);
	n.param<bool>("left_row", o.left_row, true);
			
	ros::Subscriber sub_laser_front = n.subscribe(front_str,50, &PCL_CLASS::readFrontCloud, &o);		//Einlesen der PointCloud 
 
    o.row_pose_pub=n.advertise<geometry_msgs::PoseStamped>(row_out.c_str(), 50);
     
	ros::spin();
  

}
