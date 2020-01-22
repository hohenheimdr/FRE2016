#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
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
 * this node is responsible, that out of the two laserscanners a reliable 
 * obastacle avoidance is possible.
 * 
 * also the planed direction (in robot coordinates is given. When this is not possible
 * a other way is suggested*/
 
 //output pose....
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

void operator >> (geometry_msgs::Point point, tf::Vector3& vec){
	vec[0] = point.x;
	vec[1] = point.y;
	vec[2] = point.z;
}

class LaserGoal
{
private:	
	//actual raw and downsampled point cloud
	pcl::PointCloud<pcl::PointXYZ> cloud,cloud_raw;
	//check if laser data received
	bool laser_data;
	tf::TransformBroadcaster br;
	tf::TransformListener li;
	geometry_msgs::TransformStamped goal_trans;
	//one saved point of actual obstacle
	pcl::PointXYZ obstacle_point;
	//variables for position and goals...
	nav_msgs::Odometry odometry;
	geometry_msgs::PoseStamped actual_goal,last_goal, received_goal;
	
		
public:
// public variables
	double robot_width;
	ros::Publisher pub_goal;
	std::string laserframe,rabbitframe;
	
	LaserGoal()
	{
	laser_data=false;
	//define first the size of the robot... later move that param to launch file...
	robot_width=0.4;		
	}
	
	~LaserGoal()
	{
	}
	//subscribers
	void check_odometry(const nav_msgs::Odometry::ConstPtr& msg)
	{
		odometry=*msg;
	}
	
	void check_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		actual_goal.header.frame_id=msg->header.frame_id;
		//goal received from mission planer
		received_goal=*msg;
		if (actual_goal.pose.position.x==last_goal.pose.position.x)
		{actual_goal=*msg;}
		//if (laser_data)
		//{
			//check for obstacles in path and change if neccessary the goalpoint
			if(check_for_Obstacles())
			{
				
				
				
				//check if distance below thresh....
				if(check_min_distance(cloud_raw,0.25))
				{
					ROS_INFO("stop!");
					//set goal to base => dont move...
					actual_goal.header.frame_id="base_link";
					actual_goal.pose.position.x=0;
					actual_goal.pose.position.y=0;
					
				}else
				{
				//case a obstacle was detected, change goal...
				set_new_goal();				
				}
			}
		//}
		//write info back for other use	
		pub_goal.publish(actual_goal);

	  // publish the transform message to follow new goalpoint
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(actual_goal.pose.orientation.z);
			 goal_trans.header.stamp = ros::Time::now();
			 goal_trans.header.frame_id = actual_goal.header.frame_id;
			 goal_trans.child_frame_id =  rabbitframe;
			 goal_trans.transform.translation.x = actual_goal.pose.position.x;
			 goal_trans.transform.translation.y = actual_goal.pose.position.y;
			 goal_trans.transform.translation.z = 0.0;
			 goal_trans.transform.rotation = odom_quat;
			 br.sendTransform(goal_trans);
	//overwrite last goal point
	last_goal=actual_goal;	
	}
	//give back true, if distance smaller than thresh
	bool check_min_distance(pcl::PointCloud<pcl::PointXYZ> c,double thresh)
	{
		
		int counter=0;
		double thresh_point=thresh;
		
		while (thresh_point>=thresh && counter<c.points.size())
		{
			thresh_point=sqrt(c.points[counter].x*c.points[counter].x+c.points[counter].y*c.points[counter].y);
			counter++;
		}
		
		if (counter==c.points.size())
		{
			return false;
		}else
		{
			ROS_INFO("dist_min=%f",thresh_point);
		return true;
		}
	}
	
	void check_laser_data(const std_msgs::Bool::ConstPtr& msg)
	{
		laser_data=msg->data;		
	}
	
	void readCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		pcl::fromROSMsg(*msg,cloud_raw);
			
	}
//check if there is an obstacle between actual position and next goalpoint
	bool check_for_Obstacles()
	{
		//listen to transform of laser
		tf::StampedTransform laser_transform;
		try{
			//check for transform between fixed frame and laser
		li.lookupTransform(actual_goal.header.frame_id,laserframe,ros::Time(0),laser_transform);
		}catch(tf::TransformException ex)
		{
			ROS_INFO("error with transform listener");
		}
		pcl::copyPointCloud(cloud_raw,cloud);
		
		//first do a voxel grid filter with robot_width/2 =>reduce data 
		// Create the filtering object: downsample the dataset using a leaf size of 1cm
		/*
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(cloud_raw,*cloud_in);
		vg.setInputCloud (cloud_in);
		//do voxel grid with robot_with/4 can be done optional also in 3D
		vg.setLeafSize (0.1,0.1, 0.1);
		vg.filter (cloud);*/
		
		
		//transform cloud with actual laser position transform
		Eigen::Affine3f trafo=pcl::getTransformation (laser_transform.getOrigin().x(),laser_transform.getOrigin().y(),0,0,0,getYaw(laser_transform.getRotation()));
		pcl::transformPointCloud(cloud,cloud,trafo);		
		//compute line variables A=Robot Position, B = goal and n=normal of line between the two points
		Eigen::Vector4f A (odometry.pose.pose.position.x,odometry.pose.pose.position.y,0, 0);
		Eigen::Vector4f B (actual_goal.pose.position.x,actual_goal.pose.position.y,0, 0);
		//compute normal
		Eigen::Vector4f n =A-B;		
		//apply filter with the given area to check if obstacles in the way
		//return true if obstacle, else give back false
		return check_free_space(A,B,n,robot_width/2,cloud);	
	}
	
	//check if points laying between A and B in the pointcloud in a defined range!
	bool check_free_space(Eigen::Vector4f &A, Eigen::Vector4f &B, Eigen::Vector4f &normal, double thresh,pcl::PointCloud<pcl::PointXYZ> &c)
	{
		for (int i=0; i<c.points.size();i++)
		{
			//check if  srt distance to line is smaller than thresh, then 
			double dist;
			Eigen::Vector4f p(c.points[i].x,c.points[i].y,c.points[i].z,0);
			dist=pcl::sqrPointToLineDistance(p,A,normal);
			if(dist<=thresh)
			{
				double AB=pcl::euclideanDistance(pcl::PointXYZ(A[0],A[1],0),pcl::PointXYZ(B[0],B[1],0))+thresh;
				double AC=pcl::euclideanDistance(pcl::PointXYZ(A[0],A[1],0),c.points[i]);
				double BC=pcl::euclideanDistance(pcl::PointXYZ(B[0],B[1],0),c.points[i]);
				//check if point realy is between A and B
				if(AC<AB && BC<AB)
				{
				//found one point of the obstacle...
				return true;
				}
			}
		}
		return false;
	}
	
	bool check_for_inliners(double xmin,double xmax,double ymin,double ymax,pcl::PointCloud<pcl::PointXYZ> c,int count)
	{
		int point_nr=0;
		

	//check if there are points in the defined range	
			  for (int i=0; i<c.points.size();i++)
			  {
				  if(c.points[i].x>xmin && c.points[i].x<xmax)
				  {
					  if(c.points[i].y>ymin && c.points[i].y<ymax)
						{
							
							point_nr++;
							if(count==point_nr)
							{
								obstacle_point=c.points[i];
								return true;
							}
							
						}
				  }
			  }
		 obstacle_point.x=actual_goal.pose.position.x;
		 obstacle_point.y=actual_goal.pose.position.y;
		 return false;
		 
	}
	//search for a free space near the point cloud
	void set_new_goal()
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(cloud,*cloud_in);
		
		//do euclidean segmentation of the obstacle to gain all points belonging to the obstacle
		// kd-tree object for searches.
		kdtree->setInputCloud(cloud_in);
		// Euclidean clustering object.
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
		// Set cluster tolerance to robot_width to get the whole object with no passing
		// in several clusters, whereas big values may join objects in a same cluster
		clustering.setClusterTolerance(robot_width);
		// Set the minimum and maximum number of points that a cluster can have.
		clustering.setMinClusterSize(3);
		clustering.setMaxClusterSize(300);
		clustering.setSearchMethod(kdtree);
		clustering.setInputCloud(cloud_in);
		std::vector<pcl::PointIndices> clusters;
		clustering.extract(clusters);
		// For every cluster check if actual obstacle point is part of what cluster!
		int currentClusterNum = 1;
		for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
		{
			bool found_obstacle=false;
			// ...add all its points to a new cloud...
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			{
				if(cloud_in->points[*point].x==obstacle_point.x && cloud_in->points[*point].y==obstacle_point.y)
				{
					//ROS_INFO("obstacle cluster is Number %d",currentClusterNum);
					found_obstacle=true;
				}
				cluster->points.push_back(cloud_in->points[*point]);
			}
			cluster->width = cluster->points.size();
			cluster->height = 1;
			cluster->is_dense = true;
				if(found_obstacle)
				{
				pcl::copyPointCloud(*cluster,*cloud_obstacle);
				break;
				//break actual loop => dont need to check other obstacles at the moment
				}
			currentClusterNum++;
		}
	  
		
		//now check the next point to get around the obstacle. A= actual position
		//B=goal and next_point is the middle goal
		pcl::PointXYZ next_point;
		pcl::PointXYZ A(odometry.pose.pose.position.x,odometry.pose.pose.position.y,0);
		pcl::PointXYZ B(received_goal.pose.position.x, received_goal.pose.position.y,0);
		double path_length=check_shortest_path(A,B, next_point,cloud_obstacle);
		ROS_INFO("path length= %f",path_length);
		
		//check first if goal is reachable...
		if(check_if_goal_in_cloud(cloud_in,received_goal))
		{
			ROS_INFO("goal unreachable!");
			//do some error handling later...
			//for now just get next waypoint... this works if goal is get set to actual pose :)
			//time delay maybee later?
			//actual_goal.pose.position.x=odometry.pose.pose.position.x;
			//actual_goal.pose.position.y=odometry.pose.pose.position.y;
			//last_goal.pose.position.x=odometry.pose.pose.position.x;
			//last_goal.pose.position.y=odometry.pose.pose.position.y;
			//received_goal.pose.position.x=odometry.pose.pose.position.x;
			//received_goal.pose.position.y=odometry.pose.pose.position.y;	
		}else
		{
			//set new goal... now!
			actual_goal.pose.position.x=next_point.x;
			actual_goal.pose.position.y=next_point.y;
		}
		
	}
	
bool check_if_goal_in_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr ob, geometry_msgs::PoseStamped goal)
{
	pcl::PointXYZ g(goal.pose.position.x,goal.pose.position.y,0);
	for(int i=0;i<ob->points.size();i++)
	{
		//solve eucledian distance and check if sufficient
		double dist=pcl::euclideanDistance(ob->points[i],g);
		if(dist<robot_width)
		{
			return true;
		}
	}
	return false;
}
//search for the shortest path around a obstacle... recusive function...
 double check_shortest_path(pcl::PointXYZ A,pcl::PointXYZ B, pcl::PointXYZ &C,pcl::PointCloud<pcl::PointXYZ>::Ptr ob)
 {
	 //get nearest point to obstacle first
	 pcl::PointXYZ near_point=get_nearest_point(ob,A);
	 //double obstacle_distance=pcl::euclideanDistance(A,near_point)
		//read laser transform
		tf::StampedTransform laser_transform;
		try{
		li.lookupTransform(actual_goal.header.frame_id,laserframe,ros::Time(0),laser_transform);
		}
		catch(tf::TransformException ex)
		{
			ROS_INFO("error with transform listener");
		}
	//get center of Object:
	Eigen::Vector4f Center;
	pcl::compute3DCentroid(*ob,Center);
	//calculate distance between laser and obstacle center
	pcl::PointXYZ P(laser_transform.getOrigin().x(), laser_transform.getOrigin().y(),0);
	double dist=pcl::euclideanDistance(P,pcl::PointXYZ(Center[0],Center[1],0));
	
	pcl::PointXYZ normal((P.x-A.x),(P.y-A.y),0);
	C.x=(P.x+normal.x);
	C.y=(P.y+normal.y);
	//C=P;
   /*
	//get center of Object:
	Eigen::Vector4f Center;
	pcl::compute3DCentroid(*ob,Center);
	pcl::PointXYZ Min,Max,O;
	pcl::getMinMax3D(*ob,Min,Max);
	//get Centroid of circle around
	O.x=(Max.x+Min.x)/2;
	O.y=(Max.y+Min.y)/2;
	//create circle diameter;
	double radius=(pcl::euclideanDistance(Max,Min)+robot_width)/2;
	pcl::PointXYZ P(laser_transform.getOrigin().x(), laser_transform.getOrigin().y(),0);
	pcl::PointXYZ T1,T2;
	ROS_INFO("Center.x:%f,Center.y%f,O.x:%f,O.y:%f",Center[0],Center[1],O.x,O.y);
	pcl::PointXYZ normal(Center[0]-O.x,Center[1]-O.y,0);
	
	C.x=P.x-normal.x;
	C.y=P.y-normal.y;
	
	
	if(pcl::euclideanDistance(P,pcl::PointXYZ(Center[0],Center[1],0))<=radius)
	{
		//solve two gradients for tangents of the circle around the obstacle point cloud
		GetTangentPointsAtCircle(O,radius,P,T1,T2);
		//check eucledian distance to goal point 
		//set point C to the point with the lower value
		if(pcl::euclideanDistance(T1,B)<pcl::euclideanDistance(T2,B))
		{
		C=T1;		
		}else
		{
		C=T2;
		}
		
	}else
	{
		//no need to solve tangent points...
		//no need 
		C=P;
	}
	* */
	 //A means actual robot position, B the goal and C the next intermediate goal
	 //check distance from AC and CB and return this value
	double distance=pcl::euclideanDistance(A,C)+pcl::euclideanDistance(C,B);
	 
return distance;
	 
 }
 
 pcl::PointXYZ get_nearest_point(pcl::PointCloud<pcl::PointXYZ>::Ptr ob,pcl::PointXYZ A)
 {
	 pcl::PointXYZ Point;
	 double dist_min=1000;
	 for (int i=0; i<ob->points.size();i++)
	 {
		 double dist=pcl::euclideanDistance(ob->points[i],A);
		 if(dist_min>dist)
		 {
			 Point=ob->points[i];
			 dist_min=dist;
		 }
	 }
	 return Point;
 }
 
 //solves the equation of two possible tangent between circle with center and radius R and a point P
 //values of Tangent points get written to T1 and T2
 void GetTangentPointsAtCircle(pcl::PointXYZ Center, double R,pcl::PointXYZ P,pcl::PointXYZ &T1,pcl::PointXYZ &T2)
 {
	 //first shift point with center point of the circle to solve equation without shift
	 //also scale with radius to solve problem with unity circle
	 pcl::PointXYZ N;
	 N.x=(P.x-Center.x)/R;
	 N.y=(P.y-Center.y)/R;
	 
	 double xy=N.x*N.x+N.y*N.y;
	 if(xy==1)
	 {
		 //just one tangent...Point lays on circle
		T1=P;
		T2=P;
		//shift origin a little bit back to get a result..
		ROS_INFO("position=radius...");
		//shift point P
		//P.x=P.x*2;
		//P.y=P.y*2;
		
	 }else if(xy<1)
	 {
		 //error! no tangents possible
		 ROS_INFO("position in circle...");
		 T1=P;
		 T2=P;
		 //P.x=P.x*2;
		 //P.y=P.y*2;
	 }
	 //two tangents
	 double D=N.y*sqrt(xy-1);
	 T1.x=(N.x-D)/xy;
	 T2.x=(N.x+D)/xy;
	 
		 if(N.y!=0)
		 {
			 T1.y=Center.y+R*(1-T1.x)/N.y;
			 T2.y=Center.y+R*(1-T2.x)/N.y;
		 }else{
			 //point horizontal at the center
			 D=R*sqrt(1-T1.x*T1.x);
			 T1.y=Center.y+D;
			 T2.y=Center.y-D;		 
		 }
	 //reset center shift
	 T1.x=Center.x+R*T1.x;
	 T2.x=Center.x+R*T2.x;
	  
 }
 
 
};


//-------------------------------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_pcd_node");
	
	std::string source_str, data_str,publisher_str,odometry_str, goal_str;
	
	ros::NodeHandle n("~");
	if(!n.hasParam("source")) ROS_WARN("Used default parameter for source");
	n.param<std::string>("source", source_str, "/scan_cloud");
	if(!n.hasParam("laser_data")) ROS_WARN("Used default parameter for laser_data");
	n.param<std::string>("laser_data", data_str, "/laser_data");
	if(!n.hasParam("publisher")) ROS_WARN("Used default parameter for goal publisher");
	n.param<std::string>("publisher", publisher_str, "/goal");
	if(!n.hasParam("goal_sub")) ROS_WARN("Used default parameter for goal sub");
	n.param<std::string>("goal_sub", goal_str, "/goal");
	if(!n.hasParam("odometry_sub")) ROS_WARN("Used default parameter for odometry");
	n.param<std::string>("odometry_sub", odometry_str, "/odometry");
	
  
	LaserGoal l;
  
    n.param<std::string>("laser_frame", l.laserframe, "/lmslaser");
    n.param<std::string>("rabbit_frame", l.rabbitframe, "/rabbit_laser");
    
    ros::Subscriber o1=n.subscribe(source_str,1,&LaserGoal::readCloud,&l);
    ros::Subscriber o2=n.subscribe(data_str,1,&LaserGoal::check_laser_data,&l);
	ros::Subscriber o3=n.subscribe(goal_str,1,&LaserGoal::check_goal,&l);
	ros::Subscriber o4=n.subscribe(odometry_str,1,&LaserGoal::check_odometry,&l);
 
	//publisher
	l.pub_goal = n.advertise<geometry_msgs::PoseStamped>(publisher_str.c_str(),10);
	
	ros::spin();
					
	
}
