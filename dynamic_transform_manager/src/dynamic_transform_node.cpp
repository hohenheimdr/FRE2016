/*ros node for handling 3 different laser scanners of the type lms 111
 * the tf for all can be dynamicaly set and the output published in 
 * one single laserscan refering to the robot frame
 * 
 * also a bunch of different settings can be set for every sensor
 * also as dynamic filter appling * 
 */
#include <ros/ros.h>
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include <ros/subscriber.h>
#include <dynamic_transform_manager/DynamicTransformConfig.h>
#include <dynamic_reconfigure/server.h>

using namespace std;

class DynamicTransform
{
private:	

		
	// public variables
public:
    double frequency;
    bool transform_active;
	dynamic_reconfigure::Server<dynamic_transform_manager::DynamicTransformConfig> server_;
	tf::TransformListener li;
	tf::TransformBroadcaster br;
	string frame_id, reference_id;
	tf::Transform transform;
	std_msgs::Float64 cur_tilt_angle;
	double x,y,z,roll,pitch,yaw,tilt_angle;
	
		//geometry_msgs::TransformStamped odom_trans;

	//start with normal constructor
	DynamicTransform()
	{
		//set up dynamic reconfigure server
		dynamic_reconfigure::Server<dynamic_transform_manager::DynamicTransformConfig>::CallbackType f;
	    f = boost::bind(&DynamicTransform::configure,this, _1, _2);
	    server_.setCallback(f);
	    frequency=0;
	    transform_active=true;
	    tilt_angle=0;
	    //set startvalues for the dynamic transform....
		std::string com, number;
	    //set frame_id
	    //com="rosrun dynamic_transform_manager dynamic_transform_node _
	    
	    
			//number=std::to_string(kinect_tilt);
			//com="rosrun kinect_aux kinect_aux_node _tilt_angle:="+number+ " &";	
			//system(com.c_str());
	    
	    
	    
	}
	
	~DynamicTransform()
	{
	}
		
	void configure(dynamic_transform_manager::DynamicTransformConfig &config, uint32_t level) {
		ROS_INFO("reading dynamic reconfigure files");
		transform_active=config.active;
		//convert Hz to seconds..
		frequency=double(1/config.frequency);
		x=config.x;
		y=config.y;
		z=config.z;
		roll=config.roll;
		pitch=config.pitch;
		yaw=config.yaw;
		frame_id=config.frame_id;
	    reference_id=config.reference_id;
	
	}
	
	void readImu(const std_msgs::Float64::ConstPtr& msg)
	{
		tilt_angle=-(msg->data*2*M_PI/360);		
	}
	
	void publish_Transforms(const ros::TimerEvent& e)
	{
		try{
			
		if(transform_active)
			{
			transform.setOrigin(tf::Vector3(x,y,z));
		    transform.setRotation(tf::createQuaternionFromRPY(roll,pitch+tilt_angle,yaw));
				
			br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),reference_id,frame_id));
			}
		}
		catch(tf::TransformException ex)
			{
			ROS_INFO("error with transform ");							
			}
	}
		
};
//-------------------------------------------------------------------------

int main(int argc, char** argv){
  ros::init(argc, argv,"apply_filter");

ros::NodeHandle n("~");
      
	DynamicTransform tr;
	string subscriber;

	//publish transforms with 50 Hz
	ros::Timer t;
	//use timer with transform frequency
	t = n.createTimer(ros::Duration(tr.frequency), &DynamicTransform::publish_Transforms,&tr);
	n.param<std::string>("imu_sub", subscriber, "/kinect_aux/cur_tilt_angle");
	ros::Subscriber o=n.subscribe(subscriber,1,&DynamicTransform::readImu,&tr);  
	
	//n.param<std::string>("parent", tr.frame_id,"base_link");
	//n.param<std::string>("child", tr.reference_id,"laser");
	//n.param<double>("x", tr.x,0);
	//n.param<double>("y", tr.y,0);
	//n.param<double>("z", tr.z,0);
	//n.param<double>("roll", tr.roll,0);
	//n.param<double>("pitch", tr.pitch,0);
	//n.param<double>("yaw", tr.yaw,0);
	 
	ros::spin();
  

}
