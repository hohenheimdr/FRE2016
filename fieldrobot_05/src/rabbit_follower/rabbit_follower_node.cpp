/*
 * rabbit_follower_node.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: morl
 */


#include <ros/ros.h>
#include "RabbitFollow.h"

#include <fieldrobot_05/rabbit_follow_paramsConfig.h>
#include <dynamic_reconfigure/server.h>

void callback(fieldrobot_05::rabbit_follow_paramsConfig &config, uint32_t level,RabbitFollow* r)
{
  ROS_INFO("Reconfigure Request: P %.4f I %.4f Imax %.4f",config.P_gain,config.I_gain,config.I_max);
  r->setParams(config.P_gain,config.I_gain,config.D_gain,config.I_max,config.max_angular_vel,config.max_linear_vel,config.waypoint_dist);
}


int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "follow_the_rabbit");

	std::string r_frame,o_frame;
	std::string topic_id, goal_id;

	//Create Nodehandlers
	ros::NodeHandle nh("~");
	ros::NodeHandle n();

	nh.param<std::string>("rabbit_frame_id",r_frame,"/rabbit");
	nh.param<std::string>("vehicle_frame_id",o_frame,"/base_footprint");
	nh.param<std::string>("cmd_vel_topic_id",topic_id,"/fmControllers/cmd_vel");
	nh.param<std::string>("goal_distance",goal_id,"/fmControllers/goal_distance");

	RabbitFollow alice(r_frame,o_frame);

	nh.param<double>("field_of_view_rad",alice.fov,2*M_PI);
	nh.param<double>("max_angular_vel",alice.max_ang_vel,1.5);
	nh.param<double>("max_linear_vel",alice.max_lin_vel,1.0);
	nh.param<double>("oscilation_bounds",alice.oscilation_bound,0.05);
	nh.param<double>("target_acquired_tolerance",alice.waypoint_dist,0.1);
	nh.param<bool>("slow_down", alice.slow_down,true);
	nh.param<bool>("allow_backward_driving", alice.backward_driving,true);  
	alice.nh = &nh;

	ros::Timer t = nh.createTimer(ros::Duration(0.05),&RabbitFollow::spin,&alice);

	alice.cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(topic_id.c_str(),10);
	
	alice.goal_distance_pub = nh.advertise<std_msgs::Float64>(goal_id.c_str(),10);

	dynamic_reconfigure::Server<fieldrobot_05::rabbit_follow_paramsConfig> server;

	dynamic_reconfigure::Server<fieldrobot_05::rabbit_follow_paramsConfig>::CallbackType cb;

	cb = boost::bind(&callback, _1, _2,&alice);
	server.setCallback(cb);


	t.start();
	//Handle callbacks
	ros::spin();

}
