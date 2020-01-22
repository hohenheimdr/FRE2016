/*
 * RabbitFollow.cpp
 *
 *  Created on: Apr 27, 2012
 *      Author: morl
 */

#include "RabbitFollow.h"
#include <math.h>

RabbitFollow::RabbitFollow(std::string rabbit,std::string alice)
: tf_listen(ros::Duration(1))
{
	// TODO Auto-generated constructor stub
	vehicle_frame = alice;
	rabbit_frame = rabbit;

	I = P = 0;
	Ig = 0.001;
	Pg = 0.2;
	I_max = 0;



}

RabbitFollow::~RabbitFollow()
{
	// TODO Auto-generated destructor stub
}

void RabbitFollow::setParams(double P,double I,double D,double I_max,double max_ang,double max_lin,double waypoint_dist)
{
	Ig = I;
	Pg = P;
	Dg=D;
	//testparams
	
	this->I_max = I_max;
	this->waypoint_dist=waypoint_dist;
	max_ang_vel = max_ang;
	max_lin_vel = max_lin;
	//slow_down=false;
		
}

void RabbitFollow::spin(const ros::TimerEvent& e)
{
	findTheRabbit();

	// prevent oscilation when the rabbit is directly behind us.
	if(previous_rabbit_heading < -M_PI+oscilation_bound && current_rabbit_heading > M_PI-oscilation_bound)
	{
		current_rabbit_heading = previous_rabbit_heading;
	}
	else if(previous_rabbit_heading > M_PI-oscilation_bound && current_rabbit_heading < -M_PI + oscilation_bound)
	{
		current_rabbit_heading = previous_rabbit_heading;
	}

	ROS_DEBUG_THROTTLE(1,"running with max vel: %.4f",max_lin_vel);

	driveToTheRabbit();
}

void RabbitFollow::findTheRabbit()
{
	try
	{
		tf::StampedTransform transformer;
		tf_listen.lookupTransform(vehicle_frame,rabbit_frame,ros::Time(0), transformer);

		tf::Vector3 xyz = transformer.getOrigin();

				previous_rabbit_heading = current_rabbit_heading;
				current_rabbit_heading =atan2(xyz[1],xyz[0]);
				distance = sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1]);

				ROS_DEBUG("Found rabbit %.4f %.4f %.4f distance %.4f",xyz[0],xyz[1],current_rabbit_heading,distance);

	}
	catch (tf::TransformException& ex){
		ROS_DEBUG_THROTTLE(1,"follower: FAILED!");
		ROS_DEBUG_THROTTLE(1,"%s",ex.what());
		current_rabbit_heading = 0;
		distance = 0;
	}
}

void RabbitFollow::driveToTheRabbit()
{

	//TODO: do some fancy scaling  of the cmd_vel
	//change maybe later to a PID controler?
	// PID controller

	I = current_rabbit_heading*Ig; //*0.1
	if(I > I_max)
	{
		I = I_max;
	}
	else if(I < -I_max)
	{
		I = -I_max;
	}

	P = current_rabbit_heading * Pg;
	D = (current_rabbit_heading -previous_rabbit_heading)* Dg;

	cmd_vel.twist.angular.z = (P + I + D);
	
	
	
	
//case distance is near level down the speed
	goal_distance.data=distance;
	if(distance < 1.0 && slow_down)
	{
		//ROS_INFO_THROTTLE(10,"target_reached");
		//cmd_vel.twist.angular.z = (P + I*Ig)*distance;
		cmd_vel.twist.linear.x = 0.1+max_lin_vel*distance;
		//cmd_vel.twist.linear.x = max_lin_vel;
		//test
		
	}else
	{
	cmd_vel.twist.linear.x = max_lin_vel;
	}
	
	//if(current_rabbit_heading > fov/2 || current_rabbit_heading < -fov/2)
	if(abs(cmd_vel.twist.angular.z)>=max_ang_vel)
	{
		// turn in place
		cmd_vel.twist.linear.x = 0;
	}
	
	//set this to global parameter of dynamic reconfigure
	if(distance < waypoint_dist && slow_down)
	{
		ROS_INFO_THROTTLE(10,"target_reached");
		cmd_vel.twist.angular.z = 0;
		cmd_vel.twist.linear.x = 0;
	}

	if(cmd_vel.twist.angular.z > max_ang_vel)
	{
		cmd_vel.twist.angular.z = max_ang_vel;
	}

	if(cmd_vel.twist.angular.z < -max_ang_vel)
	{
		cmd_vel.twist.angular.z = - max_ang_vel;
	}

	cmd_vel_pub.publish(cmd_vel);
	goal_distance_pub.publish(goal_distance);

}


