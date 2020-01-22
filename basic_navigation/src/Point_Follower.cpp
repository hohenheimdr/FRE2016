/*
 * PointFollower.cpp
 *
 *  Created on: 12.10.15 
 *      Author: david
 * 
 * dr: some changes for the fieldrobot event 28.5.16
 *		- include PID
 * 		- include turn on spot
 * 		- include
 */


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <tf/transform_listener.h>


class PointFollower
{
	private:
		double distance, heading,last_heading,last_distance;
		double integral, deviation,last_heading_error;
		tf::TransformListener tf_listen;
		geometry_msgs::Twist cmd_vel;
		tf::Vector3 xyz;
		ros::Time last_tf_time;
		
		double rel_heading;

	public:
		std::string r_frame,v_frame, odom_frame;
		double thresh_min,dist_thresh,angle_thresh;
		
		ros::Publisher cmd_vel_pub;
		ros::Publisher goal_distance_pub;
		ros::Publisher goal_reached_pub;
		double angle_offset,speed_max,angle_speed_max,linear_offset,time_offset;
		double P,I,D,angle_react_thresh,start_moving_angle;
		bool use_PID_for_turn;

		PointFollower ()
		{
			thresh_min=0.05;
		}
		~PointFollower()
		{
			//stop program with zero command for the speed
			cmd_vel.angular.z=0;	
			cmd_vel.linear.x=0;
			cmd_vel_pub.publish(cmd_vel);
		}
		
		void spin(const ros::TimerEvent& e)
		{
			getDistance();
			createSpeed();
		}
		
		void getDistance()
		{
			try
			{
				tf::StampedTransform transformer;
				tf_listen.lookupTransform(v_frame,r_frame,ros::Time(0), transformer);
				xyz = transformer.getOrigin();
				last_heading=heading;
				heading =atan2(xyz[1],xyz[0]);
				last_distance=distance;
				distance = sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1]);
				std_msgs::Float64 dist;
				dist.data=distance;
				//publish the distance as a topic
				goal_distance_pub.publish(dist);
				rel_heading=transformer.getRotation().getZ();
				ROS_INFO("Found rabbit at x=%.3f y=%.3f angle=%.3f dist=%.3f",xyz[0],xyz[1],heading,distance);
				//save time...
				last_tf_time=ros::Time::now();

			}
			catch (tf::TransformException& ex){
				ROS_DEBUG_THROTTLE(1,"tf: FAILED!");
				ROS_DEBUG_THROTTLE(1,"%s",ex.what());
				heading = 0;
				distance = 0;
				last_distance=0;
			}
		}
	
		void createSpeed()
		{
			//take care that all errors get a maximum of +/-1 as value
			double heading_error=heading;
				if(heading_error>1.57)
					heading_error=3.1415-heading_error;
				if(heading_error<-1.57)
					heading_error=-3.1415-heading_error; 
					
			ROS_INFO("heading_error :%f",heading_error);
			//this part is not perfect... need some more tests....
			integral=(integral+heading_error)/2.0;
			deviation=(heading_error-last_heading_error)/2;
			last_heading_error=heading_error;
			//publish msgs to check if goal reached...
			std_msgs::Bool dummy;
			//check if robot is turned right to the goal, if not, first turn to the right direction!
			if(heading_error>angle_react_thresh){
				
				cmd_vel.angular.z=(angle_speed_max*(P*heading_error+I*integral+D*deviation)/3+angle_offset);
					if(!use_PID_for_turn)
						cmd_vel.angular.z=angle_speed_max;
							}
				
			if(heading_error<(-angle_react_thresh)){
				cmd_vel.angular.z=(angle_speed_max*(P*heading_error+I*integral+D*deviation)/3-angle_offset);
					if(!use_PID_for_turn)
						cmd_vel.angular.z=angle_speed_max;
				}
			
			if(sqrt(heading_error*heading_error)<angle_thresh)
			{
				cmd_vel.angular.z=0;
			}
			if(sqrt(heading_error*heading_error)<start_moving_angle && distance>dist_thresh)
			{
				double distance_error=xyz[0];
					if (distance_error>1)
						distance_error=1;
					if(distance_error<-1)
						distance_error=-1;
						if(distance_error>0)
							cmd_vel.linear.x=speed_max*P*distance_error+linear_offset;
						if(distance_error<0)
							cmd_vel.linear.x=speed_max*P*distance_error-linear_offset;
				//case if we move backwards, we have to inverse the angle value
			if(distance_error<0)
				cmd_vel.angular.z=-cmd_vel.angular.z;
				
			}
			
			//reached position
			if(distance<dist_thresh)
			{
				cmd_vel.linear.x=0;
				//check the orientation if neccessary!
				ROS_INFO("rel_heading %f",rel_heading);
				
					cmd_vel.angular.z=(angle_speed_max*(P*rel_heading)+angle_offset);
					//define if goal was reached
					if(heading==0 && xyz[0]==0 && xyz[1]==0)
						cmd_vel.angular.z=0;
				if(sqrt(rel_heading*rel_heading)<dist_thresh)
				{	
					cmd_vel.angular.z=0;
					cmd_vel.linear.x=0;
					dummy.data=true;
					goal_reached_pub.publish(dummy);
					//sleep(0.1);
					//dummy.data=false;
					//goal_reached_pub.publish(dummy);
				}
	
			}else
			{
				dummy.data=false;
				goal_reached_pub.publish(dummy);
			}
			//when timeout is longer than time_offset (in seconds), skip everything
			if(abs_d((ros::Time::now()-last_tf_time).toSec())>time_offset)
			{
				ROS_INFO("no goal...");
				cmd_vel.linear.x=0;
				cmd_vel.linear.y=0;
				cmd_vel.linear.z=0;
				cmd_vel.angular.y=0;
				cmd_vel.angular.x=0;
				cmd_vel.angular.z=0;
			}
			
			cmd_vel_pub.publish(cmd_vel);
			ROS_INFO("x=%f z=%f" , cmd_vel.linear.x,cmd_vel.angular.z);
		}
		
		double abs_d(double in){
			return sqrt(in*in);
		}

};


int main(int argc, char **argv) {

	//Initialize ros usage
	ros::init(argc, argv, "PointFollower");

	std::string topic_id, goal_id, goal_reached;

	//Create Nodehandlers
	ros::NodeHandle n("~");
	
	PointFollower pf;

	n.param<std::string>("rabbit_frame_id",pf.r_frame,"/rabbit");
	n.param<std::string>("vehicle_frame_id",pf.v_frame,"/base_link");
	n.param<std::string>("odom_frame_id",pf.odom_frame,"/odom");
	n.param<std::string>("cmd_vel_topic_id",topic_id,"/cmd_vel");
	n.param<std::string>("goal_distance",goal_id,"/goal_distance");
	n.param<std::string>("goal_reached",goal_reached,"/goal_reached");
	n.param<double>("angle_offset",pf.angle_offset,0.1);
	
	n.param<double>("linear_offset",pf.linear_offset,0.1);
	//distance for defining if goal was reached
	n.param<double>("dist_thresh",pf.dist_thresh,0.1);
	//difference acceptable for angle
	n.param<double>("angle_thresh",pf.angle_thresh,0.1);
	//just react on angle changes when this tresh was reached!
	n.param<double>("angle_react_thresh",pf.angle_react_thresh,0.0);
	n.param<double>("start_moving_angle",pf.start_moving_angle,0.75);
	n.param<double>("time_offset",pf.time_offset,0.1);
	n.param<double>("speed_max",pf.speed_max,0.1);
	n.param<double>("angle_speed_max",pf.angle_speed_max,0.1);
	n.param<bool>("use_PID_for_turn",pf.use_PID_for_turn,true);
	n.param<double>("P",pf.P,0.1);
	n.param<double>("I",pf.I,0.1);
	n.param<double>("D",pf.D,0.1);
	
	pf.cmd_vel_pub = n.advertise<geometry_msgs::Twist>(topic_id.c_str(),10);
	pf.goal_distance_pub = n.advertise<std_msgs::Float64>(goal_id.c_str(),10);
	pf.goal_reached_pub = n.advertise<std_msgs::Bool>(goal_reached.c_str(),10);
	
	ros::Timer t = n.createTimer(ros::Duration(0.5),&PointFollower::spin,&pf);

	ros::spin();

}


