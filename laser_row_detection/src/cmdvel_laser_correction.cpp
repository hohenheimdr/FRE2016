#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <ros/subscriber.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64MultiArray.h>
#include "nav_msgs/Odometry.h"
#include <math.h>
#include "geometry_msgs/Twist.h"

// Executable zur Ermittlung eins Korrekturwertes zur Navigation zwischen zwei Pflanzenreihen
// von Max Staiger & Markus Ullrich 17.3.16
//überarbeitet David Reiser am 1.6.16
//added normal twist as additional output




class PCL_CLASS
{
public:																					//Bestimmung der Variablen

	double position_left,position_right,orientation_left,orientation_right,correction;
	geometry_msgs::TwistStamped cmd_vel_laser_correction, cmd_vel_headland;
	geometry_msgs::Twist cmd_vel_twist;
	//geometry_msgs::PoseStamped Position_right;
	
	ros::Publisher correction_pub, headland_pub,twist_pub;
	
	PCL_CLASS()
	{}
	
	~ PCL_CLASS()
	{}
	
	void Position_left (const geometry_msgs::PoseStamped::ConstPtr& msg)				//Einlesen und Umbenennen der Variablen aus Ransac Filter Linke Seite
	{
	position_left =msg->pose.position.y;
	orientation_left=msg->pose.orientation.z;
	position_left=fabs (position_left);
	orientation_left=fabs (orientation_left);
	
	//std::cout<<"\r  pose_y_test"<<position_left<<std::endl;							//Ausgabe einzelner Werte zur Kontrolle
	Compare();

	//std::cout<<"\r  pose_y:"<<msg->pose.position.y<<std::endl;
	//std::cout<<"\r  orientl_z:"<<msg->pose.orientation.z<<std::endl;
	}
	
	void Position_right (const geometry_msgs::PoseStamped::ConstPtr& msg)				//Einlesen und Umbenennen der Variablen aus Ransac Filter Rechte Seite
	{
	position_right =msg-> pose.position.y;
	orientation_right=msg->pose.orientation.z;
	position_right=fabs(position_right);
	orientation_right=fabs(orientation_right);
	
	//std::cout<<"\r  pose_y:"<<position_right<<std::endl;								//Ausgabe der einzelnen Werte zur Kontrolle
	Compare();
	//std::cout<<"\r  poser_y:"<<msg->pose.position.y<<std::endl;
	//std::cout<<"\r  orientr_z:"<<msg->pose.orientation.z<<std::endl;

	}
	
	double fabs(double input)															//Bildung des Betrages vom Input 
	{
		return sqrt(input*input);
	}
	
	void Compare()																		//Vergleich von Orientation(Winkel) links und rechts
	{
		std_msgs::Bool headland_b;
		if (orientation_left > 0.35 || orientation_right > 0.35)						//Wenn Winkel außerhalb +- 0,35 ist Reihe zu Ende 
		{
			cmd_vel_headland.twist.linear.x=0.00;
			headland_b.data=true;																		//setze Geschwindigkeit 0 und gebe info aus wenn Vorgewende erkannt
			headland_pub.publish(headland_b);
			twist_pub.publish(cmd_vel_headland.twist);
			ROS_INFO ("headland"); 														//Textausgabe wenn Vorgewende erkannt
		}
		
		else 
		{		
			headland_b.data=false;														//setze Geschwindigkeit 0 und gebe info aus wenn Vorgewende erkannt
			headland_pub.publish(headland_b);																					//Falls kein Headland, Korrekturwert zur Richtungsanpassung in der Reihe
			correction = (position_left+position_right)+(orientation_left+orientation_right);		//Ermittlung des Korrekturwertes -=rechts +=links
			//std::cout<<"\r  correction:"<<correction<<std::endl;
			cmd_vel_laser_correction.twist.angular.z=correction;									//Bestimmung Korrektur als cmd_vel
			correction_pub.publish(cmd_vel_laser_correction);
			twist_pub.publish(cmd_vel_laser_correction.twist);
		}
	}
	
	
};

int main(int argc, char** argv)
{

ros::init(argc, argv, "ransac");

ros::NodeHandle a("~");

 PCL_CLASS b;
 std::string pose_left,pose_right,correction, headland,twist;
	a.param<std::string>("pose_left",pose_left, "/ransac_left_pose");
	a.param<std::string>("pose_right",pose_right, "/ransac_right_pose");
	a.param<std::string>("correction_out",correction,"/correction");
	a.param<std::string>("headland_out",headland,"/headland");
	a.param<std::string>("cmd_vel_out",twist,"/cmd_vel_row");
	
	ros::Subscriber sub_left_r = a.subscribe(pose_left,50, &PCL_CLASS::Position_left, &b); 		//Einlesen der PCL links
	ros::Subscriber sub_right_r = a.subscribe(pose_right,50, &PCL_CLASS::Position_right, &b); 	//Einlesen der PCL rechts

	b.correction_pub=a.advertise<geometry_msgs::TwistStamped>(correction.c_str(), 50);			//Publizierung des Korrekturwertes im Topic
	b.headland_pub=a.advertise<std_msgs::Bool>(headland.c_str(), 50);				  	 		//Publizierung der Headlanderkennung im Topic
	b.twist_pub=a.advertise<geometry_msgs::Twist>(twist.c_str(), 50);	
	
	ros::spin();
	
}
