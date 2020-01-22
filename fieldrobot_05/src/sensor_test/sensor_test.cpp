#include "ros/ros.h"
#include <msgs/FloatStamped.h>

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <msgs/IntStamped.h>



//reads the signals from joystick topics and send them to the Motorcontroller command topic


class Sender_Node
{
	

public:
	// public variables
	ros::Publisher pub;
	msgs::FloatStamped test_sensor;
	std::string name, frame_id;
int counter;

	Sender_Node()
	{
			
	}
	
	~Sender_Node()
	{
	}
	
	//-------continuous publisher called by the node to garante a constant output---------------
	void PublishSpeed(const ros::TimerEvent& e)
	{
		
		if (counter<1000)
		{
			counter++;
		}else
		{
			counter=0;
		}
			test_sensor.data=counter;
			test_sensor.header.frame_id=frame_id;
			test_sensor.header.stamp=ros::Time::now();
			
			pub.publish(test_sensor);
	}
	
};
//-------------------------------------------------------------------------


int main (int argc, char** argv)
{
		ros::init(argc,argv,"sender_controller");
	ros::NodeHandle n("~");
	
 	
	Sender_Node s;
	//in seconds...
	n.param<std::string>("sensor", s.name, "test");
	n.param<std::string>("frame_id", s.frame_id, "test");

	//publisher
	s.pub = n.advertise<msgs::FloatStamped>(s.name.c_str(),10);
		//define the update rate by a seperate function...
	ros::Timer t;
	t = n.createTimer(ros::Duration(0.1), &Sender_Node::PublishSpeed,&s);
	
	ros::spin();
		
}
