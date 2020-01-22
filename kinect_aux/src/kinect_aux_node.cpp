#include <libusb-1.0/libusb.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"

#include <kinect_aux/KinectAuxConfig.h>
#include <dynamic_reconfigure/server.h>


   

// VID and PID for Kinect and motor/acc/leds
#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
// Constants for accelerometers
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.
// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
// The control input looks like 2*desired_degrees
#define MAX_TILT_ANGLE 2.
#define MIN_TILT_ANGLE (-61.)

class KinectAux
{
public:
	
	dynamic_reconfigure::Server<kinect_aux::KinectAuxConfig> server_;

	tf::TransformBroadcaster br;
	tf::Transform trans_imu;
	
	ros::Publisher pub_imu;
	ros::Publisher pub_tilt_angle;
	ros::Publisher pub_tilt_status;
	ros::Publisher pub_tilt;
	
	
	ros::Subscriber sub_tilt_angle;
	ros::Subscriber sub_led_option;
	
	libusb_device_handle *dev;
	//strings for input of launch file
	std::string imu_publisher_str,imu_frame_id,base_frame_id;
	//angle defined by user an dynamic reconfigure
	std_msgs::Float64 angle;
	
	bool dynamic_tilt;
	bool publish_trans;
	
	KinectAux()
	{
		//set up dynamic reconfigure server
		dynamic_reconfigure::Server<kinect_aux::KinectAuxConfig>::CallbackType f;
	    f = boost::bind(&KinectAux::configure,this, _1, _2);
	    server_.setCallback(f);
		
	}
	~KinectAux()
	{
	}

	void openAuxDevice(int index = 0)
	{
		libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
		ssize_t cnt = libusb_get_device_list (0, &devs); //get the list of devices
		if (cnt < 0)
		{
			ROS_ERROR("No device on USB");
			return;
		}
		
		int nr_mot(0);
		for (int i = 0; i < cnt; ++i)
		{
			struct libusb_device_descriptor desc;
			const int r = libusb_get_device_descriptor (devs[i], &desc);
			if (r < 0)
				continue;

			// Search for the aux
			if (desc.idVendor == MS_MAGIC_VENDOR && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT)
			{
				// If the index given by the user matches our camera index
				if (nr_mot == index)
				{
					if ((libusb_open (devs[i], &dev) != 0) || (dev == 0))
					{
						ROS_ERROR_STREAM("Cannot open aux " << index);
						return;
					}
					// Claim the aux
					libusb_claim_interface (dev, 0);
					break;
				}
				else
					nr_mot++;
			}
		}

		libusb_free_device_list (devs, 1);  // free the list, unref the devices in it
	}


	void publishState(const ros::TimerEvent& e)
	{
		uint8_t buf[10];
		const int ret = libusb_control_transfer(dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0);
		if (ret != 10)
		{
			ROS_ERROR_STREAM("Error in accelerometer reading, libusb_control_transfer returned " << ret);
			ros::shutdown();
		}
		
		const uint16_t ux = ((uint16_t)buf[2] << 8) | buf[3];
		const uint16_t uy = ((uint16_t)buf[4] << 8) | buf[5];
		const uint16_t uz = ((uint16_t)buf[6] << 8) | buf[7];
		
		const int16_t accelerometer_x = (int16_t)ux;
		const int16_t accelerometer_y = (int16_t)uy;
		const int16_t accelerometer_z = (int16_t)uz;
		const int8_t tilt_angle = (int8_t)buf[8];
		const uint8_t tilt_status = buf[9];
		
		// publish IMU
		sensor_msgs::Imu imu_msg;
		if (pub_imu.getNumSubscribers() > 0)
		{
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.linear_acceleration.x = (double(accelerometer_x)/FREENECT_COUNTS_PER_G)*GRAVITY;
			imu_msg.linear_acceleration.y = (double(accelerometer_y)/FREENECT_COUNTS_PER_G)*GRAVITY;
			imu_msg.linear_acceleration.z = (double(accelerometer_z)/FREENECT_COUNTS_PER_G)*GRAVITY;
			imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4]
				= imu_msg.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
			imu_msg.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
			imu_msg.orientation_covariance[0] = -1; // indicates orientation not provided
			//set frame_id
			imu_msg.header.frame_id=imu_frame_id.c_str();
			pub_imu.publish(imu_msg);
			
		}
		
		// publish tilt angle and status
		if (pub_tilt_angle.getNumSubscribers() > 0)
		{
			std_msgs::Float64 tilt_angle_msg;
			tilt_angle_msg.data = double(tilt_angle) / 2.;
			pub_tilt_angle.publish(tilt_angle_msg);
		}
		if (pub_tilt_status.getNumSubscribers() > 0)
		{
			std_msgs::UInt8 tilt_status_msg;
			tilt_status_msg.data = tilt_status;
			pub_tilt_status.publish(tilt_status_msg);
			
		}
		
		if (publish_trans)
		{
			//set Transform and publish
			trans_imu.setOrigin(tf::Vector3(0,0,0));
			trans_imu.setRotation(tf::createQuaternionFromRPY(0,tilt_status,0));
			publish_Transform();
		}
		
		if(dynamic_tilt)
		{
		pub_tilt.publish(angle);
		}
	}


	void publish_Transform()
	{
		try{
			
			br.sendTransform(tf::StampedTransform(trans_imu,ros::Time::now(),imu_frame_id,base_frame_id));
			
		}
		catch(tf::TransformException ex)
			{
			ROS_INFO("error with transform ");							
			}
	}

	void setTiltAngle(const std_msgs::Float64 angleMsg)
	{
		
		uint8_t empty[0x1];
		double angle(angleMsg.data);
		
		//angle=0;
		
		angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
		angle = angle * 2;
		//ROS_INFO("angle= %f",angle);
		const int ret = libusb_control_transfer(dev, 0x40, 0x31, (uint16_t)angle, 0x0, empty, 0x0, 0);
		
		if (ret != 0)
		{
			ROS_ERROR_STREAM("Error in setting tilt angle, libusb_control_transfer returned " << ret);
			ros::shutdown();
		}
		//sleepms(10);
		sleep(0.1);
	}

	void setLedOption(const std_msgs::UInt16 optionMsg)
	{
		uint8_t empty[0x1];
		const uint16_t option(optionMsg.data);
		
		const int ret = libusb_control_transfer(dev, 0x40, 0x06, (uint16_t)option, 0x0, empty, 0x0, 0);
		if (ret != 0)
		{
			ROS_ERROR_STREAM("Error in setting LED options, libusb_control_transfer returned " << ret);
			ros::shutdown();
		}
	}
	
	void configure(kinect_aux::KinectAuxConfig &config, uint32_t level) {
		
		dynamic_tilt=config.start_tilt;
		angle.data=config.tilt_angle;
		imu_frame_id=config.imu_frame;
		base_frame_id=config.base_frame;
		ROS_INFO("setting tilt to %f",angle.data);
		publish_trans=config.publish_transform;
		
	}

};// end class KinectAux


int main(int argc, char* argv[])
{
	int ret = libusb_init(0);
	if (ret)
	{
		ROS_ERROR_STREAM("Cannot initialize libusb, error: " << ret);
		return 1;
	}
	
	ros::init(argc, argv, "kinect_aux");
	ros::NodeHandle n("~");
	
	KinectAux k;
  
  if(!n.hasParam("publisher_imu")) ROS_WARN("Used default parameter for imu_publisher");
  n.param<std::string>("publisher", k.imu_publisher_str, "imu");
   n.param<std::string>("frame_id", k.imu_frame_id, "kinect");
  
   //cloud_msg.header.frame_id=frame_id.c_str();
  
  //set frame id first
	
	int deviceIndex;
	n.param<int>("device_index", deviceIndex, 0);
	k.openAuxDevice(deviceIndex);
	if (!k.dev)
	{
		ROS_ERROR_STREAM("No valid aux device found");
		libusb_exit(0);
		return 2;
	}
	
	k.pub_imu = n.advertise<sensor_msgs::Imu>(k.imu_publisher_str.c_str(), 15);
	k.pub_tilt_angle = n.advertise<std_msgs::Float64>("cur_tilt_angle", 15);
	k.pub_tilt_status = n.advertise<std_msgs::UInt8>("cur_tilt_status", 15);
	k.pub_tilt= n.advertise<std_msgs::Float64>("tilt_angle", 15);
	
	//sleep(1);
	k.sub_tilt_angle = n.subscribe("tilt_angle", 10,&KinectAux::setTiltAngle,&k);
	k.sub_led_option = n.subscribe("led_option", 10, &KinectAux::setLedOption,&k);
	
	//
	//publish transforms with 10 Hz
	ros::Timer t;
	t = n.createTimer(ros::Duration(0.1), &KinectAux::publishState,&k);
	
	ros::spin();
	libusb_exit(0);
	return 0;
}
