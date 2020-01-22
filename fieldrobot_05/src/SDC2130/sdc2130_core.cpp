#include "SDC2130.h"
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <msgs/IntStamped.h>
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/TwistStamped.h"
#include <nav_msgs/Odometry.h>


using namespace std;

class M_Controller
{
private:	
	
	//setting variables and publish
		msgs::IntStamped encoder_msg_ch1;
		msgs::IntStamped encoder_msg_ch2;
		std_msgs::Int64 motor_speed_msg_ch1;
		std_msgs::Int64 motor_speed_msg_ch2;
		std_msgs::Float64 voltage_msg_ch1;
		std_msgs::Float64 voltage_msg_ch2;
		std_msgs::Float64 ampere_msg_ch1;
		std_msgs::Float64 ampere_msg_ch2;
		
		
		
	
	// public variables
public:

	ros::Publisher pub_motor_speed_ch1;
		ros::Publisher pub_motor_speed_ch2;
		ros::Publisher pub_encoder_ch1;
		ros::Publisher pub_encoder_ch2;
		ros::Publisher pub_ampere_ch1;
		ros::Publisher pub_ampere_ch2;
		ros::Publisher pub_voltage_ch1;
		ros::Publisher pub_voltage_ch2;
		
	
	std::string port;

	double speedcommand_ch1=0;
	double speedcommand_ch2=0;

	//sending the goalcommands to the motorcontrollers
	double vmax,angle_max,output_max,output_offset_left,output_offset_right; //max velocity,max angle and maximal outputvalue
	double actual_speed,actual_angle,speed_soll,angle_soll;
	bool closed_loop,toggle_turn;
	//PID values
	double P,Pang;
	double I,Iang;
	double D,Dang;
	double actual_error_lin, actual_error_ang;
	double last_error_lin, last_error_ang,last_error_lin2, last_error_ang2;
	double error_integral;
	double error_integral_ang;
	double W;
	int status;
	int amax,dmax;
	bool motor_connected;

	double ticks_left,ticks_right;
	//create the motorcontroller object
	SDC2130 M;
    
	
	M_Controller()
	{
	
	}
	
	~M_Controller()
	{
	}
	
	void init()
	{
		status=0;
		status=M.init(port,amax,dmax);
		//publish variables		
		
	}
	
	void publish_topics(const ros::TimerEvent& e)
	{	
		status=status+M.readEncoders();
		encoder_msg_ch1.data = M.encoder_ch1;
		pub_encoder_ch1.publish(encoder_msg_ch1);
		encoder_msg_ch2.data = M.encoder_ch2;
		pub_encoder_ch2.publish(encoder_msg_ch2);
		status=status+M.setSpeed(1,speedcommand_ch1);
		//loop_rate.sleep();
		status=status+M.setSpeed(2,speedcommand_ch2);
		//loop_rate.sleep();
		status=status+M.readMotors();	
		motor_speed_msg_ch1.data = M.motor_speed_ch1;
		pub_motor_speed_ch1.publish(motor_speed_msg_ch1);		
		motor_speed_msg_ch2.data = M.motor_speed_ch2;
		pub_motor_speed_ch2.publish(motor_speed_msg_ch2);
		status=status+M.readVoltage();
		voltage_msg_ch1.data = M.voltage_ch1;
		voltage_msg_ch2.data = M.voltage_ch2;
		pub_voltage_ch1.publish(voltage_msg_ch1);
		pub_voltage_ch2.publish(voltage_msg_ch2);

		status=status+M.readAmpere();
		ampere_msg_ch1.data = M.ampere_ch1;
		ampere_msg_ch2.data = M.ampere_ch2;				
		pub_ampere_ch1.publish(ampere_msg_ch1);
		pub_ampere_ch2.publish(ampere_msg_ch2);
		//break the node if system gets to much erros...
		if(status>5){
			motor_connected=false;
		}
		
	}
	
	void getOdometry(const nav_msgs::Odometry::ConstPtr& msgs)
	{
	  actual_speed=msgs->twist.twist.linear.x;	
	  actual_angle=msgs->twist.twist.angular.z;
	}

	void setSpeed(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		ROS_INFO("got speed msg");
		speed_soll=double(msg->twist.linear.x);
		angle_soll=double(msg->twist.angular.z);
		//define start values
				
		if(toggle_turn)
		{
			angle_soll=-angle_soll;
		}
		//case if no closed loop just use the twist msg to produce output (einfache Steuerung)
		
	if (closed_loop)
	{
			double P_speed,I_speed,D_speed,P_ang,D_ang,I_ang=0;	
			//define the error values for speed and angle
			actual_error_lin=(speed_soll-actual_speed);
			actual_error_ang=-(angle_soll-actual_angle);
			
			//calculate the PID values for the torque controller	
			P_speed=P*actual_error_lin;
			I_speed=I*error_integral;
			D_speed=D*last_error_lin2;
			error_integral=error_integral+actual_error_lin;
			//set a limit for the integral
			if (abs(error_integral)>100.0)
			{
				error_integral=0.0;
			}
			//calculate the PID values for the angle controll
			P_ang=Pang*actual_error_ang;
			error_integral_ang=error_integral_ang+P_ang;
			I_ang=Iang*error_integral_ang;
			D_ang=Dang*last_error_ang;
			//set a limit for the error integral
			if (abs(error_integral_ang)>10.0)
			{
				error_integral_ang=0.0;
			}
			//catch inf values for the D_angle
			if(abs(D_ang)>10.0)
			D_ang=0.0;
			
		//ROS_INFO("P %f I %f D %f",actual_error_lin,error_integral,last_error_lin);
		//ROS_INFO("Pang %f Iang %f Dang %f",actual_error_ang,error_integral_ang,last_error_ang);
		//correct angle values with PID controler values
		actual_angle=angle_soll+P_ang+I_ang+D_ang;
		//calculate new torque value neccessary
		actual_speed=speed_soll+P_speed+I_speed+D_speed;
		//define last error values
		last_error_lin2=last_error_lin;
		last_error_lin=P_speed+I_speed+D_speed;
		last_error_ang2=last_error_ang;
		last_error_ang=P_ang+I_ang+D_ang;
		
		ROS_INFO("cmd:= %f, %f",actual_speed,actual_angle);
		
	}
	else
	{
		actual_speed=speed_soll;
		actual_angle=angle_soll;
	}
	
	//only take the max value for the speed and angle ( normated...)
		if (actual_speed>1.0)
		actual_speed=1.0;
		if (actual_speed<-1.0)
		actual_speed=-1.0;
		if (W*actual_angle>2.0)
		actual_angle=2.0;
		if (W*actual_angle<-2.0)
		actual_angle=-2.0;	
	
	//convert speed to differential steering values
	 double vel_right = double(actual_speed + ( W * actual_angle));
	 double vel_left = double(actual_speed - ( W * actual_angle));
	 //norm value of joystick
	 if (vel_right>1.0)
	 vel_right=1;
	 if (vel_right<-1.0)
	 vel_right=-1;
	 if (vel_left>1.0)
	 vel_left=1;
	 if (vel_left<-1.0)
	 vel_left=-1;
	  
	//check if cmd vel was 0
	 if (speed_soll==0 && angle_soll==0)
	 {
			speedcommand_ch1=0;
			speedcommand_ch2=0;
	 }else
	  {
		  //create a percentage because of the tick difference... and add this later to one side of the motors 
		 //set motorcontroller outputs.
		
		if (vel_right>0)
		speedcommand_ch1=output_offset_right+vel_right*vmax*(output_max-output_offset_left);//+output_offset_left)*ticks_right/ticks_left;
		if (vel_right<0)
		speedcommand_ch1=-output_offset_right+vel_right*vmax*(output_max-output_offset_left);//-output_offset_left)*ticks_right/ticks_left;
		if (vel_left>0)
		speedcommand_ch2=output_offset_left+vel_left*vmax*(output_max-output_offset_right);//+output_offset_right)*ticks_left/ticks_right;
		if (vel_left<0)
		speedcommand_ch2=-output_offset_left+vel_left*vmax*(output_max-output_offset_right);//-output_offset_right)*ticks_left/ticks_right;
	 
	 }

	ROS_INFO("speedcommand %f, %f",speedcommand_ch1,speedcommand_ch2);
	 //ROS_INFO("speedcommand %d",speedcommand_ch2);
	 
	}

	
};		

int main( int argc, char** argv )
{
	std::string speed_ch1_sub_str,speed_ch2_sub_str,cmd_vel_sub_str;
	std::string odom_str,speed_sub_str;
	
	ros::init(argc, argv, "sdc2130_core");
	ros::NodeHandle n("~");
      
	M_Controller mc;
	
	mc.motor_connected=true;
	 
	if(!n.hasParam("speed_sub")) ROS_WARN("Used default parameter for speed_ch2_sub");
	n.param<std::string>("speed_sub", speed_sub_str, "/cmd_vel");
	n.param<std::string>("port",mc.port,"/dev/ttyACM0"); 
	n.param<bool>("toggle_turn",mc.toggle_turn,false);
	n.param<double>("vel_max",mc.vmax,0.8);
	n.param<int>("max_acceleration",mc.amax,500);
	n.param<int>("max_decceleration",mc.dmax,20000);
	n.param<double>("distance_center_to_wheel",mc.W,0.2);
    n.param<double>("angle_max",mc.angle_max,1.0);
    n.param<double>("output_max",mc.output_max,1000);
    n.param<double>("output_offset_left",mc.output_offset_left,0);
     n.param<double>("output_offset_right",mc.output_offset_right,0);
    n.param<bool>("closed_loop",mc.closed_loop,true);
    n.param<double>("P",mc.P,1);
    n.param<double>("I",mc.I,0);
    n.param<double>("D",mc.D,0);
    n.param<double>("Pang",mc.Pang,1);
    n.param<double>("Iang",mc.Iang,0);
    n.param<double>("Dang",mc.Dang,0);
    n.param<double>("ticks_per_meter_left",mc.ticks_left,138000);
    n.param<double>("ticks_per_meter_right",mc.ticks_right,138000);
      
    n.param<std::string>("odometry_sub",odom_str,"/fmKnowledge/odometry");

	//define a subscriber for the goal commands.

	ros::Subscriber sub_speedcommand_ch1=n.subscribe(speed_sub_str,10,&M_Controller::setSpeed,&mc);
	ros::Subscriber sub_odometry=n.subscribe(odom_str,10,&M_Controller::getOdometry,&mc);
	
	
	mc.pub_encoder_ch1 = n.advertise<msgs::IntStamped>("encoder_ch1", 100);
	mc.pub_encoder_ch2 = n.advertise<msgs::IntStamped>("encoder_ch2", 100);
	mc.pub_motor_speed_ch1 = n.advertise<std_msgs::Int64>("motor_speed_ch1", 150);
	mc.pub_motor_speed_ch2 = n.advertise<std_msgs::Int64>("motor_speed_ch2", 150);
	mc.pub_voltage_ch1 = n.advertise<std_msgs::Float64>("voltage_ch1", 150);
	mc.pub_voltage_ch2 = n.advertise<std_msgs::Float64>("voltage_ch2", 150);
	mc.pub_ampere_ch1 = n.advertise<std_msgs::Float64>("ampere_ch1", 150);
	mc.pub_ampere_ch2 = n.advertise<std_msgs::Float64>("ampere_ch2", 150);
	//connect to motorcontroller	
	mc.init();
	//publish transforms with 50 Hz
	ros::Timer t;
	t = n.createTimer(ros::Duration(0.02), &M_Controller::publish_topics,&mc);
	//use timer with transform frequency
	while (mc.motor_connected) {
	ros::spinOnce();
	}
		
	return 0;

}
