#ifndef __SDC2130_H_
#define __SDC2130_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include "ros/time.h"

#include <iostream>
#include <sstream>
#include <cmath>
#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

class SDC2130
{
private:
//create object for handling of the motorcontroller
RoboteqDevice device;

ros::NodeHandle sdc2130_node_handler;
//global start settings
std::string DevicePort;
int max_acceleration,max_decceleration,ticks_per_meter;
	
protected:
	//motordistance and angle for thread
	double turn_angle;
	double turn_drive;
	double moving_distance;
	double maxspeed;
	//check if robot is moving at the moment
	bool moving;
	
	bool reset;
	//break variable for thread task (SDC2130_task())
	bool task_running;
	
	double get_distance(int tickstart, int tickstop, int MotNr);
	
	double get_turnangle(double  distleft, double distright);
	
	
	//thread handling functions
	int turn();
	int turn_driving();
	int drive_fwd();

public:

	int encoder_ch1;
	int encoder_ch2;
	int motor_speed_ch1;
	int motor_speed_ch2;
	int voltage_ch1;
	int voltage_ch2;
	int ampere_ch1;
	int ampere_ch2;

	//constructors
	SDC2130();
	~SDC2130();
	//intitalisation
	int init();
	int init(std::string, int, int);
	//external functions for thread handling
	int set_turnangle(double turn);
	int set_turndrive(double turn);
	int set_movingdistance(double dist);
	int set_maxspeed(double speed);

	double get_turnangle();
	double get_movingdistance();
	bool get_motorstatus(void);
	//drive for defined distance	
	int drive_fwd(double distance,double speed);
	
	//get motorstatus
	bool running();
	
	int turn(double degree);
	//read values of Encoders
	int readEncoders();
	//readmotorspeed
	int readMotors();
	//get out channel specific voltage
	int readVoltage();
	//get out channel specific ampere
	int readAmpere();
	
	int resetMotors();
	
	//sendspeed command in range of 0..1000
	int setSpeed(RoboteqDevice &rtq,int MotorNr, int speed);
	int setSpeed(int MotorNr, int speed);
	
	void SDC2130_task();
	
	
};//end class

#endif
