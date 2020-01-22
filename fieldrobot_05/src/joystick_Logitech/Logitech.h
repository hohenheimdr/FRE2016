

#ifndef __LOGITECH_H_
#define __LOGITECH_H_

#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <arpa/inet.h>
#include <expat.h>
#include <math.h>
#include <linux/joystick.h>
#include <iostream>
#include <cmath>
////////////////////////

#include "ErrorCodes.h"

#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>



class Logitech
{
private:

	
protected:

/********* Joystick definitions  *********/

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

/******** Global variables *************/
int iAxes, iButtons, iOverride, iEstop;
int jDev = 0;  //File descriptors
char* joyDev;


char toggleState = 0, manOverride = 0;



//Structure to hold control configuration
struct controlConfig {
	int maxfwd;
	int maxturn;
	int deadmin;
	int deadmax;
    int fastBut;
	char control;
	double scaleleft;  // for differential drive
	double scaleright;
        double slowFactor;
};
struct controlConfig ctrlConfig;

public:
	ros::NodeHandle joystick_node_handler;

//Structure to hold joystick values
struct jVal {
	int button[16];
	int axes[16];
};
struct jVal joyValues;

	
	int speedleft,speedright;
	//set to true if speedleft or speed right was updated
	bool update;
	bool joyRunning;
	
	//constructors
	Logitech();
	~Logitech();
	
	//global variables:
	
	//intitalisation
	int init();
	
	int Disconnect();
	
	int Read_Logitech();
	
	int Solve_Speed();
	

	//do some processing
	
};//end class

#endif


