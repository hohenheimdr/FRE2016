#include "SDC2130.h"

#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <linux/usbdevice_fs.h>

SDC2130::SDC2130()
{
	//clear start values
	encoder_ch1=0;
	encoder_ch2=0;
	motor_speed_ch1=0;
	motor_speed_ch2=0;
	moving=false;
	reset=false;
	task_running=false;	
}

SDC2130::~SDC2130()
{
device.Disconnect();

}

int SDC2130::init()
{
	//read the server data and connect to Motorcontrollers with launch file parameter
	sdc2130_node_handler.getParam("/motorcontroller/sdc2130/port",DevicePort);
	sdc2130_node_handler.getParam("/motorcontroller/sdc2130/max_acceleration",max_acceleration);
	sdc2130_node_handler.getParam("/motorcontroller/sdc2130/max_decceleration",max_decceleration);
	int status = device.Connect(DevicePort);
	//try to connect while status not ok
	while (status!=RQ_SUCCESS)
	{
		//create a usb reset..
		
		
		sleep(1);
		status = device.Connect(DevicePort);
	}
	cout<<"start configurations..."<<endl;
	//set configurations for Motorcontroller
	device.SetConfig(_ECHOF, 1);
	//set closed loop encoder mode
	device.SetConfig(_MMOD, 1,0);
	device.SetConfig(_MMOD, 2,0);
	//set PID Controller..not used right now
	//set motor Acceleration
	device.SetConfig(_MAC, 1, max_acceleration);
	device.SetConfig(_MAC, 2, max_acceleration);
	//set deceleration
	device.SetConfig(_MDEC, 1, max_decceleration);
	device.SetConfig(_MDEC, 2, max_decceleration);
	//set Encoder configuration set channel 1 and 2 to feedback
	device.SetConfig(_EMOD,1,18);
	device.SetConfig(_EMOD,2,34);
	//Wait 10 ms before sending another command to device
	sleepms(10);
	//reset Motorspeed
	device.SetCommand(_GO,1,0);
	device.SetCommand(_GO,2,0);		
	printf("finished Initalisation...\n");
	
	return 0;
}

int SDC2130::init(std::string DevicePort,int max_acceleration, int max_decceleration)
{
	//read the server data and connect to Motorcontrollers with launch file parameter
	int status = device.Connect(DevicePort);
	//try to connect while status not ok
	while (status!=RQ_SUCCESS)
	{
		sleep(1);
		status = device.Connect(DevicePort);
	}
	cout<<"start configurations..."<<endl;
	//set configurations for Motorcontroller
	device.SetConfig(_ECHOF, 1);
	//set closed loop encoder mode
	device.SetConfig(_MMOD, 1,0);
	device.SetConfig(_MMOD, 2,0);
	//set PID Controller..not used right now
	//set motor Acceleration
	device.SetConfig(_MAC, 1, max_acceleration);
	device.SetConfig(_MAC, 2, max_acceleration);
	//set deceleration
	device.SetConfig(_MDEC, 1, max_decceleration);
	device.SetConfig(_MDEC, 2, max_decceleration);
	//set Encoder configuration set channel 1 and 2 to feedback
	device.SetConfig(_EMOD,1,18);
	device.SetConfig(_EMOD,2,34);
	//Wait 10 ms before sending another command to device
	sleepms(10);
	//reset Motorspeed
	device.SetCommand(_GO,1,0);
	device.SetCommand(_GO,2,0);		
	printf("finished Initalisation...\n");
	
	return 0;
}
////////////////////////////////////////////////////////////////////////
bool SDC2130::get_motorstatus(void)
{
	//return if motor is moving or not
	return moving;
}
///////////////////////////////////////////////////////////////////////
int SDC2130::resetMotors()
{
	reset=true;
	device.SetCommand(_GO,1,0);
	device.SetCommand(_GO,2,0);	
		
}
///////////////////////////////////////////////////////////////////////////////
int SDC2130::readEncoders()
{
		int status1,status2;
		int dummy1,dummy2;
		status1=device.GetValue(_ABCNTR,1,dummy1);
		status2=device.GetValue(_ABCNTR,2,dummy2);
		if (status1==RQ_SUCCESS)
		{
			encoder_ch1=dummy1;
		}else
		{
			return 1;
		}
		if (status2==RQ_SUCCESS)
		{
			encoder_ch2=-dummy2;
		}else
		{
			return 1;
		}
		//ROS_INFO("Encoder value: %d  %d  [%d     %d]", status1, status2 , encoder_ch1, encoder_ch2);
	return 0;
}
///////////////////////////////////////////////////////////////////////////////
int SDC2130::readMotors()
{
	int status1,status2;
		int dummy1,dummy2;
		status1=device.GetValue(_RELSPEED,1,dummy1);
		status2=device.GetValue(_RELSPEED,2,dummy2);
		if (status1==RQ_SUCCESS)
		{
			motor_speed_ch1=dummy1;
		}else
		{
			return 1;
		}
		if (status2==RQ_SUCCESS)
		{
			motor_speed_ch2=-dummy2;
		}else
		{
			return 1;
		}
			
	return 0;
}
///////////////////////////////////////////////////////////////////////////////
int SDC2130::readVoltage()
{
	int status1,status2;
		int dummy1,dummy2;
		status1=device.GetValue(_VOLTS,1,dummy1);
		status2=device.GetValue(_VOLTS,2,dummy2);
		if (status1==RQ_SUCCESS)
		{
			voltage_ch1=dummy1;
		}else
		{
			return 1;
		}
		if (status2==RQ_SUCCESS)
		{
			voltage_ch2=dummy2;
		}else
		{
			return 1;
		}
	
		//ROS_INFO("Voltage value: %d  %d  [%d     %d]", status1, status2 , voltage_ch1, voltage_ch2);
		
	return 0;
}
///////////////////////////////////////////////////////////////////////////////
int SDC2130::readAmpere()
{
		int status1,status2;
		int dummy1,dummy2;
		status1=device.GetValue(_MOTAMPS,1,dummy1);
		status2=device.GetValue(_MOTAMPS,2,dummy2);
		if (status1==RQ_SUCCESS)
		{
			ampere_ch1=dummy1;
		}else
		{
			return 1;
		}
		if (status2==RQ_SUCCESS)
		{
			ampere_ch2=dummy2;
		}else
		{
			return 1;
		}
	
			
	return 0;
}
////////////////////////////////////////////////////////////////////////////
int SDC2130::setSpeed(int MotorNr, int speed){
	//set command corresponding to neccesary speedvalue of Motor
	//readEncoders();
	int ret=0;
	if(MotorNr==1||MotorNr==2)
	{
	ret=device.SetCommand(_GO,MotorNr,speed);
	}
	return ret;

}
////////////////////////////////////////////////////////////////////////////
