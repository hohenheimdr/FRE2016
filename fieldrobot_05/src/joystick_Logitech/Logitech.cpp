
#include "Logitech.h"

//constructor and deconstructor build...

Logitech::Logitech()
{
//global object  
//define port and other values for joystick
joyDev="/dev/input/js0";
ctrlConfig.maxfwd=100;
ctrlConfig.maxturn=100;
ctrlConfig.fastBut=4;
ctrlConfig.slowFactor=1.0;
//define startvalues of 
speedleft=0;
speedright=0;
update=true;

}

Logitech::~Logitech()
{

}

int Logitech::init()
{
	std::string Dev;
joystick_node_handler.getParam("/motorcontroller/joystick/port",Dev);
	
//Open first serial port
  if ((jDev = open (Dev.c_str(), O_RDWR /*| O_NONBLOCK*/)) == -1) {
    fprintf(stderr,"   Can't open joystick port: %s\n",Dev.c_str());
    joyRunning = -1;
    return -1;
  }  else {
      printf("   Opened joystick on port: %s\n",Dev.c_str());
      joyRunning = 2;
  }

  //Query and print the joystick name
  char name[128];
  if (ioctl(jDev, JSIOCGNAME(sizeof(name)), name) < 0)
        strncpy(name, "Unknown", sizeof(name));
  printf("   Joystick model: %s\n", name);

  //Query and print number of axes and buttons
  char number_of_axes, number_of_buttons;
  ioctl (jDev, JSIOCGAXES, &number_of_axes);
  ioctl (jDev, JSIOCGBUTTONS, &number_of_buttons);
  printf("   Registrered %d axes and %d buttons on joystick\n",number_of_axes,number_of_buttons);
 
	//start data colecting thread 
	boost::thread DataThread(&Logitech::Read_Logitech,this);

	std::cout<<"threads started"<<std::endl;
		
	sleep(1);	
	return RQ_SUCCESS;
}
///////////////////////////////////////////////////////////////////////
int Logitech::Disconnect()
{
	joyRunning = -1;
	//Wait for thread to close
	sleep(1);
return 0;
}
///////////////////////////////////////////////////////////////////////
int Logitech::Read_Logitech()
{
	//Thread has now started
	joyRunning=true;
	int bytes;
    struct js_event jse;
	
    while (joyRunning) {
			update=false;
            //Read a joystick package from dev
           bytes = read(jDev, &jse, sizeof(jse));

            if (bytes == -1) {
                joyRunning = -1;
                printf("   JoyControl: Failed reading from joystick - Exiting!\n");
                init();
                //break;
            } else if (bytes != sizeof(jse)) {
                printf("   JoyControl: Unexpected bytes from joystick:%d\n", bytes);

            } else { //Proper joystick package has been recieved
				
                //Joystick package parser
                jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
                switch(jse.type) {
                    case JS_EVENT_AXIS:
                        if (jse.number < 16) {
                            joyValues.axes[jse.number] = jse.value;
                        }
                        break;

                    case JS_EVENT_BUTTON:
                        if (jse.number < 16) {
                            joyValues.button[jse.number] = jse.value;
                        }
                        break;
                    default:
                        break;
                }
                 update=true;
                 Solve_Speed();
            }
            
        }
	return 0;
}

///////////////////////////////////////////////////////////////////////
int Logitech::Solve_Speed()
{
	int fwd=0;
	int turn=0;
	//solving drive comands for diverential drive with speed left/right
	if (joyValues.button[4])
	{
		fwd=-1000*joyValues.axes[4]/32768;
		turn=1000*joyValues.axes[3]/32768;
		
		if (turn==0)
		{
		speedleft=fwd;
		speedright=fwd;
		}
		if(abs(turn)>0)
		{
			//turn right
			double faktor=turn/1000;
			speedleft=(turn+fwd);
			speedright=-turn+fwd;
					
		}
				
	}else
	{
		speedleft=0;
		speedright=0;
	}
		
	return 0;
}
