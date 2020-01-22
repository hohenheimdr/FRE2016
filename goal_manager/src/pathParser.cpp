/*
 * autor: 	David
 * date:	25.1.16
 * Path parser for the easyest goal manager possible.
 * Input reads a txt file, float numbers separated by ","
 * x,y,z,waittime, (roll), (pitch), (yaw)
 * 
 * it publishes the result in a 
 * Path parser for reading the goal positions of a machine using 3 coordinates
 * comments can be included with # 
 * this will skip the whole line.
 *
 * todos:
 *  * include a option to also read in the poses, when needed...
 */

#include "pathParser.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;

pathParser::pathParser()
{
}

pathParser::pathParser(std::string filepath){
	ROS_INFO("reading file %s",filepath.c_str());
	//create later reader for yaml file...
	
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x=0;
	pose.pose.position.y=0;
	pose.pose.position.z=0;
	
	ifstream file;
	file.open(filepath.c_str());
	string output;
	if (file.is_open())
	{
		
		while (!file.eof())
		{
			try{
				getline(file,output);
				string test=output;
				//check if line include a comment symbol... when yes, skip line
				if(test.find("#")==-1)
				{
					int find=output.find(",");
					
					std::string x=output.substr(0,find);
					output=output.substr(find+1,output.length()-2);
					
					find=output.find(",");
					std::string y=output.substr(0,find);
					output=output.substr(find+1,output.length()-2);
					
					find=output.find(",");
					std::string z=output.substr(0,find);
					output=output.substr(find+1,output.length()-2);
					
					find=output.find(",");
					std::string time=output.substr(0,find);
					output=output.substr(find+1,output.length()-2);
					
					find=output.find(",");
					std::string roll=output.substr(0,find);
					output=output.substr(find+1,output.length()-2);
					
					find=output.find(",");
					std::string pitch=output.substr(0,find);
					std::string yaw=output.substr(find+1,output.length()-2);
					
					if(find>0)
					{
						try{
						pose.pose.position.x=atof(x.c_str());
						pose.pose.position.y=atof(y.c_str());
						pose.pose.position.z=atof(z.c_str());
						
						pose.pose.orientation.x=atof(roll.c_str());
						pose.pose.orientation.y=atof(pitch.c_str());
						pose.pose.orientation.z=atof(yaw.c_str());
						geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
						pose.pose.orientation=odom_quat;
						
						}catch(...)
						{
							ROS_INFO("error parsing angles in line...");
						}
						//write duration of waypoint to header...
						ros::Time header_time(atof(time.c_str()));
						pose.header.stamp=header_time;
						path.push_back(pose);
					}
					float t=pose.header.stamp.toSec();
							
					printf("point x= %.2f, y = %.2f z= %.2f, waittime= %.2f, roll=%.2f, pitch=%.2f, yaw=%.2f \n",
					pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,t,pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
				}
			}//end try
			catch(...)
			{
				ROS_INFO("parsing error");
			}
		}//ebd while
		file.close();
		
		ROS_INFO("%s",output.c_str());
	}
	
}

