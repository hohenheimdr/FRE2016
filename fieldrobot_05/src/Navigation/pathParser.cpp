/*
 * parse-yaml.cpp
 *
 *  Created on: Apr 23, 2012
 *      Author: soeni05
 */

#include "pathParser.h"

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
	
	ifstream file;
	file.open(filepath.c_str());
	string output;
	if (file.is_open())
	{
		
		while (!file.eof())
		{
			getline(file,output);
			//file >> output;
			
			int find=output.find(",");
			
			std::string x=output.substr(0,find);
			std::string y=output.substr(find+1,output.length()-2);
			
			if(find>0)
			{
				
			pose.pose.position.x=atof(x.c_str());
			pose.pose.position.y=atof(y.c_str());
			path.push_back(pose);
			}
			
			ROS_INFO("point x= %f, y = %f",pose.pose.position.x,pose.pose.position.y);
		}
		file.close();
		
		ROS_INFO("%s",output.c_str());
	}
	
	
	//file readed to output, now seperate  the information:
	//first get rid of first line

	
/*
	path = new std::vector<geometry_msgs::PoseStamped>;

	std::ifstream fin(filepath.c_str());
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	try{
		for(unsigned i=0;i<doc.size();i++) {
			  geometry_msgs::PoseStamped pose;
			  doc[i] >> pose;
			  path->push_back(pose);
		}
	} catch(YAML::ParserException& e) {
		std::cout << e.what() << "\n";
	}*/
}

