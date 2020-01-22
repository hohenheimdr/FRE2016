/*
 * pathParser.h
 *
 *  Created on: Apr 27, 2012
 *      Author: soeni05
 */

#ifndef PATHPARSER_H_
#define PATHPARSER_H_


#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class pathParser{
public:



	std::vector<geometry_msgs::PoseStamped> path;
	pathParser();
	pathParser(std::string filepath);
	
};



#endif /* PATHPARSER_H_ */
