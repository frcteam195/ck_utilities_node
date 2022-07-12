#pragma once
#ifdef ROSCPP_ROS_H
#include <ros/ros.h>
#include <string>

#define STR_PARAM(s) #s
//Parameter helper to turn the variable name into a string, prefixed with the node name
#define CKSP(s) ckgp( STR_PARAM(s) )

std::string ckgp(std::string instr)
{
	std::string retVal = ros::this_node::getName();
	retVal += "/" + instr;
	return retVal;
}
#endif