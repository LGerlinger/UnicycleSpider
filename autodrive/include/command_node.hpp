#ifndef COMMANDE_NODE_HPP
#define COMMANDE_NODE_HPP


// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8MultiArray.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
// %EndTag(MSG_HEADER)%

#include <sstream>
#include <cstdint>
#include <cmath>


class CommandNode {
public:
	CommandNode();
	
	void Map2Command();
	
	void getMapCallback(const std_msgs::UInt8MultiArray& msg); // http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html
	void getOdomCallback(const nav_msgs::Odometry& msg); // http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
	
private :
	ros::NodeHandle n;
	// Publisher to :
	// cmd_vel [geometry_msgs::Twist]
	ros::Publisher cmd_vel_pub;
	
	// Suscriber to :
	// stat_pot [std_msgs::UInt8MultiArray]
	ros::Subscriber map_sub;
	
	// odom [nav_msgs::Odometry]
	ros::Subscriber odom_sub;
	
	uint8_t* map;
	float resolution = 0;
	uint32_t width;
	uint32_t height;
	
	float filtre[3][3] = {
	{0.707f,	1,	0.707f},
	{		1,		0,		 1},
	{0.707f,	1,	0.707f},
	};
	uint8_t tailleFiltre = 3; // doit être impaire merci >:(
	float posture[3] = {0, 0, 0};
	float gradient[2];
	
	float coefMomentum = 0.9f;
	
	float coefCommande = 1; 
};




#endif
