#ifndef COMMANDE_NODE_HPP
#define COMMANDE_NODE_HPP


// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/UInt8MultiArray.h"
#include <nav_msgs/Odometry.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
// %EndTag(MSG_HEADER)%

#include <tf2_ros/transform_listener.h>

#include <sstream>
#include <fstream>
#include <iostream>

#include <cstdint>
#include <cmath>

#define SEND_CMD_HZ 3

#define TAILLE_FILTRE 11


class CommandNode {
public:
	CommandNode();
	~CommandNode();
	
	
private :
	ros::NodeHandle nh_;
	// Publisher to :
	// cmd_vel [nav_msgs::OccupancyGrid::ConstPtr]
	ros::Publisher cmd_vel_pub;

	ros::Subscriber sub_map_;
	
	// Suscriber to :
	// stat_pot [std_msgs::UInt8MultiArray]
	ros::Subscriber pot_map_sub;

	// Timer
	ros::Timer timer;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	geometry_msgs::TransformStamped tfRobot2Map;
	geometry_msgs::Pose originMap;
	float resolution = 1;
	uint32_t pixelPosition[2] = {0, 0};
	
	uint8_t* map;
	uint32_t width;
	uint32_t height;
	
	float filtre[TAILLE_FILTRE][TAILLE_FILTRE];

	float posture[3] = {0, 0, 0};
	float gradient[2] = {0, 0};
	
	float coefMomentum = 0.5f;
	float coefCommande = 0.001f;

	void Map2Command(const ros::TimerEvent& event);

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void getPotMapCallback(const std_msgs::UInt8MultiArray& msg); // http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html

	void printMap(const std_msgs::UInt8MultiArray& carte, std::string nom, uint16_t valMax);
};




#endif
