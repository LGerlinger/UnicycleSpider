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
#include <std_msgs/Bool.h>
// %EndTag(MSG_HEADER)%

#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sstream>
#include <fstream>
#include <iostream>

#include <cstdint>
#include <cmath>

#define SEND_CMD_HZ 10

#define TAILLE_FILTRE 11


class CommandNode {
public:
	CommandNode();
	~CommandNode();
	
	
private :
	ros::NodeHandle nh_;
	// Publisher to :
	// cmd_vel [nav_msgs::OccupancyGrid::ConstPtr]
	ros::Publisher pub_cmd_vel_;

	ros::Subscriber sub_map_;
	
	// Suscriber to :
	// stat_pot [std_msgs::UInt8MultiArray]
	ros::Subscriber sub_pot_map_;
	ros::Subscriber sub_activation_;

	// Timer
	ros::Timer timer;
	ros::Timer timer_start;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	geometry_msgs::TransformStamped tfRobot2Map;
	geometry_msgs::Pose originMap;
	float resolution = 1;
	uint32_t pixelPosition[2] = {0, 0};
	
	uint8_t* map = nullptr;
	uint32_t width;
	uint32_t height;
	
	float filtre[TAILLE_FILTRE][TAILLE_FILTRE];

	float posture[3] = {0, 0, 0};
	float gradient[2] = {0, 0};
	
	float coefMomentum = 0.f;
	float coefCmdLin = 0.01f;
	float coefCmdRot = 0.8f;

	bool first_init = true;
	bool stop_ = false;

	void checkMapInit(const ros::TimerEvent& event);
	void Map2Command(const ros::TimerEvent& event);
	void getRobotPos();

	void changeState(const std_msgs::Bool::ConstPtr& stop);
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	void getPotMapCallback(const std_msgs::UInt8MultiArray& msg); // http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html
	void printMap(const uint8_t* carte, std::string nom, uint16_t valMax);
};




#endif
