#ifndef PLANNIF_NODE_V2_HPP
#define PLANNIF_NODE_V2_HPP

#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>

#include <sstream>
#include <fstream>
#include <iostream>

#include <cmath>

#define TAILLE_FILTRE_GAUSS 23 //23 // PAS DE FILTRE DE TAILLE PAIRE !!
#define WALL_MULT 1

//Autres 
#define GOAL_VAL_MAX 200 // < 255
#define OCCUPANCYGRID_VAL_MAX 100

#define SEND_MAP_HZ 1.5



class PlannifNode_V2 {
	public:
		PlannifNode_V2();
		~PlannifNode_V2();

	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_staticmap;
		ros::Subscriber sub_activation_;
		ros::Subscriber sub_map_;
		ros::Subscriber sub_goal_;

		ros::Timer send_static_timer;
		
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener;

		std_msgs::UInt8MultiArray staticPotential;

		bool first_init = true;

		double og_goal_point[2] = {INFINITY, INFINITY};
		double goal_point[2] = {INFINITY, INFINITY};

		geometry_msgs::TransformStamped tfRobot2Map;
		geometry_msgs::Pose originMap;
		float resolution = 1;

		uint8_t* map_data = nullptr;
		uint8_t* mapData_temp = nullptr; // pour l'ouverture
		float gaussianFilter[TAILLE_FILTRE_GAUSS*TAILLE_FILTRE_GAUSS];
		uint8_t gaussianDelta = 0;
		
		// Pour goalMap : carte de potentiel noyée vers l'objectif
		float* goalMap = nullptr;
		uint8_t goalMapRes = 4;
		uint32_t* ptsToChange = nullptr; // Tableau de pair d'entiers
		uint64_t ptsToChangeSize = 0;

		std::vector<std::array<float, 2>> pastPositionBrut;
		std::vector<std::array<float, 2>> pastPosition;
		float actualPosition[2] = {0.0, 0.0};
		
		void changeState(const std_msgs::Bool::ConstPtr& stop);
		void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void goalOffset();
		void checkMapInit4Goal(const ros::TimerEvent& event);
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void initMaps(uint32_t width_, uint32_t height_);

		void calculGoalMap();
		void map2Goal(uint32_t height, uint32_t width);
		void rechLarg(uint32_t height, uint32_t width);
		void normalise(uint32_t height, uint32_t width);
		
		void sendStaticPotential(const std_msgs::UInt8MultiArray& msg);

		void preCalculateFilter(float* filter_, uint8_t delta, int taille_filtre, float mult);
		void applyFilter(std_msgs::UInt8MultiArray* mapPotential, float* filter_, int indice, uint8_t delta, float coef);
		void applyConvolution(uint8_t* mapData, uint32_t height, uint32_t width, float* filter_, uint8_t delta);

		void fermeture(uint8_t* mapData, uint32_t width, uint32_t height, uint8_t taille);
		
		void printMap(std_msgs::UInt8MultiArray& map, std::string nom, uint16_t valMax);
		void printMap(uint8_t* map, uint32_t height, uint32_t width, std::string nom, uint16_t valMax);
};

#endif // PLANNIF_NODE_V2_HPP
