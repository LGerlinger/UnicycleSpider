#ifndef PLANNIF_NODE_HPP
#define PLANNIF_NODE_HPP

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

//Hyperparamétres Filtres
#define TRACE_COEFF_A 1
#define WALL_COEFF_A 1

#define TAILLE_FILTRE_WALL 23 // PAS DE FILTRE DE TAILLE PAIRE !!
#define TAILLE_FILTRE_TRACE 19

#define WALL_MULT 15
#define TRACE_MULT 200

#define WALL_COEFF_B 0.01
#define TRACE_COEFF_B 0.09

//Autres 
#define GOAL_VAL_MAX 85
#define GOAL_VAL_MIN 0
#define OCCUPANCYGRID_VAL_MAX 100

#define TAILLE_MAX_TRACE 10
#define CALCUL_TRACE_MAP_HZ 1.0
#define SEND_MAP_HZ 1.5



class PlannifNode {
    public:
        PlannifNode();
        ~PlannifNode();

    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_staticmap;
        ros::Subscriber sub_activation_;
        ros::Subscriber sub_map_;
        ros::Subscriber sub_goal_;
        ros::Timer timer;
        ros::Timer timer2;
        ros::Timer timer_goal;
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        std_msgs::UInt8MultiArray initPotential;
        std_msgs::UInt8MultiArray goalPotential;
        std_msgs::UInt8MultiArray tracePotential;
        std_msgs::UInt8MultiArray staticPotential;

        bool first_init = true;

        double goal_point[2];

        geometry_msgs::TransformStamped tfRobot2Map;
        geometry_msgs::Pose originMap;
        float resolution = 1;

        uint8_t* map_data = nullptr;
        float wallFilter[TAILLE_FILTRE_WALL*TAILLE_FILTRE_WALL];
        float traceFilter[TAILLE_FILTRE_TRACE*TAILLE_FILTRE_TRACE]; 

        uint8_t wallDelta = 0;
        uint8_t traceDelta = 0;

        std::vector<std::array<float, 2>> pastPosition;
        float actualPosition[2] = {0.0, 0.0};
        
        void changeState(const std_msgs::Bool::ConstPtr& stop);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void checkMapInit4Goal(const ros::TimerEvent& event);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void initMaps(uint32_t width_, uint32_t height_);
        void calculInitPotential();
        void calculGoalPotential();
        void calculTracePotential(const ros::TimerEvent& event);
        void sendStaticPotential(const ros::TimerEvent& event);

        void preCalculateFilter(float* filter_, uint8_t delta, int taille_filtre, float mult);
        void applyFilter(std_msgs::UInt8MultiArray* mapPotential, float* filter_, int indice, uint8_t delta, int coef);
        void applyConvolution(uint8_t* mapData, uint16_t width, float* filter_, uint64_t indice, uint8_t delta, uint8_t& ptToChange);
        
        void addPotentialToStatic(std_msgs::UInt8MultiArray& mapPotential, int coeff);
        void printMap(std_msgs::UInt8MultiArray& map, std::string nom, uint16_t valMax);
};

#endif // PLANNIF_NODE_HPP
