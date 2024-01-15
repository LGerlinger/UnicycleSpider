#ifndef PLANNIF_NODE_HPP
#define PLANNIF_NODE_HPP

#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

#include <sstream>
#include <fstream>
#include <iostream>

#define TAILLE_FILTRE_WALL 11
#define WALL_COEFF_A 1
#define WALL_COEFF_B 0.1
#define WALL_MULT 4

#define GOAL_VAL_MAX 85
#define GOAL_VAL_MIN 0
#define OCCUPANCYGRID_VAL_MAX 100

#define TAILLE_FILTRE_TRACE 7 // PAS DE FILTRE DE TAILLE PAIRE !!
#define TRACE_COEFF_A 1
#define TRACE_COEFF_B 0.4
#define TRACE_MULT 100
#define TAILLE_MAX_TRACE 10
#define CALCUL_TRACE_MAP_HZ 1.0
#define SEND_MAP_HZ 1.5

#define MAP_OFFSET_X 0
#define MAP_OFFSET_Y 0


class PlannifNode {
    public:
        PlannifNode();
        ~PlannifNode();

        
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_staticmap;
        ros::Subscriber sub_map_;
        ros::Subscriber sub_goal_;
        ros::Subscriber sub_odom_;
        ros::Timer timer;
        ros::Timer timer2;

        std_msgs::UInt8MultiArray initPotential;
        std_msgs::UInt8MultiArray goalPotential;
        std_msgs::UInt8MultiArray tracePotential;
        std_msgs::UInt8MultiArray staticPotential;

        double goal_point[2];

        uint8_t* map_data = nullptr;
        float wallFilter[TAILLE_FILTRE_WALL*TAILLE_FILTRE_WALL];
        float traceFilter[TAILLE_FILTRE_TRACE*TAILLE_FILTRE_TRACE]; 

        uint8_t wallDelta = 0;
        uint8_t traceDelta = 0;

        std::vector<std::array<float, 2>> pastPosition;
        float actualPosition[2] = {0.0, 0.0};
        
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void initMaps(uint32_t width_, uint32_t height_, float resolution_);
        void calculInitPotential();
        void calculGoalPotential();
        void calculTracePotential(const ros::TimerEvent& event);
        void sendStaticPotential(const ros::TimerEvent& event);

        void preCalculateFilter(float* filter_, uint8_t delta, int taille_filtre, float coeffA, float coeffB, float mult);
        void applyFilter(std_msgs::UInt8MultiArray* mapPotential, float* filter_, int indice, uint8_t delta, int coef);
        
        void addPotentialToStatic(std_msgs::UInt8MultiArray& mapPotential, int coeff);
        void printMap(std_msgs::UInt8MultiArray& map, std::string nom, uint16_t valMax);
};

#endif // PLANNIF_NODE_HPP
