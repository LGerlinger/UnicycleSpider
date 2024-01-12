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

#define TAILLE_FILTRE_WALL 3
#define WALL_COEFF_A 10
#define WALL_COEFF_B 0.8

#define GOAL_VAL_MAX 33
#define GOAL_VAL_MIN 0

#define TAILLE_FILTRE_TRACE 6
#define TRACE_COEFF_A 20
#define TRACE_COEFF_B 0.4
#define TAILLE_MAX_TRACE 10
#define CALCUL_TRACE_MAP_HZ 1.0


class PlannifNode {
    public:
        PlannifNode();
        void sendStaticPotential();

        
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_staticmap;
        ros::Subscriber sub_map_;
        ros::Subscriber sub_goal_;
        ros::Subscriber sub_odom_;

        std_msgs::UInt8MultiArray initPotential;
        std_msgs::UInt8MultiArray goalPotential;
        std_msgs::UInt8MultiArray tracePotential;
        std_msgs::UInt8MultiArray staticPotential;

        double goal_point[2];

        uint8_t* map_data;
        uint8_t* wallFilter;
        uint8_t* traceFilter; 

        uint8_t wallDelta;
        uint8_t traceDelta;

        std::vector<std::array<float, 2>> pastPosition;
        float actualPosition[2] = {0.0, 0.0};
        
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void initMaps(uint32_t width_, uint32_t height_, float resolution_);
        void calculInitPotential();
        void calculGoalPotential();
        void calculTracePotential(const ros::TimerEvent& event);

        void preCalculateFilter(uint8_t* filter_, uint8_t delta, int taille_filtre, float coeffA, float coeffB);
        void applyFilter(std_msgs::UInt8MultiArray* mapPotential, uint8_t* filter_, int indice, uint8_t delta);
        
        void printMap(std_msgs::UInt8MultiArray& map, std::string nom);
};

#endif // PLANNIF_NODE_HPP
