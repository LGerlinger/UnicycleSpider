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

#define TAILLE_FILTRE_WALL 39 //23 // PAS DE FILTRE DE TAILLE PAIRE !!
#define WALL_MULT 10
#define WALL_COEFF_B 0.01


#define CALCUL_TRACE_MAP_HZ 0.5f //4.0
#define TRACE_DELAY 2 //Nombre de valeur de décalage
#define TRACE_DECAY 5
#define TAILLE_MAX_TRACE 20.0
#define TAILLE_FILTRE_TRACE 29
#define TRACE_MULT 500

//Autres 
#define GOAL_VAL_MAX 85
#define GOAL_VAL_MIN 0
#define OCCUPANCYGRID_VAL_MAX 100

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

        ros::Timer trace_calcul_timer;
        ros::Timer send_static_timer;
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;

        std_msgs::UInt8MultiArray initPotential;
        std_msgs::UInt8MultiArray goalPotential;
        std_msgs::UInt8MultiArray tracePotential;
        std_msgs::UInt8MultiArray staticPotential;

        bool first_init = true;

        double og_goal_point[2] = {INFINITY, INFINITY};
        double goal_point[2] = {INFINITY, INFINITY};

        geometry_msgs::TransformStamped tfRobot2Map;
        geometry_msgs::Pose originMap;
        float resolution = 1;

        uint8_t* map_data = nullptr;
        float wallFilter[TAILLE_FILTRE_WALL*TAILLE_FILTRE_WALL];
        float traceFilter[TAILLE_FILTRE_TRACE*TAILLE_FILTRE_TRACE]; 

        uint8_t wallDelta = 0;
        uint8_t traceDelta = 0;
        
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
        void calculInitPotential();
        uint8_t goalMode = 1;
        void calculGoalPotential();
        void calculGoalPotential1();
        void calculTracePotential(const ros::TimerEvent& event);
        void applyTracePotential();
        void sendStaticPotential(const ros::TimerEvent& event);

        void preCalculateFilter(float* filter_, uint8_t delta, int taille_filtre, float mult);
        void applyFilter(std_msgs::UInt8MultiArray* mapPotential, float* filter_, int indice, uint8_t delta, float coef);
        void applyConvolution(uint8_t* mapData, uint16_t width, float* filter_, uint64_t indice, uint8_t delta, uint8_t& ptToChange);
        
        void addPotentialToStatic(std_msgs::UInt8MultiArray& mapPotential, int coeff);
        void printMap(std_msgs::UInt8MultiArray& map, std::string nom, uint16_t valMax);
};

#endif // PLANNIF_NODE_HPP
