#ifndef MANAGER_NODE_HPP
#define MANAGER_NODE_HPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
// #include "geometry_msgs/Twist.h"


#include <sstream>
#include <fstream>
#include <iostream>

#define GET_POS_ROBOT_HZ 10
#define THRESHOLD_DISTANCE_GOAL 0.2 //en m 

class ManagerNode {
    public:
        ManagerNode();
        ~ManagerNode();
        
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_activation_;
        ros::Publisher pub_my_goal_;

        ros::Subscriber sub_goal_;
        ros::Timer timer;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        geometry_msgs::TransformStamped tfRobot2Map;

        geometry_msgs::PoseStamped my_goal;
        double goal_point[2] =  {1000.0, 1000.0};;
        float actualPosition[2] = {0.0, 0.0};
        bool goalAchieved = true;

        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void getPosRobot(const ros::TimerEvent& event);

};

#endif // MANAGER_NODE_HPP
