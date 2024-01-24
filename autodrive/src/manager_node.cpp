#include "../include/manager_node.hpp"


ManagerNode::ManagerNode(): tfBuffer(), tfListener(tfBuffer){

    pub_activation_ = nh_.advertise<std_msgs::Bool>("rbt_actv", 1000);
    pub_my_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("my_goal", 1000);

    sub_goal_ = nh_.subscribe("move_base_simple/goal", 1, &ManagerNode::goalCallback, this);
    timer = nh_.createTimer(ros::Duration(1.0f/GET_POS_ROBOT_HZ), &ManagerNode::getPosRobot, this);
}

ManagerNode::~ManagerNode(){
    
}


void ManagerNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    my_goal = *msg;
    //Recupérer les coordonées de la map
    goal_point[0] = msg->pose.position.x;
    goal_point[1] = msg->pose.position.y;
    
    // goal_point[0] = (goal_point[0] - originMap.position.x);
    // goal_point[1] = (goal_point[1] - originMap.position.y);
        
    //Frame ID de la map
    // std::cout << "Frame ID: " << msg->header.frame_id << std::endl;

    //Relance si on reçoit un nouveau goal
    std_msgs::Bool stop;
    stop.data = false;
    pub_activation_.publish(stop);
    //Ajouter Delay ?


    //ROS_INFO("MANAGER : Publish nouveau goal");
    pub_my_goal_.publish(my_goal);
    goalAchieved = false;
}

void ManagerNode::getPosRobot(const ros::TimerEvent& event){
    float distance_goal = 0;

    if(!goalAchieved){
        //Recupérer pos robot 
        try {
            tfRobot2Map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            
            actualPosition[0] = tfRobot2Map.transform.translation.x; //- originMap.position.x;
            actualPosition[1] = tfRobot2Map.transform.translation.y; //- originMap.position.y;
            // std::cout << "Frame ID ROBOT : " << tfRobot2Map.header.frame_id << std::endl;
            // std::cout << "childFrame ID  ROBOT : " << tfRobot2Map.child_frame_id << std::endl;
        }catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
        }
        
        //ROS_INFO("MANAGER : X_robot %f et X_goal %f", actualPosition[0], goal_point[0]);
        //ROS_INFO("MANAGER : Y_robot %f et Y_goal %f", actualPosition[1], goal_point[1]);

        //Faire test si le goal est proche du goal
        distance_goal = sqrt(pow(actualPosition[0] - goal_point[0], 2) 
                            + pow(actualPosition[1] - goal_point[1], 2));


        //ROS_INFO("MANAGER : DISTANCE GOAL %f\n\n", distance_goal);
        //Stop les autres nodes si on est suffisament proche du goal
        if(distance_goal < THRESHOLD_DISTANCE_GOAL){
            goalAchieved = true;
            std_msgs::Bool stop;
            stop.data = true;
            //Send l'arret des deux nodes 
            pub_activation_.publish(stop);
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "manager_node");
    ManagerNode node;

    ros::spin();
    return 0;
}
