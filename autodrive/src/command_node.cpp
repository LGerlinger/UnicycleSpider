// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "geometry_msgs/Twist.h"
// %EndTag(MSG_HEADER)%

#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  geometry_msgs::Twist msg;
  while (ros::ok())
  {

    msg.linear.x = 0.0;
    msg.angular.z = 1.0;
    //
    // ROS_INFO("%s", msg.data.c_str());

    cmd_vel_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
