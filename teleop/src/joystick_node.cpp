/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class Joystick_node{
  public :
    Joystick_node();
  private :
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber joy_sub_;


    int linear_, angular_;
    double l_scale_, a_scale_;

};

  Joystick_node::Joystick_node(): linear_(1), angular_(0), l_scale_(.4), a_scale_(0.6){
    ROS_INFO("Creation ROS joystick_node");

    n.param("axis_linear", linear_, linear_);
    n.param("axis_angular", angular_, angular_);
    n.param("scale_angular", a_scale_, a_scale_);
    n.param("scale_linear", l_scale_, l_scale_);
    
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &Joystick_node::joyCallback, this);
  }

  void Joystick_node::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
  {
    geometry_msgs::Twist twist;
    twist.linear.x = l_scale_ * joy->axes[linear_];

    // ROS_INFO("ON LIS 9A : %f", joy->axes[angular_]);
    twist.angular.z = a_scale_ * joy->axes[angular_];
    cmd_vel_pub.publish(twist);
  }


  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "joystick_node");
    Joystick_node joystick_node;


    
    ros::spin();  

    return 0;
  }
  // %EndTag(FULLTEXT)%
