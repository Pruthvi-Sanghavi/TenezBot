/**BSD 3-Clause License
  *
  *Copyright (c) 2019, Pruthvikumar Sanghavi
  *Copyright (c) 2019, Achal Vyas
  *All rights reserved.
  *
  *Redistribution and use in source and binary forms, with or without
  *modification, are permitted provided that the following conditions are met:
  *
  *1. Redistributions of source code must retain the above copyright notice, this
  *   list of conditions and the following disclaimer.
  *
  *2. Redistributions in binary form must reproduce the above copyright notice,
  *   this list of conditions and the following disclaimer in the documentation
  *   and/or other materials provided with the distribution.
  *
  *3. Neither the name of the copyright holder nor the names of its
  *   contributors may be used to endorse or promote products derived from
  *   this software without specific prior written permission.
  *
  *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
/*
*@copyright Copyright 2019 Pruthvikumar Sanghavi
*                          Achal Vyas
*@file turtlebot.cpp
*@author 	Pruthvikumar Sanghavi, Achal Vyas
*@license BSD 3-Clause
*@brief  Functions definitions for turtlebot class
*/

#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "ros/console.h"
#include "turtlebot.hpp"
#include "tenezbot/pos.h"
#include <geometry_msgs/Twist.h>
#include <math.h>

void turtlebot::dir_sub(tenezbot::pos msg) {
    turtlebot::dir = msg.direction;
}

void turtlebot::mobile_base(geometry_msgs::Twist &move,
 ros::Publisher &movement_pub, ros::Rate &rate) {
    
    if (turtlebot::dir == 3) {
      ros::Time start_turn = ros::Time::now();
      while(ros::Time::now() - start_turn < ros::Duration(2))
        geometry_msgs::Twist move;
        move.linear.x = 0;
        move.angular.z = 0.3;
        movement_pub.publish(move);
        rate.sleep();
        ROS_INFO_STREAM("Searching");
    }


    if (turtlebot::dir == 2) {
      ros::Time start_turn = ros::Time::now();
      while(ros::Time::now() - start_turn < ros::Duration(1.0))
        geometry_msgs::Twist move;
        move.linear.x = 0.1;
        move.angular.z = -0.15;
        movement_pub.publish(move);
        rate.sleep();
        ROS_INFO_STREAM("turning right");
    }

    if (turtlebot::dir == 1) {
      ros::Time start_turn = ros::Time::now();
      while(ros::Time::now() - start_turn < ros::Duration(1.0))
        geometry_msgs::Twist move;
        move.linear.x = 0.5;
        move.angular.z = 0;
        movement_pub.publish(move);
        rate.sleep();
        ROS_INFO_STREAM("Straight");
    }

    if (turtlebot::dir == 0) {
      ros::Time start_turn = ros::Time::now();
      while(ros::Time::now() - start_turn < ros::Duration(1.0))
        geometry_msgs::Twist move;
        move.linear.x = 0.1;
        move.angular.z = 0.15;
        movement_pub.publish(move);
        rate.sleep();
        ROS_INFO_STREAM("turning left");
    }
    
}
