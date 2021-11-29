/*
 *Copyright (C) MIT.
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 *
 * @file roombaWalker.cpp
 * @author Sumedh Koppula
 * @date 29th Nov 2021
 * @copyright All rights reserved
 * @brief Logic for obstacle avoidance for roomba vaccum cleaner
 */
#include <iostream>
#include <unistd.h>
#include "../include/auto_roomba/roombaWalker.hpp"

RoombaWalker::RoombaWalker() {
    minimumDistance = 0.5;
    obstacle = false;
    // Publishing velocity into the node.
    vel = nodeh.advertise<geometry_msgs::Twist>("/cmd_vel", 500);
    // Subscribing to the laser scan message.
    scan = nodeh.subscribe <sensor_msgs::LaserScan> ("/scan", 500, \
                                 &RoombaWalker::scanCallback, this);
    resetRoomba();
}

RoombaWalker::~RoombaWalker() {
    resetRoomba();
}

void RoombaWalker::resetRoomba() {
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
}

void RoombaWalker::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    obstacle = false;
    ///  Checking obstace within minimum range.
        if (msg->ranges[0] < minimumDistance) {
            obstacle = true;
            ROS_INFO("Obstacle detected at %f distance!", msg->ranges[0]);
        }
}

void RoombaWalker::moveRoomba() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        //  Rotating roomba upon obstacle detected
        if (getObstacle() == true) {
            resetRoomba();
            msg.linear.x = 0.0;
            msg.angular.z = 0.5;
            //  rotating in opposite direction for level 2 detection.
            if (getObstacle() == true) {
                    resetRoomba();
                    msg.linear.x = 0.0;
                    msg.angular.z = -0.5;
            }
            usleep(1000000);
            obstacle= false;
        //  Moving roomba forward
        } else {
            ROS_INFO("Clear Path, Moving as Planned!");
            msg.linear.x = 0.2;
            msg.angular.z = 0.0;
        }
        // Publish the velocity to the roomba.
        vel.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
