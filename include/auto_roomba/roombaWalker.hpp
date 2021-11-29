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
 * @file roombaWalker.hpp
 * @author Sumedh Koppula
 * @date 29th Nov 2021
 * @copyright All rights reserved
 * @brief class for obstacle avoidance for roomba vaccum cleaner
 */
#ifndef INCLUDE_ROOMBAWALKER_HPP_
#define INCLUDE_ROOMBAWALKER_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief This class describes a turtle walker.
 */
class RoombaWalker {
 private:
    bool obstacle;
    geometry_msgs::Twist msg;
    ros::Publisher vel;
    ros::Subscriber scan;
    double minimumDistance;
    ros::NodeHandle nodeh;

 public:
    /**
     * @brief Contructor for RoombaWalker class.
     */
    RoombaWalker();

    /**
     * @brief Destructor for RoombaWalker class.
     */
    ~RoombaWalker();

    /**
     * @brief To get obstacle information.
     * @return True/False on status of obstacle detected.
     */
    bool getObstacle();

    /**
     * @brief Callback function for subscriber function.
     * @param[in] msg The message subscribed from laser scan topic.
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /**
     * @brief Resets the Roomba velocities.
     */
    void resetRoomba();

    /**
     * @brief Moves the Roomba in world map.
     */
    void moveRoomba();
};

#endif /* INCLUDE_ROOMBAWALKER_HPP_ */
