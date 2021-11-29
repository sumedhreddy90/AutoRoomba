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
