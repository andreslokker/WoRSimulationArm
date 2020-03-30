/**
 * @file Cup.hpp
 * @author Andre Slokker
 * @brief Cup class
 * @version 0.1
 * @date 2020-03-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef CUP_HPP
#define CUP_HPP

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"

/**
 * @brief The Cup class can be used to add a new cup to Rviz. It also does contain some useful functions to check if it is gripped at the moment
 * and to move the cup.
 */
class Cup {
    public:
        /**
         * @brief Construct a new Cup object
         * 
         */
        Cup();

        /**
         * @brief Destroy the Cup object
         * 
         */
        virtual ~Cup();

        /**
         * @brief Callback function for the markerInGripper subscriber. It will change the color of the marker based on the message
         * Also it will call the gravity function if needed
         * @param msg Message containing a boolean value if the marker is gripped
         */
        void gripperHoldCB(const std_msgs::Bool::ConstPtr& msg);

        /**
         * @brief Callback function for the gripperPos subscriber. It will update the position of the marker and publish it.
         * 
         * @param msg Message containing the position of the marker
         */
        void posUpdateByGripperCB(const geometry_msgs::PoseStamped::ConstPtr& msg);

        /**
         * @brief Function which publishes the marker pose
         * 
         */
        void sendNewPosition();

        /**
         * @brief Initialization functon for the cup (start position and orientation etc.)
         * 
         */
        void initCup();

        /**
         * @brief Function which should be called when you want to update the cup in Rviz
         * 
         */
        void publishCup();

    private:
        ros::NodeHandle n;
        visualization_msgs::Marker marker;
        ros::Publisher markerPublisher;
        ros::Subscriber gripperPos;
        ros::Subscriber markerInGripper;
        bool hold = false;
};


#endif