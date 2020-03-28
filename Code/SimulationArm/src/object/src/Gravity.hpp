/**
 * @file Gravity.hpp
 * @author Andre Slokker
 * @brief Gravity class
 * @version 0.1
 * @date 2020-03-28
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef GRAVITY_HPP
#define GRAVITY_HPP

#include "Cup.hpp"
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

/**
 * @brief Class Gravity can be used to drop a object
 * 
 */
class Gravity {
    public:
        /**
         * @brief Construct a new Gravity object
         * 
         */
        Gravity();

        /**
         * @brief Destroy the Gravity object
         * 
         */
        virtual ~Gravity();

        /**
         * @brief This function can be used to update the marker position with gravity
         * 
         * @param marker The marker which needs to be updated
         * @param cup The cup is used to call the publish function
         */
        void updatePosGravity(visualization_msgs::Marker& marker, Cup* cup);
    private:
        ros::Time lastMillis;
};

#endif