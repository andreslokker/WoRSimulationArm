/**
 * @file Gripper.hpp
 * @author Andre Slokker
 * @brief Gripper class
 * @version 0.1
 * @date 2020-03-30
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef GRIPPER_HPP
#define GRIPPER_HPP

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_listener.h"
#include <string>

/**
 * @brief Class which can be used to control the gripper. It can be used to detect if the gripper is in the area of the cup
 * 
 */
class Gripper {
    public:
        /**
         * @brief Construct a new Gripper object
         * 
         */
        Gripper();

        /**
         * @brief Destroy the Gripper object
         * 
         */
        virtual ~Gripper();

        /**
         * @brief Callback function for updating the marker pose
         * 
         * @param msg Message containing the marker pose
         */
        void callback(const visualization_msgs::Marker::ConstPtr& msg);

        /**
         * @brief This function can convert a pose from one frame to a new frame
         * 
         * @param oldType Old frame
         * @param newType New frame to convert the pose to
         * @param x X position
         * @param y Y position
         * @param z Z position
         * @param Qx X orientation
         * @param Qy Y orientation
         * @param Qz Z orientation
         * @param Qw W orientation
         * @return geometry_msgs::PoseStamped The pose converted to the newType frame
         */
        geometry_msgs::PoseStamped convertPoint(const std::string& oldType, const std::string& newType, double x, double y, double z, double Qx = 0, double Qy = 0, double Qz = 0, double Qw = 1);
        
        /**
         * @brief This function checks if the gripper is in the area of the cup. In case it does, it publishes a message that it holds the cup. 
         * It will also publish a message containing the a pose for the cup.
         */
        void checkForCup();
    private:
        ros::NodeHandle n;
        ros::Subscriber markerSubscriber;
        ros::Publisher markerCtrPointPublisher;
        ros::Publisher markerInGripper;
        tf::TransformListener listener;
        visualization_msgs::Marker marker;
};

#endif