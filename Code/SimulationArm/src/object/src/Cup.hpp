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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

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
         * @brief This function should be called when the cup is gripped. It will follow the movement of the gripper.
         */
        void moveCup();

        /**
         * @brief This function can convert a point from one frame to a new frame
         * 
         * @param oldType Old frame
         * @param newType New frame to convert the position to
         * @param x X position
         * @param y Y position
         * @param z Z position
         * @param Qx X orientation
         * @param Qy Y orientation
         * @param Qz Z orientation
         * @param Qw W orientation
         * @return geometry_msgs::PoseStamped The point converted to the newType frame
         */
        geometry_msgs::PoseStamped convertPoint(const std::string& oldType, const std::string& newType, double x, double y, double z, double Qx = 0, double Qy = 0, double Qz = 0, double Qw = 1);
        
        /**
         * @brief This function checks if the cup is gripped at the moment. Also when the cup is gripped, but the gripper releases the cup, a gravity function will be called
         * so that the cup drops down.
         */
        void checkForGripper();

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

        /**
         * @brief Boolen which returns if the gripper is hold
         * 
         * @return true In case it is hold
         * @return false In case it is not hold
         */
        bool isHold();
    private:
        ros::NodeHandle n;
        visualization_msgs::Marker marker;
        tf::TransformListener listener;
        ros::Publisher markerPublisher;
        ros::Subscriber jointsSubscriber;
        bool hold = false;
        double distGripper = 0;
};


#endif