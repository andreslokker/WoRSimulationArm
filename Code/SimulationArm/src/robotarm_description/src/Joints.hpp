/**
 * @file Joints.hpp
 * @author Andre Slokker
 * @brief Joints class
 * @version 0.1
 * @date 2020-03-24
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef JOINTS_HPP
#define JOINTS_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "Message.hpp"
#include "Gripper.hpp"
#include <vector>
#include <string>

/**
 * @brief Class Joints can be used to interact with the robot. 
 * 
 */
class Joints {
    public:
        /**
         * @brief Construct a new Joints object
         * 
         */
        Joints();

        /**
         * @brief Destroy the Joints object
         * 
         */
        virtual ~Joints();

        /**
         * @brief InitJoints sets the start positions of the RobotArm.
         * 
         */
        void initJoints();

        /**
         * @brief This function can be used to move the arm. 
         * 
         * @param commands vector of Messages, each message needs to contain a unique servo id, and a position (in radian)
         */
        void move(const std::vector<Message>& commands);

        /**
         * @brief Get the Radian Range object
         * 
         * @return const std::array<std::array<double, 2>, 7>& radianRange
         */
        const std::array<std::array<double, 2>, 7>& getRadianRange() const;
    private:
        ros::NodeHandle n;
        ros::Publisher joint_pub;
        sensor_msgs::JointState jointState;
        Gripper gripper;
        std::array<std::array<double, 2>, 7> radianRange = {{
            {{-M_PI/2, M_PI/2}}, // base
            {{-M_PI/2, M_PI/5}}, // shoulder
            {{-M_PI/2, M_PI/4}}, // Elbow
            {{M_PI/2, -M_PI/2}}, // Wrist
            {{-M_PI/2, M_PI/2}}, // Wrist rotate
            {{-M_PI/300, M_PI/120}}, // gripper left
            {{M_PI/300, -M_PI/120}} // gripper right
        }};
};

#endif