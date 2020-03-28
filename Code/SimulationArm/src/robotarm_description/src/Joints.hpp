#ifndef JOINTS_HPP
#define JOINTS_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "Message.hpp"
#include <vector>
#include <string>

class Joints {
    public:
        Joints();
        virtual ~Joints();
        void initJoints();
        void move(const std::vector<Message>& position);

        const std::array<std::array<double, 2>, 7>& getRadianRange() const;
    private:
        ros::NodeHandle n;
        ros::Publisher joint_pub;
        tf::TransformBroadcaster broadcaster;
        sensor_msgs::JointState jointState;
        //geometry_msgs::TransformStamped odomTrans;
        std::vector<std::string> armJoints;
        std::array<std::array<double, 2>, 7> radianRange = {{
            {{-M_PI/2, M_PI/2}}, // base
            {{-M_PI/2, M_PI/5}}, // shoulder
            {{-M_PI/2, M_PI/4}}, // Elbow
            {{M_PI/2, -M_PI/2}}, // Wrist
            {{-M_PI/2, M_PI/2}}, // Wrist rotate
            {{0, M_PI/45}}, // gripper left
            {{0, M_PI/45}} // gripper right
        }};
};

#endif