#ifndef GRAVITY_HPP
#define GRAVITY_HPP

#include "Cup.hpp"
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

class Gravity {
    public:
        Gravity();
        virtual ~Gravity();
        void updatePosGravity(visualization_msgs::Marker& marker, Cup* cup);
    private:
        ros::Time lastMillis;
};

#endif