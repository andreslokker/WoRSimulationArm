#ifndef CUP_HPP
#define CUP_HPP

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

class Cup {
    public:
        Cup();
        virtual ~Cup();
        void moveCup();
        geometry_msgs::PoseStamped convertPoint(const std::string& oldType, const std::string& newType, double x, double y, double z, double Qx = 0, double Qy = 0, double Qz = 0, double Qw = 1);
        void checkForGripper();
        void initCup();
        void publishCup();

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