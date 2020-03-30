#include "Cup.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cup_shape");

    Cup cup;

    ros::Duration(1).sleep(); // wait for other applications to start

    cup.initCup();
    cup.publishCup();
    
    ros::Rate loop_rate(60);
    while(ros::ok()) {
        cup.sendNewPosition();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}