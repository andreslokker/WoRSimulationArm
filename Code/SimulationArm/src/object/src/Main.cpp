#include "Cup.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cup_shape");

    Cup cup;
    cup.initCup();
    cup.publishCup();
    
    ros::Rate loop_rate(120);
    while(ros::ok()) {
        cup.checkForGripper();
        if(cup.isHold()) {
            cup.moveCup();
        }
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}