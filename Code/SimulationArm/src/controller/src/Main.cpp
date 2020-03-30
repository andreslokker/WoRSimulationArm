/**
 * @file Main.cpp
 * @author Andre Slokker
 * @brief This main class is used to send positions to the robotarm.
 * These messages use the SSC32U protocol
 * @version 0.1
 * @date 2020-03-28
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "ros/ros.h"
#include "robotarm_description/SSC32UPosition.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<robotarm_description::SSC32UPosition>("SSC32UPosition", 1000);

    ros::Duration(1).sleep();

    while(pub.getNumSubscribers() < 1) {
        // wait on the robotarm
    }
    
    // these messages will be send to the robotarm
    robotarm_description::SSC32UPosition msg;
    msg.SSC32UPosition = "#0 P1500 T3000 #1 P1500 T3000 #2 P2450 T3000 #3 P800 T4000"
                        " #4 P1500 T5000 #5 P2000 T2000 \r\n";
    pub.publish(msg);

    ros::Duration(1).sleep();

    robotarm_description::SSC32UPosition msg2;
    msg2.SSC32UPosition = "#1 P2500 T3000 #2 P2500 T1000 #5 P1500 \r\n";
    pub.publish(msg2);

    ros::Duration(1).sleep();

    robotarm_description::SSC32UPosition msg3;
    msg3.SSC32UPosition = "#0 P2500 T4000 #1 P1500 T4000 #2 P1500 T4000 \r\n";
    pub.publish(msg3);

    ros::Duration(1).sleep();

    robotarm_description::SSC32UPosition msg4;
    msg4.SSC32UPosition = "#5 P1000 T2000 \r\n";
    pub.publish(msg4);

    return 0;
}