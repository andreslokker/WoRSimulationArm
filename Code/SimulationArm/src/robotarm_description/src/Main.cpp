#include <ros/ros.h>
#include <boost/bind.hpp>
#include "Joints.hpp"
#include "Parser.hpp"
#include "MessageHandler.hpp"
#include "robotarm_description/SSC32UPosition.h"
#include <vector>
#include <iostream>

void callback(const robotarm_description::SSC32UPosition::ConstPtr& msg, MessageHandler* messageHandler, Parser* parser) {
    parser->parseMessage(msg->SSC32UPosition, *messageHandler);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle n;
    ros::Duration(1).sleep(); // wait on other applications to start

    Parser parser;
    MessageHandler messageHandler;

    ros::Subscriber sub = n.subscribe<robotarm_description::SSC32UPosition>("SSC32UPosition", 1000, boost::bind(callback, _1, &messageHandler, &parser));

    ros::Rate loopRate(60);

    while(ros::ok()) {
        messageHandler.run();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}