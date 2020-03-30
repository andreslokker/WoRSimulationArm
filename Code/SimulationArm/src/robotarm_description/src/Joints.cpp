#include "Joints.hpp"

#define UPDATE_INTERVAL 30
#define UPDATE_MARGE 0.01

Joints::Joints() : joint_pub(n.advertise<sensor_msgs::JointState>("joint_states", 1000)) {
    initJoints();
}

Joints::~Joints() {

}

void Joints::initJoints() {
    ros::Duration(1).sleep(); // we wait 1 second to wait on other applications
    jointState.name.resize(7);
    jointState.position.resize(7);
    jointState.name[0] = "base_link2turret";
    jointState.name[1] = "turret2upperarm";
    jointState.name[2] = "upperarm2forearm";
    jointState.name[3] = "forearm2wrist";
    jointState.name[4] = "wrist2hand";
    jointState.name[5] = "gripper_left2hand";
    jointState.name[6] = "gripper_right2hand";

    //these are the starting positions of the arm.
    // future: maybe read these values from the arm URDF file
    for(std::size_t i = 0; i < radianRange.size(); i++) {
        jointState.position[i] = 0;
    }

    jointState.header.stamp = ros::Time::now();
    joint_pub.publish(jointState);
}

void Joints::move(const std::vector<Message>& commands) {
    ros::Time lastMillis = ros::Time::now();
    bool finished = false;
    std::vector<double> posToUpdate;
    for(std::size_t i = 0; i < commands.size(); i++) {
        // we calculate for each joint the update which has to be done in every time interval
        posToUpdate.push_back((commands.at(i).position - jointState.position[commands.at(i).servo]) / (commands.at(i).time / UPDATE_INTERVAL));
    }

    while(!finished) {
        if(ros::Time::now() > lastMillis + ros::Duration((float) UPDATE_INTERVAL / 1000)) {
            jointState.header.stamp = ros::Time::now();
            finished = true;
            // we update the positions
            for(std::size_t i = 0; i < commands.size(); i++) {
                if((jointState.position[commands.at(i).servo] + UPDATE_MARGE < commands.at(i).position && posToUpdate.at(i) > 0)
                || (jointState.position[commands.at(i).servo] + UPDATE_MARGE > commands.at(i).position && posToUpdate.at(i) < 0)) {
                    jointState.position[commands.at(i).servo] += posToUpdate.at(i);
                    if((jointState.position[commands.at(i).servo] + UPDATE_MARGE < commands.at(i).position && posToUpdate.at(i) > 0)
                    || (jointState.position[commands.at(i).servo] + UPDATE_MARGE > commands.at(i).position && posToUpdate.at(i) < 0)) {
                        finished = false;
                    }
                }
            }
            jointState.position[6] = jointState.position[5]; // we copy over the left gripper position to the right gripper
            joint_pub.publish(jointState);
            gripper.checkForCup();
            lastMillis = ros::Time::now();
        }
    }
}

const std::array<std::array<double, 2>, 7>& Joints::getRadianRange() const {
    return radianRange;
}