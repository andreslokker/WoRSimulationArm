#include "Joints.hpp"

#define UPDATE_INTERVAL 18
#define UPDATE_MARGE 0.01

Joints::Joints() : joint_pub(n.advertise<sensor_msgs::JointState>("joint_states", 1000)){
    //armJoints.push_back("base_link");
    //armJoints.push_back("turret");
    //armJoints.push_back("upperarm");
    //armJoints.push_back("wrist");
    //armJoints.push_back("hand");
    //armJoints.push_back("gripper_left");
    //armJoints.push_back("gripper_right");
    initJoints();
}

Joints::~Joints() {

}

void Joints::initJoints() {
    jointState.header.stamp = ros::Time::now();
    jointState.name.resize(7);
    jointState.position.resize(7);
    jointState.name[0] = "base_link2turret";
    jointState.name[1] = "turret2upperarm";
    jointState.name[2] = "upperarm2forearm";
    jointState.name[3] = "forearm2wrist";
    jointState.name[4] = "wrist2hand";
    jointState.name[5] = "gripper_left2hand";
    jointState.name[6] = "gripper_right2hand";

    // update this part --> maybe read pos from urdf?
    for(std::size_t i = 0; i < radianRange.size(); i++) {
        jointState.position[i] = (radianRange.at(i).at(0) + radianRange.at(i).at(1)) / 2;
    }
    jointState.position[1] = 0.01;
    jointState.position[2] = 0.01;
    // -->

    joint_pub.publish(jointState);
}

void Joints::move(const std::vector<Message>& position) {
    ros::Time lastMillis = ros::Time::now();
    bool finished = false;
    std::vector<double> posToUpdate;
    for(std::size_t i = 0; i < position.size(); i++) {
        posToUpdate.push_back((position.at(i).position - jointState.position[position.at(i).servo]) / (position.at(i).time / UPDATE_INTERVAL));
    }

    while(!finished) {
        if(ros::Time::now() > lastMillis + ros::Duration((float) UPDATE_INTERVAL / 1000)) {
            jointState.header.stamp = ros::Time::now();
            for(std::size_t i = 0; i < position.size(); i++) {
                if((jointState.position[position.at(i).servo] + UPDATE_MARGE < position.at(i).position && posToUpdate.at(i) > 0)
                || (jointState.position[position.at(i).servo] + UPDATE_MARGE > position.at(i).position && posToUpdate.at(i) < 0)) {
                    jointState.position[position.at(i).servo] += posToUpdate.at(i);
                }
            }
            jointState.position[6] = jointState.position[5]; // we copy over the left gripper position to the right gripper

            finished = true;
            for(std::size_t i = 0; i < position.size(); i++) {
                if((jointState.position[position.at(i).servo] + UPDATE_MARGE < position.at(i).position && posToUpdate.at(i) > 0)
                || (jointState.position[position.at(i).servo] + UPDATE_MARGE > position.at(i).position && posToUpdate.at(i) < 0)) {
                    finished = false;
                }
            }
            joint_pub.publish(jointState);
            lastMillis = ros::Time::now();
        }
    }
    /*
    for(int i = 0; i < 5; i++) {
        odomTrans.header.frame_id= armJoints.at(i);
        odomTrans.child_frame_id = armJoints.at(i+1);
        odomTrans.header.stamp = ros::Time::now();
        odomTrans.transform.translation.x = 0.0;
        odomTrans.transform.translation.y = 0.0;
        odomTrans.transform.translation.z = 1;
        odomTrans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    }*/

    joint_pub.publish(jointState);
    //broadcaster.sendTransform(odomTrans);
}

const std::array<std::array<double, 2>, 7>& Joints::getRadianRange() const {
    return radianRange;
}