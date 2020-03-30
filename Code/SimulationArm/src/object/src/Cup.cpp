#include "Cup.hpp"
#include "Gravity.hpp"

Cup::Cup() : markerPublisher(n.advertise<visualization_msgs::Marker>("visualization_marker", 1000)), gripperPos(n.subscribe("gripper_ctr_pos", 1000, &Cup::posUpdateByGripperCB, this)), markerInGripper(n.subscribe("marker_in_gripper", 1000, &Cup::gripperHoldCB, this)) {

}

Cup::~Cup() {

}

void Cup::gripperHoldCB(const std_msgs::Bool::ConstPtr& msg) {
    hold = msg->data;
    if(msg->data) {
        // red
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
    } else {
        // green
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;

        // we change the orientation back to base_link
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 1;
        marker.pose.orientation.w = 0;

        if(marker.pose.position.z > 0) {
            Gravity gravity;
            gravity.updatePosGravity(marker, this); // drop the cup with gravity
        }
    }
}

void Cup::posUpdateByGripperCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if(hold) {
        marker.pose.position.x = msg->pose.position.x;
        marker.pose.position.y = msg->pose.position.y;
        marker.pose.position.z = msg->pose.position.z;
        marker.pose.orientation.x = msg->pose.orientation.x;
        marker.pose.orientation.y = msg->pose.orientation.y;
        marker.pose.orientation.z = msg->pose.orientation.z;
        marker.pose.orientation.w = msg->pose.orientation.w;
        publishCup();
    }
}

void Cup::sendNewPosition() {
    markerPublisher.publish(marker);
}

void Cup::initCup() {
    marker.header.frame_id = "base_link";
    marker.ns = "cup";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.25;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 1.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.1;

    // color: red
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}

void Cup::publishCup() {
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    while (markerPublisher.getNumSubscribers() < 1)
    {
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    markerPublisher.publish(marker);
}