#include "Cup.hpp"
#include "Gravity.hpp"
#include <geometry_msgs/PointStamped.h>
#include <cmath>

#define GRIPPER_LENGTH 0.03
#define GRIPPER_HEIGHT 0.02
#define DIST_CTR_LINE_MARGE 0.05


Cup::Cup() : listener(ros::Duration(60)), markerPublisher(n.advertise<visualization_msgs::Marker>("visualization_marker", 1000)) {
    listener.waitForTransform("base_link", "base_link", ros::Time::now(), ros::Duration(5.0)); // dirty trick to check if the robotarm has been initialized
}

Cup::~Cup() {

}

void Cup::moveCup() {
    double pitch = -M_PI/2;
    double roll = 0;
    double yaw = 0;
    
    tf::Quaternion rotatedMarker = tf::createQuaternionFromRPY(roll, pitch, yaw);

    geometry_msgs::PoseStamped baseToGripper = convertPoint("gripper_left", "base_link", 0, 0 + ((GRIPPER_HEIGHT/2 + distGripper)/2), 0 + marker.scale.y/2 , rotatedMarker[0], rotatedMarker[1], rotatedMarker[2], rotatedMarker[3]);
    marker.pose.position.x = baseToGripper.pose.position.x;
    marker.pose.position.y = baseToGripper.pose.position.y;
    marker.pose.position.z = baseToGripper.pose.position.z;
    marker.pose.orientation.x = baseToGripper.pose.orientation.x;
    marker.pose.orientation.y = baseToGripper.pose.orientation.y;
    marker.pose.orientation.z = baseToGripper.pose.orientation.z;
    marker.pose.orientation.w = baseToGripper.pose.orientation.w;
    publishCup();                         
}

geometry_msgs::PoseStamped Cup::convertPoint(const std::string& oldType, const std::string& newType, double x, double y, double z, double Qx, double Qy, double Qz, double Qw) {
    geometry_msgs::PoseStamped gripperPoint;
    gripperPoint.header.stamp = ros::Time();
    gripperPoint.header.frame_id = oldType;
    gripperPoint.pose.position.x = x;
    gripperPoint.pose.position.y = y;
    gripperPoint.pose.position.z = z;
    gripperPoint.pose.orientation.x = Qx;
    gripperPoint.pose.orientation.y = Qy;
    gripperPoint.pose.orientation.z = Qz;
    gripperPoint.pose.orientation.w = Qw;

    // we convert the zero point
    geometry_msgs::PoseStamped transformedGripperPoint; 
    listener.waitForTransform(oldType, newType, ros::Time::now(), ros::Duration(3.0));
    listener.transformPose(newType, gripperPoint, transformedGripperPoint);
    return transformedGripperPoint;
}

void Cup::checkForGripper() {
    // get the 2 gripper points. the right point is converted so we have the points between the gripper arms
    geometry_msgs::PoseStamped rightGripperPoint = convertPoint("gripper_right", "base_link", 0.0, 0.0, 0.0); // GRIPPER_HEIGHT == GRIPPER_LENGTH
    geometry_msgs::PoseStamped leftGripperPoint = convertPoint("gripper_left", "base_link", 0.0, 0.0 + GRIPPER_HEIGHT, 0.0);

    double distGripperPoints = sqrt(pow(rightGripperPoint.pose.position.x - leftGripperPoint.pose.position.x, 2) + pow(rightGripperPoint.pose.position.y - leftGripperPoint.pose.position.y, 2) + pow(rightGripperPoint.pose.position.z - leftGripperPoint.pose.position.z, 2));
    double distMarkerCenterRight = sqrt(pow(rightGripperPoint.pose.position.x - marker.pose.position.x, 2) + pow(rightGripperPoint.pose.position.y - marker.pose.position.y, 2) + pow(rightGripperPoint.pose.position.z - marker.pose.position.z, 2));
    double distMarkerCenterLeft = sqrt(pow(leftGripperPoint.pose.position.x - marker.pose.position.x, 2) + pow(leftGripperPoint.pose.position.y - marker.pose.position.y, 2) + pow(leftGripperPoint.pose.position.z - marker.pose.position.z, 2));

    double distMarkerCtrToCtr = sqrt(pow(distGripperPoints/2, 2) + pow(distMarkerCenterRight/2, 2));

    // we check if the cup is in the area of the gripper
    if(distGripperPoints <= marker.scale.x * 2 && distMarkerCenterRight - distMarkerCenterLeft < DIST_CTR_LINE_MARGE && 
    distMarkerCenterRight - distMarkerCenterLeft > -1 * DIST_CTR_LINE_MARGE && distMarkerCtrToCtr <= GRIPPER_LENGTH &&
    rightGripperPoint.pose.position.z + GRIPPER_HEIGHT <= marker.pose.position.z + marker.scale.z && 
    rightGripperPoint.pose.position.z + GRIPPER_HEIGHT >= marker.pose.position.z - marker.scale.z)
    {
        if(!hold) {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            distGripper = distMarkerCtrToCtr;
            publishCup();
        }
        hold = true; 
    } else if(hold) {
        Gravity gravity;
        hold = false;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 1;
        marker.pose.orientation.w = 0;
        publishCup();
        gravity.updatePosGravity(marker, this);
    }
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

bool Cup::isHold() {
    return hold;
}