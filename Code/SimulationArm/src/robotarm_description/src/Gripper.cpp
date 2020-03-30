#include "Gripper.hpp"
#include "std_msgs/Bool.h"

#define GRIPPER_LENGTH 0.03
#define GRIPPER_HEIGHT 0.02
#define DIST_CTR_LINE_MARGE 0.05

Gripper::Gripper() : listener(ros::Duration(60)), markerSubscriber(n.subscribe("visualization_marker", 1000, &Gripper::callback, this)), markerCtrPointPublisher(n.advertise<geometry_msgs::PoseStamped>("gripper_ctr_pos", 1000)), markerInGripper(n.advertise<std_msgs::Bool>("marker_in_gripper", 1000)) {

}

Gripper::~Gripper() {

}

void Gripper::callback(const visualization_msgs::Marker::ConstPtr& msg) {
    marker = *msg;
}

geometry_msgs::PoseStamped Gripper::convertPoint(const std::string& oldType, const std::string& newType, double x, double y, double z, double Qx, double Qy, double Qz, double Qw) {
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

    geometry_msgs::PoseStamped transformedGripperPoint; 

    listener.transformPose(newType, gripperPoint, transformedGripperPoint);
    return transformedGripperPoint;
}

void Gripper::checkForCup() {
    
    ros::spinOnce();
    std_msgs::Bool markerInGripperMsg;
    
    // get the 2 gripper points. the right point is converted so we have the points between the gripper arms
    geometry_msgs::PoseStamped rightGripperPoint = convertPoint("gripper_right", "base_link", 0.0, 0.0 + GRIPPER_HEIGHT, 0.0); 
    geometry_msgs::PoseStamped leftGripperPoint = convertPoint("gripper_left", "base_link", 0.0, 0.0, 0.0); // GRIPPER_HEIGHT is used to get the gripper length, since they are the same

    // distance between the 2 gripper points
    double distGripperPoints = sqrt(pow(rightGripperPoint.pose.position.x - leftGripperPoint.pose.position.x, 2) + pow(rightGripperPoint.pose.position.y - leftGripperPoint.pose.position.y, 2) + pow(rightGripperPoint.pose.position.z - leftGripperPoint.pose.position.z, 2));
    
    // distance between the center of the marker and the right gripper position (used to determine if the marker is in the middle of the gripper)
    double distMarkerCenterRight = sqrt(pow(rightGripperPoint.pose.position.x - marker.pose.position.x, 2) + pow(rightGripperPoint.pose.position.y - marker.pose.position.y, 2) + pow(rightGripperPoint.pose.position.z - marker.pose.position.z, 2));
    
    // distance between the center of the marker and the left gripper position (used to determine if the marker is in the middle of the gripper)
    double distMarkerCenterLeft = sqrt(pow(leftGripperPoint.pose.position.x - marker.pose.position.x, 2) + pow(leftGripperPoint.pose.position.y - marker.pose.position.y, 2) + pow(leftGripperPoint.pose.position.z - marker.pose.position.z, 2));

    // distance between the center of the marker and the center of the gripper (used to determine if the gripper is close enough to the cup)
    double distMarkerCtrToCtr = sqrt(pow(distGripperPoints/2, 2) + pow(distMarkerCenterRight/2, 2));

    
    // this if statement is used to check if the cup is in a gripper area.
    if(distGripperPoints <= marker.scale.x && distMarkerCenterRight - distMarkerCenterLeft < DIST_CTR_LINE_MARGE && 
    distMarkerCenterRight - distMarkerCenterLeft > -1 * DIST_CTR_LINE_MARGE && distMarkerCtrToCtr <= GRIPPER_LENGTH &&
    rightGripperPoint.pose.position.z + GRIPPER_HEIGHT <= marker.pose.position.z + marker.scale.z && 
    rightGripperPoint.pose.position.z + GRIPPER_HEIGHT >= marker.pose.position.z - marker.scale.z)
    {
        // we rotate the cup over the pitchlistener
        double pitch = -M_PI/2;
        double roll = 0;
        double yaw = 0;
        
        tf::Quaternion rotatedMarker = tf::createQuaternionFromRPY(roll, pitch, yaw);

        // we take the 0,0,0 + offsets point from the gripper and convert it to base_link
        // We also use a flipped orientation of the cup, since the cup should be flipped in the "gripper_left" frame
        geometry_msgs::PoseStamped baseToGripper = convertPoint("gripper_left", "base_link", 0, 0 - (GRIPPER_HEIGHT/2) - (distGripperPoints/2), 0 + marker.scale.y/2 , rotatedMarker[0], rotatedMarker[1], rotatedMarker[2], rotatedMarker[3]);
        markerCtrPointPublisher.publish(baseToGripper);
        markerInGripperMsg.data = true;
    } else {
        markerInGripperMsg.data = false;
    }

    markerInGripper.publish(markerInGripperMsg);
}