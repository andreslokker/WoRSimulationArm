#include "Gravity.hpp"

#define UPDATE_INTERVAL 60
#define GRAVITY 0.098

Gravity::Gravity() : lastMillis(ros::Time::now()) {

}

Gravity::~Gravity() {

}

void Gravity::updatePosGravity(visualization_msgs::Marker& marker, Cup* cup) {
    double currentAccel = 0;
    double currentInterval = 1;
    while(marker.pose.position.z > 0) {
        if(ros::Time::now() >= lastMillis + ros::Duration(1.0/UPDATE_INTERVAL)) {
            // each interval moment we update the speed with the accel divided over interval periods + the total accel divided by the interval
            double speedToAdd = GRAVITY / UPDATE_INTERVAL + (currentAccel / UPDATE_INTERVAL);
            
            marker.pose.position.z -= speedToAdd;
            currentInterval++;
            cup->publishCup();
            if (currentInterval == UPDATE_INTERVAL) {
                currentInterval = 1;
                currentAccel += GRAVITY; // when all interval moments have been, we update the currentAccel and reset the current interval
            }
            lastMillis = ros::Time::now();
        }
    }
}