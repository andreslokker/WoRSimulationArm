#ifndef MESSAGE_HPP
#define MESSAGE_HPP

#include <cstdint>
#include <iostream>

/**
 * @brief The struct message keeps track of commands
 * 
 */
struct Message {
    uint8_t servo; // servo number
    double position; // position in PWM or Radians, can be converted with convertPwmToRadian
    uint16_t time; // time to move servo

    /**
     * @brief This function can be used to map the PWM position to radian postions
     * 
     * @param minPwm Minimum allowed PWM position
     * @param maxPwm Maximum allowed PWM position
     * @param minRadian Minumum allowed Radian position
     * @param maxRadian Maximum allowed Radian position
     */
    void convertPwmToRadian(uint16_t minPwm, uint16_t maxPwm, double minRadian, double maxRadian) {
        position = (position - minPwm) / (maxPwm - minPwm) * (maxRadian - minRadian) + minRadian;
    }
};

#endif