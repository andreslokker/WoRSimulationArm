/**
 * @file MessageHandler.hpp
 * @author Andre Slokker
 * @brief MessageHander class
 * @version 0.1
 * @date 2020-03-23
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef MESSAGEHANDLER_HPP
#define MESSAGEHANDLER_HPP

#include "Message.hpp"
#include "Joints.hpp"
#include <queue>
#include <vector>
#include <array>

/**
 * @brief The MessageHandler class handles all the parsed messages and calls the move function from the Joints class.
 * 
 */
class MessageHandler {
    public:
        /**
         * @brief Construct a new Message Handler object
         * 
         */
        MessageHandler();

        /**
         * @brief Destroy the Message Handler object
         * 
         */
        virtual ~MessageHandler();

        /**
         * @brief This function can be used to add a new command to the queue
         * 
         * @param command A command with a parsed message. The message should contain a servo, position and time
         */
        void addCommand(const std::vector<Message>& command);

        /**
         * @brief This function handles the first command in the queue. It will call the move function in the Joints class
         * Furthermore it will convert the PWM to radians
         */
        void handleCommand();

        /**
         * @brief Function which should be called to automate the process of this class
         * 
         */
        void run();
    private:
        Joints joints;
        std::queue<std::vector<Message>> commands;
};

#endif