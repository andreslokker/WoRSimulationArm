#ifndef MESSAGEHANDLER_HPP
#define MESSAGEHANDLER_HPP

#include "Message.hpp"
#include "Joints.hpp"
#include <queue>
#include <vector>
#include <array>

class MessageHandler {
    public:
        MessageHandler();
        virtual ~MessageHandler();
        void addCommand(const std::vector<Message>& command);
        void handleCommand();
        void run();
    private:
        Joints joints;
        std::queue<std::vector<Message>> commands;
};

#endif