#include "MessageHandler.hpp"

#define MIN_PWM_POS 500
#define MAX_PWM_POS 2500

MessageHandler::MessageHandler() {

}

MessageHandler::~MessageHandler() {

}

void MessageHandler::addCommand(const std::vector<Message>& command) {
    commands.push(command);
}

void MessageHandler::handleCommand() {
    for(std::size_t i = 0; i < commands.front().size(); i++) {
        // we convert the PWM values to radian values
        commands.front().at(i).convertPwmToRadian(MIN_PWM_POS, MAX_PWM_POS, joints.getRadianRange().at(commands.front().at(i).servo).at(0), joints.getRadianRange().at(commands.front().at(i).servo).at(1));
    }
    joints.move(commands.front());
}

void MessageHandler::run() {
    if(!commands.empty()) {
        std::cout << "not empty" << std::endl;
        handleCommand();
        commands.pop();
    }
}