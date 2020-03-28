#include "Parser.hpp"

Parser::Parser() {

}

Parser::~Parser() {

}

bool Parser::setMessageValue(Message& message, const std::string& parsedValue, CommandParts commandPart) {
    if(commandPart == ServoNumber) {
        uint16_t value = std::stoi(parsedValue);
        if(value >= 0 && value <= 5) { // servo number
            message.servo = value;
            return true;
        }
    } else if (commandPart == Pwm) {
        double value = std::stod(parsedValue, nullptr);
        if((int16_t) value >= 500 && (uint16_t) value <= 2500) { // pwm position
            message.position = value;
            return true;
        }
    } else if (commandPart == Time) {
        uint16_t value = std::stoi(parsedValue);
        if(value >= 0) {
            message.time = value; // time
            return true;
        }
    }
    return false;
}

bool Parser::parseCommand(std::vector<Message>& commands, const std::string& command) {
    Message message = {};
    message.time = 1000;
    CommandParts part = ServoNumber;
    std::string parsed;
    for(std::string::const_iterator pos = command.begin(); pos != command.end(); pos++) {
        if(*pos == 'P') {
            if(!setMessageValue(message, parsed, part))
                return false;
            parsed.clear();
            part = Pwm;
        } else if (*pos == 'T') {
            if(!setMessageValue(message, parsed, part))
                return false;
            parsed.clear();
            part = Time;
        } else if (*pos != '#' && isdigit(*pos)) {
            parsed += *pos;
        } else if (*pos != '#' && *pos != ' ' && *pos != '\r' && *pos != '\n') {
            return false;
        }
    }

    if(!parsed.empty()) {
        if(!setMessageValue(message, parsed, part))
            return false;
    }

    commands.push_back(message);
    return true;
}

bool Parser::parseMessage(const std::string& input, MessageHandler& messageHandler) {
    std::vector<Message> commands;
    if(input.length() > 2 && input.at(input.length()-1) == '\n' && input.at(input.length()-2) == '\r') {
        std::size_t start = 0;
        std::size_t end = 0;
        while((start = input.find("#", end)) != std::string::npos) {
            end = input.find("#", start+1); // we look for the last character
            if(end != std::string::npos) {
                end--; // we remove the # sign
            } else {
                end = input.length()-1;
            }
            std::string stringToParse = input.substr(start, end - start);
            if(!parseCommand(commands, stringToParse)) {
                return false;
            }
            end = start+1;
        }
        messageHandler.addCommand(commands);
    } else {
        return false;
    }
    return true;
}