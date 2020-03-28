#ifndef PARSER_HPP
#define PARSER_HPP

#include "Message.hpp"
#include <string>
#include <vector>

/**
 * @brief Enum containing the possible parts of a single command
 * 
 */
enum CommandParts {
    ServoNumber,
    Pwm,
    Time
};

/**
 * @brief Class Parser can be used to parse input SSC-32U messages
 * At the moment it only supports servo commands
 */
class Parser {
    public:
        /**
         * @brief Construct a new Parser object
         * 
         */
        Parser();

        /**
         * @brief Destroy the Parser object
         * 
         */
        virtual ~Parser();
        
        /**
         * @brief Set the Message Value object. It will check if the value satisfies the conditions
         * 
         * @param message The message which will get the parsed value
         * @param parsedValue The parsed value which needs to be inserted in the message
         * @param commandPart The type of the parsed value
         * @return true In case the parsed value is from the right format
         * @return false In case the parsed value is not from the right format
         */
        bool setMessageValue(Message& message, const std::string& parsedValue, CommandParts commandPart);
        
        /**
         * @brief This function will parse a single command like #5 P2000 T1200
         * 
         * @param commands A vector containing previous parsed commands
         * @param command command which needs to be parsed
         * @return true In case the parsed command is from the right format
         * @return false In case the parsed command is not from the right format
         */
        bool parseCommand(std::vector<Message>& commands, const std::string& command);
        
        /**
         * @brief This function will parse input messages. They need to be according 
         * the format written in the lynxmotion datasheet
         * 
         * @param input string to parse
         * @return true In case the parsed input string is from the right format
         * @return false In case the parsed input strirng is not from the right format
         */
        bool parseMessage(const std::string& input);
};

#endif