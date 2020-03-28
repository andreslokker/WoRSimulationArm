#include <ros/ros.h>
#include "Joints.hpp"

#include "MessageHandler.hpp"
#include <vector>

std::string message = "<message>";

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    MessageHandler messageHandler;


    //Joints joints;
    ros::Rate loop_rate(120);
    //double degree = M_PI/180;

    //double position = -3.14;
    //double angle = 10;


                // TEMPORARY CODE TO AVOID PARSER
            Message message;
            message.servo = 0;
            message.position = 1500;
            message.time = 3000;
            Message message1;
            message1.servo = 1;
            message1.position = 1500;
            message1.time = 3000;
            Message message2;
            message2.servo = 2;
            message2.position = 2450;
            message2.time = 3000;
            Message message3;
            message3.servo = 3;
            message3.position = 800;
            message3.time = 4000;
            Message message4;
            message4.servo = 4;
            message4.position = 1500;
            message4.time = 5000;
            Message message5;
            message5.servo = 5;
            message5.position = 2000;
            message5.time = 2000;
            std::vector<Message> commands;
            commands.push_back(message);
            commands.push_back(message1);
            commands.push_back(message2);
            commands.push_back(message3);
            commands.push_back(message4);
            commands.push_back(message5);
            messageHandler.addCommand(commands);
            messageHandler.run();
            
            Message message6;
            message6.servo = 0;
            message6.position = 1500;
            message6.time = 3000;
            Message message7;
            message7.servo = 1;
            message7.position = 2500;
            message7.time = 3000;
            Message message8;
            message8.servo = 2;
            message8.position = 2450;
            message8.time = 3000;
            Message message9;
            message9.servo = 3;
            message9.position = 800;
            message9.time = 4000;
            Message message10;
            message10.servo = 4;
            message10.position = 1500;
            message10.time = 5000;
            Message message11;
            message11.servo = 5;
            message11.position = 1900;
            message11.time = 2000;
            std::vector<Message> commands2;
            commands2.push_back(message6);
            commands2.push_back(message7);
            commands2.push_back(message8);
            commands2.push_back(message9);
            commands2.push_back(message10);
            commands2.push_back(message10);
            messageHandler.addCommand(commands2);
            messageHandler.run();

            std::cout << "ik kom hier" << std::endl;

            Message message12;
            message12.servo = 0;
            message12.position = 2500;
            message12.time = 4000;
            Message message13;
            message13.servo = 1;
            message13.position = 1500;
            message13.time = 4000;
            Message message14;
            message14.servo = 2;
            message14.position = 1500;
            message14.time = 4000;
            Message message15;
            message15.servo = 3;
            message15.position = 800;
            message15.time = 4000;
            Message message16;
            message16.servo = 4;
            message16.position = 1500;
            message16.time = 5000;
            Message message17;
            message17.servo = 5;
            message17.position = 1900;
            message17.time = 2000;
            std::vector<Message> commands3;
            commands3.push_back(message12);
            commands3.push_back(message13);
            commands3.push_back(message14);
            commands3.push_back(message15);
            commands3.push_back(message16);
            commands3.push_back(message17);
            messageHandler.addCommand(commands3);
            messageHandler.run();

            
            Message message18;
            message18.servo = 5;
            message18.position = 2500;
            message18.time = 2000;
            std::vector<Message> commands4;
            commands4.push_back(message18);
            messageHandler.addCommand(commands4);
            messageHandler.run();
            // --> end TEMPORARY CODE



    while(true) {


        //std::vector<double> pos;
        //for(int i = 0; i < 7; i++) {
          //  pos.push_back(position);
        //}

        //joints.move(pos, angle);
        //if(position >= 3.14) {
          //  position = -3.14;
        //} else {
          //  position += 0.01;
       // }

        //if(angle >= 180) {
         //   angle = 0;
        //} else {
         //   angle += degree/4;
        //}
        loop_rate.sleep();
    }
    return 0;
}