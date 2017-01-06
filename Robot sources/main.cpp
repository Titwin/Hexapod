#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "sources/SerialProtocol.hpp"
#include "sources/Server.hpp"

#define FRAME_TIME 20
#define END_TIME 4

bool finishProgram = false;
void signalHandler(int sig)
{
    finishProgram = true;
}

int main()
{
    std::cout<<"Hello friend!"<<std::endl;
    SerialProtocol uart;
    Server robotServer(5012);
    signal(SIGINT, &signalHandler);

    std::cout<<"---------------------"<<std::endl;
    std::cout<<"Start looping"<<std::endl;
    clock_t start = clock();
    clock_t looping;

//////
    SerialProtocol::Message msgtest;
    int count = 0;
//////

    while(!finishProgram)
    {

        count++;
        count %= 3;
        looping = clock();

        //
        if(robotServer.getClientCount()>0)
        {
            msgtest.first = RPI_INST_READ | RPI_TARGET_SCS15;
            msgtest.second.clear();
            msgtest.second.push_back(RPI_POSITION);
            uart.send(msgtest);

            switch(count)
            {
                case 0:
                    msgtest.first = RPI_INST_READ | RPI_TARGET_SCS15;
                    msgtest.second.clear();
                    msgtest.second.push_back(RPI_FAIL_NODE);
                    uart.send(msgtest);
                    break;

                case 1:
                    msgtest.first = RPI_INST_READ | RPI_TARGET_SCS15;
                    msgtest.second.clear();
                    msgtest.second.push_back(RPI_TORQUE);
                    uart.send(msgtest);
                    break;

                case 2:
                    msgtest.first = RPI_INST_READ | RPI_TARGET_SCS15;
                    msgtest.second.clear();
                    msgtest.second.push_back(RPI_TEMPERATURE);
                    uart.send(msgtest);
                    break;

                default: break;
            }
        }


        //
        std::string msgData;
        while(uart.validMessageCount())
        {
            SerialProtocol::Message msg = uart.getMessage();
            switch(msg.first)
            {
                case RPI_INST_ACK: break;

                case RPI_INST_READ|RPI_TARGET_CONTROL:
                    msgData.clear();
                    for(unsigned int i = 1; i<msg.second.size(); i++)
                    {
                        std::ostringstream oss;
                        oss << (int)msg.second[i];
                        msgData += std::string(oss.str()) + ' ';
                    }
                    robotServer.pushMessage("Arduino_control ",msgData);
                    break;

                case RPI_INST_READ|RPI_TARGET_SCS15:
                    switch(msg.second[0])
                    {
                        case RPI_FAIL_NODE:
                            msgData.clear();
                            for(unsigned int i = 1; i<msg.second.size(); i++)
                            {
                                std::ostringstream oss;
                                oss << (int)msg.second[i];
                                msgData += std::string(oss.str()) + ' ';
                            }
                            robotServer.pushMessage("SCS15_fail ",msgData);
                            break;

                        case RPI_POSITION:
                            msgData.clear();
                            for(unsigned int i = 1; i<msg.second.size(); i+=2)
                            {
                                std::ostringstream oss;
                                oss << (int)(msg.second[i+1]*256 + msg.second[i]);
                                msgData += std::string(oss.str()) + ' ';
                            }
                            robotServer.pushMessage("SCS15_position ",msgData);
                            break;

                        case RPI_TORQUE:
                            msgData.clear();
                            for(unsigned int i = 1; i<msg.second.size(); i+=2)
                            {
                                std::ostringstream oss;
                                oss << (int)(msg.second[i+1]*256 + msg.second[i]);
                                msgData += std::string(oss.str()) + ' ';
                            }
                            robotServer.pushMessage("SCS15_torque ",msgData);
                            break;

                        case RPI_TEMPERATURE:
                            msgData.clear();
                            for(unsigned int i = 1; i<msg.second.size(); i++)
                            {
                                std::ostringstream oss;
                                oss << (int)(msg.second[i+1]*256 + msg.second[i]);
                                msgData += std::string(oss.str()) + ' ';
                            }
                            robotServer.pushMessage("SCS15_temperature ",msgData);
                            break;

                        default:
                            std::cout<<"unknown target 2";
                            uart.debug(msg);
                            break;
                    }
                    break;

                case RPI_INST_READ|RPI_TARGET_ANALOG:
                    msgData.clear();
                    for(unsigned int i = 1; i<msg.second.size(); i++)
                    {
                        std::ostringstream oss;
                        oss << (int)msg.second[i];
                        msgData += std::string(oss.str()) + ' ';
                    }
                    robotServer.pushMessage("Analog_in ",msgData);
                    break;

                default:
                    std::cout<<"unknown msg"<<std::endl;
                    uart.debug(msg);
                    break;
            }
        }
///
        robotServer.update();
///
        while((double)(clock() - looping)/1000 < FRAME_TIME - END_TIME);
        uart.update(END_TIME);
        while((double)(clock() - looping)/1000 < FRAME_TIME);
        uint64_t timeclock = (double)(clock() - start)/1000;
		//std::cout << timeclock << std::endl;
		if(timeclock >= 300000) break;
    }

    std::cout<<"---------------------"<<std::endl;
    std::cout<<"End of program. Farewell my friend !"<<std::endl;
    return 0;
}

