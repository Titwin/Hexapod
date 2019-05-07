#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <cstring>
#include <termios.h>

#include "Network.hpp"
#include "GamePad.hpp"
#include "Hexapod.hpp"
#include "bcm2835.h"
#include "Socket/UDP.hpp"
#include "Socket/Serializer.hpp"
#include "Localization/Localization.hpp"
#include"Maths/MathConversion.hpp"

#define UDP_SEND_PORT "5014"
#define UDP_HOSTNAME "Thibault-SED-PC.local"

#define FRAME_TIME 20 //in ms
#define UDP_SCAN_FRAME 50


/// prototypes
void *TTLThreadMain(void* arg);
void openSerialPort(std::string portName);
void closeSerialPort();
void setCPUAffinity(const  pthread_t& thread, const int& cpu, const std::string& threadName);
void initializeEnvironement(Localization& locSystem);
//


/// exit program gently
bool finishProgram = false;
void signalHandler(int sig)
{
    finishProgram = true;
}
//


/// global variables for thread synchronization
Network* NET;
//


/// main program : robot control & planning
int main()
{
    /// Initialization
    signal(SIGINT, &signalHandler);
    setCPUAffinity(pthread_self(), 0, "Main");
    std::cout<<"Hello friend !"<<std::endl;

    if(!bcm2835_init())
    {
        std::cout<<"GPIO headers : wrong initialization"<<std::endl;
        return -1;
    }

    GamePad gamepad;
    openSerialPort("/dev/ttyAMA0");
    Hexapod robot("Arane 2.0");
    Localization localizationSystem("totems.txt");
    initializeEnvironement(localizationSystem);


    UDPsocket UDPcomputer(5013);
    UDPsocket UDPvision(5016);

    std::string computerIP;
    UDPcomputer.getIpFromHostname(UDP_HOSTNAME, &computerIP);
    NET = new Network();
    std::map<Network::NodeType, std::map<uint8_t, Network::Node*> > nodeMap;

    pthread_t TTLThread;
    pthread_create(&TTLThread, NULL, &TTLThreadMain, NULL);
    setCPUAffinity(TTLThread, 1, "TTLThreadMain");

    /// Wait for mapping to finished
    while(!finishProgram)
    {
        auto loopingTime = std::chrono::high_resolution_clock::now();
        NET->getNodeMap(&nodeMap);
        NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE, 0);
        if(nodeMap[Network::NODE_SCS15].size() >= 18)
            break;

        std::cout << "Current mapping : " << std::endl;
        std::cout << "  Motors SCS15 : " << nodeMap[Network::NODE_SCS15].size() << "/18 {";
            for(auto it = nodeMap[Network::NODE_SCS15].begin(); it!= nodeMap[Network::NODE_SCS15].end(); ++it)
                std::cout << (int)it->first << " ";
        std::cout << "}" << std::endl << "  LegBoards : " << nodeMap[Network::NODE_LEGBOARD].size() << "/6 {";
            for(auto it = nodeMap[Network::NODE_LEGBOARD].begin(); it!= nodeMap[Network::NODE_LEGBOARD].end(); ++it)
                std::cout << (int)it->first << " ";
        std::cout << "}" << std::endl;
        gamepad.update();

        while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() < 500);

        //break;
    }


    /// Last initialization
    robot.setTorque(false);
    NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE, 0);
    NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_SPEED, FRAME_TIME);
    NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE_LIMIT, 1023);
    NET->configuration = Network::CONFIG_ACCURATE_DISTANCE;

    std::cout<<"---------------------"<<std::endl;
    std::cout<<"Start looping"<<std::endl;
    std::chrono::time_point<std::chrono::system_clock> loopingTime = std::chrono::high_resolution_clock::now();
    unsigned long long loopCount = 0;

    while(!finishProgram)
    {
        /// begin
        loopingTime = std::chrono::high_resolution_clock::now();

        if(loopCount < 5)
        {
            NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_SPEED, FRAME_TIME);
            NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE_LIMIT, 1023);
        }
        if((loopCount%100) == 0)
            NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_ACTION);

        /// update / synchronize of scs15 nodes
        NET->getNodeMap(&nodeMap);
        bool complete = true;
        uint16_t pos[18];
        for(uint8_t id=2; id<20; id++)
        {
            auto it = nodeMap[Network::NODE_SCS15].find(id);
            if(it != nodeMap[Network::NODE_SCS15].end())
            {
                Network::Scs15* const scs15 = static_cast<Network::Scs15*>(it->second);
                pos[id-2] = scs15->presentPos;
            }
            else
            {
                complete = false;
                break;
            }
        }
        if(complete)
            robot.setMotorAngles((uint8_t*)pos);
        else std::cout<<"missing data for good robot update  " <<nodeMap[Network::NODE_SCS15].size()<<"/18"<<std::endl;
        NET->setSyncNodeAttributes(Network::NODE_SCS15, Network::SCS15_TARGET_POSITION, 18, robot.getMotorsIds(), robot.getGoalMotorAngles());

        /// update / synchronize of legboard nodes
        if((loopCount%UDP_SCAN_FRAME) == 1)
        {
            if(!computerIP.empty())
            {
                NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_RED, 100);
                NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_GREEN, 100);
                NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_BLUE, 100);
            }
            else
            {
                NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_RED, 100);
                NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_GREEN, 0);
                NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_BLUE, 0);
            }
        }

        /// update depending to gamepad
        gamepad.update();
        if(gamepad.connected() && gamepad.instantPressed(GamePad::BACK))
        {
            robot.setTorque(!robot.getTorque());
            NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE, (robot.getTorque()?1:0));
        }
        if(gamepad.instantPressed(GamePad::START))
            robot.setState(Hexapod::INIT);
        MyVector3f translate(gamepad.getExpAxis(GamePad::AXIS_1X), -gamepad.getExpAxis(GamePad::AXIS_1Y), 0);
        if(gamepad.getAxis(GamePad::AXIS_3Y) < 0)
            translate.z = 0.1f;
        else if(gamepad.getAxis(GamePad::AXIS_3Y) > 0)
            translate.z = -0.1f;
        MyVector3f rotate(-gamepad.getExpAxis(GamePad::AXIS_2X), 0, 0);
        robot.animate(FRAME_TIME, 0.3f*translate, 0.01f*rotate);

        /// send informations
        if((loopCount%UDP_SCAN_FRAME) == 0)
            UDPcomputer.getIpFromHostname(UDP_HOSTNAME, &computerIP, (computerIP.empty() ? -1 : 1));
        if(!computerIP.empty())
        {
            std::string s = Serializer::serialize(localizationSystem.getRobotTransform().getOrigin(), "position");
            s = Serializer::serialize(MathConversion::toQuat(localizationSystem.getRobotTransform()), s + ";orientation");
            s = Serializer::serialize(robot.getCorrectedTranslationSpeed(), s + ";velocity");
            s = Serializer::serialize(robot.getCorrectedRotationSpeed(), s + ";angularVelocity");
            UDPcomputer.sendMessageTo(Serializer::SYSTEM, (const uint8_t*)s.c_str(), s.size(), computerIP.c_str(), UDP_SEND_PORT);

            if(!nodeMap.empty() && !nodeMap[Network::NODE_SCS15].empty())
            {
                s = Serializer::serialize(&nodeMap[Network::NODE_SCS15], true);
                UDPcomputer.sendMessageTo(Serializer::MOTORS, (const uint8_t*)s.c_str(), s.size(), computerIP.c_str(), UDP_SEND_PORT);
            }
            if(!nodeMap.empty() && !nodeMap[Network::NODE_LEGBOARD].empty())
            {
                s = Serializer::serialize(&nodeMap[Network::NODE_LEGBOARD], true);
                UDPcomputer.sendMessageTo(Serializer::SLAVES, (const uint8_t*)s.c_str(), s.size(), computerIP.c_str(), UDP_SEND_PORT);
            }
        }

        /// parse received informations
        std::string computerMsg = UDPcomputer.read();
        if(!computerMsg.empty())
        {
            std::cout << Utils::SUCCESS << " : from UDPcomputer" << std::endl;
            std::cout << computerMsg << std::endl;
        }
        localizationSystem.update(UDPvision.read(), FRAME_TIME, robot.getCorrectedTranslationSpeed(), robot.getCorrectedRotationSpeed());


        /// end
        loopCount++;
        UDPcomputer.incrementTimestamp();
        UDPvision.incrementTimestamp();
        //std::cout << "Main : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() << "ms" <<std::endl;
        while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() < FRAME_TIME);
    }

    std::cout<<"---------------------"<<std::endl;
    for(int i=0; i<5; i++)
    {
        loopingTime = std::chrono::high_resolution_clock::now();

        NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE, 0);
        NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_RED, 100);
        NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_GREEN, 0);
        NET->nodeBroadcast(Network::NODE_LEGBOARD, Network::LEGBOARD_BLUE, 0);

        NET->synchonize(false);

        loopCount++;
        while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() < FRAME_TIME);
    }

    pthread_join(TTLThread, NULL);
    closeSerialPort();
    bcm2835_close();

    std::cout<<"End of program. Farewell my friend !"<<std::endl;
    return 0;
}
int toto()
{
    /// Initialization
    signal(SIGINT, &signalHandler);
    setCPUAffinity(pthread_self(), 0, "Main");
    std::cout<<"Hello friend !"<<std::endl;

    if(!bcm2835_init())
    {
        std::cout<<"GPIO headers : wrong initialization"<<std::endl;
        return -1;
    }

    openSerialPort("/dev/ttyAMA0");
    NET = new Network();
    std::map<Network::NodeType, std::map<uint8_t, Network::Node*> > nodeMap;

    pthread_t TTLThread;
    pthread_create(&TTLThread, NULL, &TTLThreadMain, NULL);
    setCPUAffinity(TTLThread, 1, "TTLThreadMain");

    std::chrono::time_point<std::chrono::system_clock> loopingTime = std::chrono::high_resolution_clock::now();

    /// Wait for mapping to finished
    while(!finishProgram)
    {
        loopingTime = std::chrono::high_resolution_clock::now();
        NET->getNodeMap(&nodeMap);
        NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE, 0);
        NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE_LIMIT, 128);

        std::cout << "Current mapping : " << std::endl;
        std::cout << "  Motors SCS15 : " << nodeMap[Network::NODE_SCS15].size() << "/18 {";
            for(auto it = nodeMap[Network::NODE_SCS15].begin(); it!= nodeMap[Network::NODE_SCS15].end(); ++it)
                std::cout << (int)it->first << " ";
        std::cout << "}" << std::endl << "  LegBoards : " << nodeMap[Network::NODE_LEGBOARD].size() << "/6 {";
            for(auto it = nodeMap[Network::NODE_LEGBOARD].begin(); it!= nodeMap[Network::NODE_LEGBOARD].end(); ++it)
                std::cout << (int)it->first << " ";
        std::cout << "}" << std::endl;

        int i=0;
        for(auto it = nodeMap[Network::NODE_SCS15].begin(); it!= nodeMap[Network::NODE_SCS15].end(); ++it, i++)
        {
            Network::Scs15* const scs15 = static_cast<Network::Scs15*>(it->second);
            std::cout<<"    id: "<<(int)it->first<<" position: "<<(int)scs15->presentPos - 512<<std::endl;
            if((i%3)==2)
                std::cout<<std::endl;
        }

        while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() < 500);
    }

    for(int i=0; i<5; i++)
    {
        loopingTime = std::chrono::high_resolution_clock::now();

        NET->nodeBroadcast(Network::NODE_SCS15, Network::SCS15_TORQUE, 0);
        NET->synchonize(false);

        while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() < FRAME_TIME);
    }

    pthread_join(TTLThread, NULL);
    closeSerialPort();
    bcm2835_close();

    std::cout<<"End of program. Farewell my friend !"<<std::endl;
    return 0;
}
//


/// TTL bus communication thread
bool verbose = false;
void* TTLThreadMain(void* arg)
{
    //std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::system_clock> loopingTime = std::chrono::high_resolution_clock::now();
    NET->initialize(verbose);

    while(!finishProgram)
    {
        loopingTime = std::chrono::high_resolution_clock::now();

        NET->synchonize(verbose);
        while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() < FRAME_TIME - 6)
            NET->nodeScan(verbose);
        while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() < FRAME_TIME - 2)
            NET->scheduler();

        /// end
		//std::cout << "TTLbus : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() << "ms" <<std::endl;
        while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - loopingTime).count() < FRAME_TIME);
    }
    return NULL;
}
//

/// Useful functions
void openSerialPort(std::string portName)
{
    std::cout<<"Connecting :  "<<portName;

    struct termios tio;

    memset(&tio,0,sizeof(tio));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8|CREAD|CLOCAL;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;

    TTLbusController::tty_fd = open(portName.c_str(), O_RDWR | O_NONBLOCK);
    cfsetospeed(&tio, B1000000);
    cfsetispeed(&tio, B1000000);

    tcsetattr(TTLbusController::tty_fd, TCSANOW, &tio);
    tcflush(TTLbusController::tty_fd, TCIOFLUSH);

    if(TTLbusController::tty_fd < 0) std::cout<<"\nUnable to open port name"<<std::endl;
    else std::cout<<"   ... OK"<<std::endl;
}
void closeSerialPort(){close(TTLbusController::tty_fd);}
void setCPUAffinity(const pthread_t& thread, const int& cpu, const std::string& threadName)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);

    if(pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) !=0)
        std::cout << "ERROR while changing CPU affinity for " << threadName << " with CPU " << cpu <<std::endl;
    if(pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0)
        std::cout << "ERROR while checking CPU affinity for " << threadName <<std::endl;

    std::cout << "CPU affinity for " << threadName << " :";
    for(int j=0; j<CPU_SETSIZE; j++)
        if (CPU_ISSET(j, &cpuset))
            std::cout << j << ' ';
    std::cout << std::endl;
}

/// Table initialization
void initializeEnvironement(Localization& locSystem)
{
    locSystem.setCameraTransform(MyVector3f(0,7,-7), MathConversion::toQuat(MyMatrix4f::rotation(-90, MyVector3f(0,1,0))), MyVector3f(1,1,1));
    locSystem.setRobotTransform(MyVector3f(0,0,-7), MyQuaternionf::identity());
}

