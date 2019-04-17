#ifndef UDP_HPP_INCLUDED
#define UDP_HPP_INCLUDED


#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>

#include <iostream>
#include <string>
#include <sstream>


#include "Utils/Utils.hpp"


class UDPsocket
{
    public:
        /// Default
        UDPsocket(const int& port = 5013);
        ~UDPsocket();
        //

        /// Communication functions
        bool sendMessageTo(const uint8_t& msgType, const uint8_t* msg, const unsigned short& msgSize, const char* destination, const char* port);
        std::string read();
        //

        /// Set / get functions
        void incrementTimestamp();

        bool getIpFromHostname(const std::string& hostname, std::string* ip, const int& lastState = 0) const;
        unsigned long getErrorsCount() const;
        //

    protected:
        /// Attributes
        int udpsocket;
        unsigned int timestamp;
        unsigned long errors;
        uint8_t* buffer;
        uint8_t* rcvBuffer;
        //


        /// Protected functions
        int resolveDestination(const char* destination, int family, const char* port, sockaddr_storage* pAddr);
        uint8_t getCRC(const uint8_t* msg, const unsigned short& msgSize) const;
        //
};

#endif // UDP_HPP_INCLUDED
