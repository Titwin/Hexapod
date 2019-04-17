#include "UDP.hpp"

#include <string.h>
#include <stdlib.h>
#include <iomanip>

#define UDP_BUFFERSIZE 50000

using namespace Utils;


/// Default
UDPsocket::UDPsocket(const int& port) : timestamp(0), errors(0)
{
	buffer = new uint8_t[UDP_BUFFERSIZE];
	
    //  create socket
    udpsocket = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in address;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);
    address.sin_family = AF_INET;
    if (bind(udpsocket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
        std::cout << ERROR << " : UDPsocket : opening UDP socket fail" << std::endl;
}
UDPsocket::~UDPsocket()
{
    close(udpsocket);
	delete[] buffer;
}
//


/// Communication functions
bool UDPsocket::sendMessageTo(const uint8_t& msgType, const uint8_t* msg, const unsigned short& msgSize, const char* destination, const char* port)
{
    //  compute destination
    sockaddr_storage addrDest = {};
    if (resolveDestination(destination, AF_INET, port, &addrDest) != 0)
    {
        std::cout << ERROR << " : UDPsocket : destination address configuration fail" << std::endl;
        return false;
    }

    //  compute message
	std::ostringstream sendingBuffer;
    sendingBuffer << (uint8_t)0xFF << (uint8_t)0xFF << std::setw(8) << std::setfill('0') << std::hex << timestamp << std::setw(5)<< msgSize;
    sendingBuffer << std::resetiosflags(std::ios::showbase) << (uint8_t)msgType;
	
	memcpy(buffer, sendingBuffer.str().c_str(), sendingBuffer.tellp());
	memcpy(buffer + 16, msg, msgSize);
	buffer[16 + msgSize] = getCRC(buffer, 16 + msgSize);
		
    //  send message
    if (sendto(udpsocket, buffer, 17 + msgSize, 0, reinterpret_cast<const sockaddr*>(&addrDest), sizeof(addrDest)) < 0)
    {
        switch (errno)
        {
            case EACCES:       std::cout << ERROR << " : UDPsocket : access denied" << std::endl; break;
            case EBADF:        std::cout << ERROR << " : UDPsocket : invalid descriptor" << std::endl; break;
            case ECONNRESET:   std::cout << ERROR << " : UDPsocket : connection reset" << std::endl; break;
            case EDESTADDRREQ: std::cout << ERROR << " : UDPsocket : no address set" << std::endl; break;
            case EFAULT:       std::cout << ERROR << " : UDPsocket : invalid address" << std::endl; break;
            case EINTR:        std::cout << ERROR << " : UDPsocket : interrupt occur" << std::endl; break;
            case EINVAL:       std::cout << ERROR << " : UDPsocket : invalid argument" << std::endl; break;

            case EISCONN:    std::cout << ERROR << " : UDPsocket : ?EISCONN?" << std::endl; break;
            case EMSGSIZE:   std::cout << ERROR << " : UDPsocket : message size problem (" << 17 + msgSize << "o)" << std::endl; break;
            case ENOBUFS:    std::cout << ERROR << " : UDPsocket : network congestion" << std::endl; break;
            case ENOMEM:     std::cout << ERROR << " : UDPsocket : no memory available" << std::endl; break;
            case ENOTCONN:   std::cout << ERROR << " : UDPsocket : socket not connected" << std::endl; break;
            case ENOTSOCK:   std::cout << ERROR << " : UDPsocket : argument is not a socket" << std::endl; break;
            case EOPNOTSUPP: std::cout << ERROR << " : UDPsocket : flags not supported" << std::endl; break;

            case EPIPE: std::cout << ERROR << " : UDPsocket : ?EPIPE?" << std::endl; break;
            default: break;
        }
        errors++;
        return false;
    }
    else
    {
        return true;
    }
}
//

/// Set / get functions
void UDPsocket::incrementTimestamp() { timestamp++; }

bool UDPsocket::getIpFromHostname(const std::string& hostname, std::string* ip, const int& lastState) const
{
    *ip = "";
    hostent* host = gethostbyname(hostname.c_str());
    if (host == NULL)
    {
        if(lastState != -1)
            std::cout << ERROR << " : UDPsocket : host unavailable" << std::endl;
        return false;
    }

    in_addr* address = (in_addr*)host->h_addr;
    *ip = inet_ntoa(*address);
    if(lastState != 1)
        std::cout << SUCCESS << " : UDPsocket : host " << hostname << " found at " << *ip << std::endl;

    return true;
}
unsigned long UDPsocket::getErrorsCount() const { return errors; }
//


/// Protected functions
int UDPsocket::resolveDestination(const char* destination, int family, const char* port, sockaddr_storage* pAddr)
{
    int result;
    addrinfo* result_list = NULL;
    addrinfo hints = {};
    hints.ai_family = family;
    hints.ai_socktype = SOCK_DGRAM;
    result = getaddrinfo(destination, port, &hints, &result_list);
    if (result == 0)
    {
        memcpy(pAddr, result_list->ai_addr, result_list->ai_addrlen);
        freeaddrinfo(result_list);
    }

    return result;
}
uint8_t UDPsocket::getCRC(const uint8_t* msg, const unsigned short& msgSize) const
{
    uint8_t crc = 0;
    for(unsigned short i=0; i< msgSize; i++)
        crc += msg[i];
    return crc;
}
//
