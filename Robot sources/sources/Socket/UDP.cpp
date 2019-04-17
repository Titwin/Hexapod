#include "UDP.hpp"

#include <string.h>
#include <stdlib.h>
#include <iomanip>

#define UDP_SEND_BUFFERSIZE 50000
#define UDP_RCV_BUFFERSIZE 1000
#define HEADER_SIZE 16

using namespace Utils;


/// Default
UDPsocket::UDPsocket(const int& port) : timestamp(0), errors(0)
{
    //  create socket
    buffer = new uint8_t[UDP_SEND_BUFFERSIZE];
    rcvBuffer = new uint8_t[UDP_RCV_BUFFERSIZE];
    udpsocket = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in address;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);
    address.sin_family = AF_INET;
    if (bind(udpsocket, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
        std::cout << ERROR << " : UDPsocket : opening UDP socket fail" << std::endl;

    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = 10;
    setsockopt(udpsocket, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);
}
UDPsocket::~UDPsocket()
{
    close(udpsocket);
    delete[] buffer;
    delete[] rcvBuffer;
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
	memcpy(buffer + HEADER_SIZE, msg, msgSize);
	buffer[HEADER_SIZE + msgSize] = getCRC(buffer, HEADER_SIZE + msgSize);

    //  send message
    if(sendto(udpsocket, buffer, HEADER_SIZE + msgSize + 1, 0, reinterpret_cast<const sockaddr*>(&addrDest), sizeof(addrDest)) < 0)
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
            case EMSGSIZE:   std::cout << ERROR << " : UDPsocket : message size problem(" << HEADER_SIZE + msgSize + 1 << "o)" << std::endl; break;
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
    else return true;
}
std::string UDPsocket::read()
{
    sockaddr_in src_addr;
    socklen_t src_addr_len = sizeof(src_addr);
    int rcvlen = recvfrom(udpsocket, rcvBuffer, UDP_RCV_BUFFERSIZE, 0, reinterpret_cast<sockaddr*>(&src_addr), &src_addr_len);
    if(rcvlen < 0)
    {
        switch (errno)
        {
            case EBADF:       std::cout << ERROR << " : UDPsocket : bad file descriptor" << std::endl; break;
            case ECONNRESET:  std::cout << ERROR << " : UDPsocket : connection closed by peer" << std::endl; break;
            case EINTR:       std::cout << ERROR << " : UDPsocket : signal interrupt" << std::endl; break;
            case EINVAL:      std::cout << ERROR << " : UDPsocket : MSG_OOB flag set and no out-of-band data available" << std::endl; break;
            case ENOTCONN:    std::cout << ERROR << " : UDPsocket : socket not in connected state" << std::endl; break;
            case ENOTSOCK:    std::cout << ERROR << " : UDPsocket : socket argument does not refer to a socket" << std::endl; break;
            case EOPNOTSUPP:  std::cout << ERROR << " : UDPsocket : flag not supported" << std::endl; break;
            case ETIMEDOUT:   std::cout << ERROR << " : UDPsocket : timeout" << std::endl; break;
            case EIO:         std::cout << ERROR << " : UDPsocket : I/O error while reading" << std::endl; break;
            case ENOBUFS:     std::cout << ERROR << " : UDPsocket : insufficient resources available" << std::endl; break;
            case ENOMEM:      std::cout << ERROR << " : UDPsocket : insufficient memory available" << std::endl; break;
            default: break;
        }
    }
    else if(rcvlen == UDP_RCV_BUFFERSIZE)
        std::cout << ERROR << " : UDPsocket : message too long" << std::endl;
    else if (rcvlen > 0 && rcvlen < HEADER_SIZE)
        std::cout << ERROR << " : UDPsocket : message too short" << std::endl;
    else if (rcvlen >= HEADER_SIZE)
    {
        try
        {
            uint8_t crc = getCRC(rcvBuffer, rcvlen - 1);
            std::string timestampStr = std::string((char*)(rcvBuffer + 2), 8);
            std::string lengthStr = std::string((char*)(rcvBuffer + 10), 5);
            //unsigned int timestampInt = std::stoi(timestampStr, nullptr, 16);
            unsigned int lengthInt = std::stoi(lengthStr, nullptr, 16)-1;

            if(rcvBuffer[0] != 0xFF || rcvBuffer[1] != 0xFF)
                std::cout << WARNING << " UDPsocket : wrong header" << std::endl;
            else if(rcvBuffer[rcvlen - 1] != crc)
                std::cout << WARNING << " UDPsocket : wrong crc (received : " << (int)rcvBuffer[rcvlen - 1] << ", computed : " << (int)crc << ")"<< std::endl;
            else if(lengthInt <= 0)
                return "";
            else return std::string((char*)(rcvBuffer + HEADER_SIZE), lengthInt);
        }
        catch(const std::exception& e)
        {
            std::cout<<WARNING<<" UDPsocket : read received exception : "<<e.what()<<" => maybe an empty msg"<<std::endl;
        }
    }
    return "";
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
