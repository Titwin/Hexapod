#ifndef SERVER_HPP_INCLUDED
#define SERVER_HPP_INCLUDED

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <errno.h>
#include <signal.h>

#include <vector>
#include <string>
#include <list>

#include "libwebsockets.h"

class Server
{
    public:
        //  miscellaneous
        struct per_session_data {
            int fd;
        };
        typedef std::pair<std::string,std::string> Message;
        //

        //  Default
        Server(int portNumber);
        ~Server();
        //

        //  Public functions
        void update();
        int getClientCount();
        void pushMessage(std::string title, std::string data);

        static int clientCounter;
        static std::list<Message> messageList;
        //

    private:
        //  Attributes
        lws_context_creation_info info;
        libwebsocket_protocols protocol[2];
        libwebsocket_context* context;
        //
};

#endif // SERVER_HPP_INCLUDED
