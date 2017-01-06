#include "Server.hpp"

//  Static attributes and functions
int Server::clientCounter = 0;
std::list<Server::Message> Server::messageList;

static int ws_service_callback(libwebsocket_context* context, libwebsocket *wsi, libwebsocket_callback_reasons reason, void *user, void *in, size_t len)
{
    switch (reason)
    {
        case LWS_CALLBACK_ESTABLISHED:
            std::cout << "[Server Service] Connection established" << std::endl;
            libwebsocket_callback_on_writable(context, wsi);
            Server::clientCounter++;
            break;

        case LWS_CALLBACK_RECEIVE:
            std::cout << "[Server Service] Server received:" << std::endl;
            for(int i=0; i<len; i++)
                std::cout << ((char*)in)[i];
            std::cout << std::endl;
            //websocket_write_back(wsi ,(char *)in, -1);
            break;

        case LWS_CALLBACK_CLOSED:
            std::cout << "[Server Service] Client close" << std::endl;
            Server::clientCounter--;
            break;

        case LWS_CALLBACK_HTTP:
            std::cout << "[Server Service] HTTP message:" << std::endl;
            break;

        case LWS_CALLBACK_SERVER_WRITEABLE:
            if(!Server::messageList.empty())
            {
                //  begin
                uint8_t buffer[512];
                uint16_t index = 0;

                //  push msg title & data
                for(uint16_t i = 0; i<Server::messageList.front().first.size() && index < 512; i++,index++)
                    buffer[index] = (uint8_t)Server::messageList.front().first[i];
                for(uint16_t i = 0; i<Server::messageList.front().second.size() && index<512; i++, index++)
                    buffer[index] = (uint8_t)Server::messageList.front().second[i];

                //  end
                Server::messageList.pop_front();
                libwebsocket_write(wsi, buffer, index, LWS_WRITE_TEXT);
            }
            libwebsocket_callback_on_writable(context, wsi);
            break;

        case LWS_CALLBACK_FILTER_NETWORK_CONNECTION:    //  ignore due to server filter useless
        case LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION:   //  ignore due to server filter useless
        case LWS_CALLBACK_PROTOCOL_INIT:                //  ignore due to server protocol initialize useless
        case LWS_CALLBACK_PROTOCOL_DESTROY:             //  ignore due to server protocol deletion useless
        case LWS_CALLBACK_ADD_POLL_FD:                  //  ignore due to poll system ignored
        case LWS_CALLBACK_DEL_POLL_FD:                  //  ignore due to poll system ignored
        case LWS_CALLBACK_SET_MODE_POLL_FD:             //  ignore due to server write callback looping
        case LWS_CALLBACK_CLEAR_MODE_POLL_FD:           //  ignore due to server write callback looping
            break;

        default: std::cout << "unknown: " << (int)reason << std::endl; break;
    }
    return 0;
}
//


//  Default
Server::Server(int portNumber)
{
    //* Server attribute declaration */
    const char *interface = NULL;
    const char *cert_path = NULL;
    const char *key_path = NULL;
    context = NULL;
    int opts = 0;

    //* setup websocket protocol */
    protocol[0].name = "robot-protocol";
    protocol[0].callback = ws_service_callback;
    protocol[0].per_session_data_size = sizeof(per_session_data);
    protocol[0].rx_buffer_size = 0;
    protocol[1].name = NULL;
    protocol[1].callback = NULL;
    protocol[1].per_session_data_size = 0;
    protocol[1].rx_buffer_size = 0;

    //* setup websocket context info*/
    memset(&info, 0, sizeof(info));
    info.port = portNumber;
    info.iface = "localhost";
    info.protocols = protocol;
    info.extensions = libwebsocket_get_internal_extensions();
    info.ssl_cert_filepath = cert_path;
    info.ssl_private_key_filepath = key_path;
    info.gid = -1;
    info.uid = -1;
    info.options = opts;

    //* create libwebsocket context. */
    lws_set_log_level(0, NULL);
    context = libwebsocket_create_context(&info);
    if (context == NULL)
    {
        std::cout << "Websocket context creation error." << std::endl;
    }
    else
    {
        std::cout << "Websocket context creation success." << std::endl;
    }
}
Server::~Server()
{
    libwebsocket_context_destroy(context);
}
//

//  Public functions
void Server::update()
{
    if(!context) return;

    int loopCounter = 0;
    while(libwebsocket_service(context,1) >= 0 && loopCounter < 10)
        loopCounter++;
}
int Server::getClientCount()
{
    if(!context) return -1;
    else return clientCounter;
}
void Server::pushMessage(std::string title, std::string data)
{
    Server::messageList.push_back(Message(title, data));
}
//


