#include "SerialProtocol.hpp"

//Default
SerialProtocol::SerialProtocol()
{
    timestamp = 0;
    resetState();
    ping.first = RPI_INST_PING;
    pingAck.first = RPI_INST_ACK;
    openSerialPort("/dev/ttyAMA0");

    if(tty_fd<0) std::cout<<"\nUnable to open port name"<<std::endl;
    else std::cout<<"   ... OK"<<std::endl;
}
SerialProtocol::~SerialProtocol()
{
    closeSerialPort();
}
//


//  Public functions
void SerialProtocol::debug(const Message& m,int16_t sum)
{
    std::cout<<(int)startByte<<' ';
    std::cout<<(int)startByte<<" 0x";
    std::cout<<std::hex<<(int)m.first<<std::dec<<' ';

    if(!m.second.empty()) std::cout<<(int)m.second.size()<<' ';
    for(unsigned int i=0; i<m.second.size(); i++)
        std::cout<<(int)m.second[i]<<' ';

    if(sum >= 0) std::cout<<sum<<std::endl;
    else std::cout<<"crc"<<std::endl;
}

void SerialProtocol::update(uint8_t maxtime)
{
    clock_t start = clock();
    while((clock() - start)/1000 < maxtime)
    {
        int16_t c = readByteTimed(0);
        if(c >= 0)
        {
            switch(state)
            {
                case 0:
                    if(c == startByte) state = 1;
                    else resetState();
                    break;

                case 1:
                    if(c == startByte) state = 2;
                    else resetState();
                    break;

                case 2:
                    currentMsg.first = c;
                    crc = c;

                    switch(currentMsg.first & RPI_INST_MASK)
                    {
                        case RPI_INST_PING:
                        case RPI_INST_ACK:
                            state = 5;
                            break;

                        case RPI_INST_READ:
                        case RPI_INST_WRITE:
                            state = 3;
                            break;

                        default:
                            resetState();
                            break;
                    }
                    break;

                case 3:
                    crc += c;
                    msgLength = c;
                    state = 4;
                    break;

                case 4:
                    crc += c;
                    currentMsg.second.push_back(c);
                    if(currentMsg.second.size() == msgLength)
                        state = 5;
                    break;

                case 5:
                    if(c == crc)
                    {
                        timestamp = (uint64_t)clock()/1000;
                        switch(currentMsg.first & RPI_INST_MASK)
                        {
                            case RPI_INST_PING:
                                send(pingAck);
                                break;

                            case RPI_INST_ACK:
                                validMsgList.push_back(pingAck);
                                break;

                            case RPI_INST_READ:
                            case RPI_INST_WRITE:
                                validMsgList.push_back(currentMsg);
                                break;

                            default: break;
                        }
                    }
                    else
                    {
                        std::cout<<"crc:" << (int)crc << "|" << (int)c;
                        std::cout<<". Rcv msg : ";
                        debug(currentMsg,c);
                    }
                    resetState();
                    break;

                default:
                    resetState();
                    break;
            }
        }
    }

    if((uint64_t)clock()/1000 - timestamp > 3000)
        send(ping);
}

void SerialProtocol::send(SerialProtocol::Message msg)
{
    uint8_t dummy = startByte;
    uint8_t msgcrc = msg.first + msg.second.size();
    for(unsigned int i=0; i<msg.second.size(); i++)
        msgcrc += msg.second[i];

    write(tty_fd,&dummy,1);
    write(tty_fd,&dummy,1);
    write(tty_fd,&msg.first,1);
    if(!msg.second.empty())
    {
        dummy = msg.second.size();
        write(tty_fd,&dummy,1);
        write(tty_fd,&msg.second[0],msg.second.size());
    }
    write(tty_fd,&msgcrc,1);

    //std::cout<<"msg send : ";
    //debug(msg,msgcrc);
}

int SerialProtocol::validMessageCount()
{
    return validMsgList.size();
}

SerialProtocol::Message SerialProtocol::getMessage()
{
    SerialProtocol::Message msg;
    if(!validMsgList.empty())
        msg = validMsgList.front();
    validMsgList.pop_front();
    return msg;
}
//


//  Private functions
void SerialProtocol::openSerialPort(std::string portName)
{
    std::cout<<"Connecting :  "<<portName;

    memset(&tio,0,sizeof(tio));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8|CREAD|CLOCAL;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;

    tty_fd = open(portName.c_str(), O_RDWR | O_NONBLOCK);
    cfsetospeed(&tio,B115200);
    cfsetispeed(&tio,B115200);

    tcsetattr(tty_fd,TCSANOW,&tio);
}

void SerialProtocol::closeSerialPort(){close(tty_fd);}

int16_t SerialProtocol::readByteTimed(uint8_t timeout)
{
    uint8_t data;
    clock_t start = clock();
    do {
        if(read(tty_fd,&data,1) > 0) return data;
        else if(errno != EAGAIN) return -1;
    } while((clock() - start)/1000 < timeout);
    return -1;
}

void SerialProtocol::resetState()
{
    state = 0;
    msgLength = 0;
    crc = 0;
    currentMsg.first = 0;
    currentMsg.second.clear();
}
//
