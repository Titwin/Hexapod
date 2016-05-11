#include "Serial.hpp"


uint16_t Serial::_timeout = 3;

//Default
Serial::Serial()
{
    openSerialPort("/dev/ttyAMA0");

    start = 0xFFFF;
    //if(tty_fd<0) std::cout<<" ... OK"<<std::endl;
    //else std::cout<<"\nUnable to open port name"<<std::endl;
}
Serial::~Serial()
{
    closeSerialPort();
}
//


//  Public functions
void Serial::send(uint8_t commandCode,uint8_t* buffer,uint8_t buffSize)
{
    crc = commandCode;
    write(tty_fd,&start,2);
    write(tty_fd,&commandCode,1);
    for(int i=0; i<buffSize; i++)
    {
        write(tty_fd,&buffer[i],1);
        crc += buffer[i];
    }
    write(tty_fd,&crc,1);
}
bool Serial::readMessage(uint8_t* buffer,uint8_t buffSize,uint8_t timeout)
{
    state = 0;
    int16_t c = readByteTimed(timeout);
    while(c>=0)
    {
        if(parseReceivedChar(c,buffer,buffSize))
            return true;
        c = readByteTimed(timeout);
    }
    return false;
}
//


//  Private functions
void Serial::openSerialPort(std::string portName)
{
    std::cout<<"Connecting :  "<<portName<<std::endl;

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
void Serial::closeSerialPort(){close(tty_fd);}
int16_t Serial::readByteTimed(uint8_t timeout)
{
    uint8_t data;
    for(uint16_t count=0;count<10*timeout;count++)
    {
        if(read(tty_fd,&data,1)>0) return data;
        else if(errno != EAGAIN) return -1;
        usleep(100);
    }
    return -1;
}


bool Serial::parseReceivedChar(uint8_t c,uint8_t* buffer,uint8_t buffSize)
{
    if(state == 0 && c == 0xFF) state = 1;
    else if(state == 1)
    {
        if(c == 0xFF)state = 2;
        else state = 0;
    }
    else if(state == 2)
    {
        code = c;
        crc = code;
        buffIndex = 0;

        if(code == Ping) {msgLength = 0; state = 4;}
        else if(code == GetMotorPosition) {msgLength = 54; state = 3;}
        else if(code == EnableDisableTorque) {msgLength = 0; state = 4;}
        else if(code == OpenUmbrella) {msgLength = 0; state = 4;}
        else {state = 0; std::cout<<"unknown code "<<(int)code<<std::endl;}
    }
    else if(state == 3)
    {
        if(buffIndex>=buffSize) {state = 0; std::cout<<"buffer overflow avoided"<<std::endl; return false;}
        buffer[buffIndex] = c;
        buffIndex++;
        crc += c;
        if(buffIndex >= msgLength) state = 4;
    }
    else if(state ==4)
    {
        if(c != crc) std::cout<<"wrong crc\n"<<(int)c<<" attempted\n"<<(int)crc<<" computed"<<std::endl;
        else return true;
    }
    return false;
}
//
