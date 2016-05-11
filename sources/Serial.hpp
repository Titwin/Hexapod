#ifndef SERIAL_HPP_INCLUDED
#define SERIAL_HPP_INCLUDED

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <errno.h>

class Serial
{
    public:
        //  Miscellaneous
        enum SerialCommandCode {
          Ping = 1,
          SetMotorPosition,
          GetMotorPosition,
          EnableDisableTorque,
          OpenUmbrella
        };
        //

        //  Default
        Serial();
        ~Serial();
        //

        //  Public functions
        void send(uint8_t commandCode,uint8_t* buffer,uint8_t buffSize);
        bool readMessage(uint8_t* buffer,uint8_t buffSize,uint8_t timeout = _timeout);
        //

    private:
        //  Private functions
        void openSerialPort(std::string portName);
        void closeSerialPort();
        int16_t readByteTimed(uint8_t timeout);
        bool parseReceivedChar(uint8_t c,uint8_t* buffer,uint8_t buffSize);
        //

        //  Attributes
        static uint16_t _timeout;
        struct termios tio;
        int tty_fd;

        uint16_t start;
        uint8_t crc;
        uint8_t state;
        uint8_t code;
        uint8_t msgLength;
        uint8_t buffIndex;
        //
};

#endif // SERIAL_HPP_INCLUDED
