#ifndef SERIAL_HPP_INCLUDED
#define SERIAL_HPP_INCLUDED

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <errno.h>

#include <vector>
#include <list>

class SerialProtocol
{
    public:
        //  Special
        typedef std::pair<uint8_t,std::vector<uint8_t> > Message;
        //

        //  Default
        SerialProtocol();
        ~SerialProtocol();
        //

        //  Public functions
        void debug(const Message& m,int16_t sum = -1);
        void update(uint8_t maxtime);
        void send(Message msg);
        int validMessageCount();
        Message getMessage();
        bool tragetOnline() const;
        //

    private:
        //  Private functions
        void openSerialPort(std::string portName);
        void closeSerialPort();

        int16_t readByteTimed(uint8_t timeout = 0);
        void resetState();
        //

        //  Attributes
        uint64_t timestamp;
        struct termios tio;
        int tty_fd;

        uint8_t crc,state,msgLength;
        Message ping;
        Message pingAck;
        Message currentMsg;
        std::list<Message> validMsgList;
        //

        //  miscellaneous
        #define startByte 0xFF
        //

        //  instruction
        #define RPI_INST_SHIFT  4
        #define RPI_INST_MASK   (0x0F<<RPI_INST_SHIFT)

        #define RPI_INST_PING   (1<<RPI_INST_SHIFT)
        #define RPI_INST_READ   (2<<RPI_INST_SHIFT)
        #define RPI_INST_WRITE  (3<<RPI_INST_SHIFT)
        #define RPI_INST_ACK    (4<<RPI_INST_SHIFT)
        //

        //  target
        #define RPI_TARGET_SHIFT   0
        #define RPI_TARGET_MASK    (0x0F<<RPI_TARGET_SHIFT)

        #define RPI_TARGET_CONTROL (1<<RPI_TARGET_SHIFT)
        #define RPI_TARGET_SCS15   (2<<RPI_TARGET_SHIFT)
        #define RPI_TARGET_ANALOG  (3<<RPI_TARGET_SHIFT)
        #define RPI_TARGET_SLAVE   (4<<RPI_TARGET_SHIFT)
        //

        //  target 2 for SCS15
        #define RPI_POSITION 1
        #define RPI_TORQUE 2
        #define RPI_TEMPERATURE 3
        #define RPI_FAIL_NODE 4
        //

        //  target 2 for Arduino configuration
        #define RPI_ANALOG_ENABLE (1<<2)
        #define RPI_SCS15_SCHEDULER_ENABLE (1<<3)
        #define RPI_SERVO 1
        #define RPI_PWM 2
        //

        //  target 2 for Arduino configuration
        #define RPI_ANALOG_ENABLE (1<<2)
        #define RPI_SCS15_SCHEDULER_ENABLE (1<<3)
        #define RPI_SERVO 1
        #define RPI_PWM 2
        //

        //  target 2 for slaves
        #define S_ID                0
        #define S_CONFIG            1
        #define S_DISTANCE_1        2
        #define S_DISTANCE_2        3
        #define S_SPEED_ORIENTATION 4
        #define S_ORIENTATION       5

        #define INST_ACTION  0x05
        #define INST_RESET   0x06
        //
};

#endif // SERIAL_HPP_INCLUDED
