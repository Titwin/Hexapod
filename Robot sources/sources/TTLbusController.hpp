#ifndef TTLBUSCONTROLLER_HPP_INCLUDED
#define TTLBUSCONTROLLER_HPP_INCLUDED

#include <stdint.h>
#include <unistd.h>
#include <iostream>
#include <errno.h>
#include <chrono>


class TTLbusController
{
    public:
        ///  default
        TTLbusController(int initialTimeout);
        //

        /// Attributes
        static int tty_fd;
        //

        /// Set/get functions
        void setTimeout(const int& newTime);
        int getTimeout() const;
        //

    protected:
        /// Protected functions
        float bytes2Float(const uint8_t& b0, const uint8_t& b1, const uint8_t& b2, const uint8_t& b3) const;
        int bytes2Int(const uint8_t& b0, const uint8_t& b1) const;
        void int2Bytes(uint8_t *low, uint8_t* high, int data) const;

        inline void send(const uint8_t& byte) const { write(tty_fd, &byte, 1); }
        int readBuf(const uint8_t& len, uint8_t *buffer = NULL) const;

        void enableTX(const bool& enable, unsigned int msgLength = 0);
        //void flush(clock_t start, const uint8_t msgLength);
        //

        /// Attributes
        int timeout;
        std::chrono::time_point<std::chrono::system_clock> startEnable;
        //
};

#endif // TTLBUSCONTROLLER_HPP_INCLUDED
