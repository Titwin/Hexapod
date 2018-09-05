#include "TTLbusController.hpp"
#include "bcm2835.h"

#define COM_DIRECTION 21


///  default
int TTLbusController::tty_fd = -1;
TTLbusController::TTLbusController(int initialTimeout) : timeout(initialTimeout)
{
    bcm2835_gpio_fsel(COM_DIRECTION, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_set(COM_DIRECTION);
};
//

/// Set/get functions
void TTLbusController::setTimeout(const int& newTime) { timeout = newTime; }
int TTLbusController::getTimeout() const { return timeout; }
//

/// Protected functions
float TTLbusController::bytes2Float(const uint8_t& b0, const uint8_t& b1, const uint8_t& b2, const uint8_t& b3) const
{
    uint8_t ptr[4] = { b0, b1, b2, b3 };
    return *(float *)&ptr;
}
int TTLbusController::bytes2Int(const uint8_t& b0, const uint8_t& b1) const
{
    return ((int)b1 << 8)|b0;
}
void TTLbusController::int2Bytes(uint8_t *low, uint8_t* high, int data) const
{
  *low = (data>>8);
  *high = (data&0xFF);
}
int TTLbusController::readBuf(const uint8_t& len, uint8_t *buffer) const
{
    uint8_t readBytes = 0;
    uint8_t data;

    //  polling bytes
    while(readBytes < len && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startEnable).count() < timeout)
    {
        if(read(tty_fd, &data, 1) > 0)
        {
            if(buffer) buffer[readBytes] = data;
            readBytes++;
        }
        else if(errno != EAGAIN) // read error
            return readBytes;
    }
    return readBytes;
}
void TTLbusController::enableTX(const bool& enable, unsigned int msgLength)
{
    if(enable)
    {
        bcm2835_gpio_clr(COM_DIRECTION);
        startEnable = std::chrono::high_resolution_clock::now();
    }
    else
    {
        int stop = (msgLength+3)*10 + 20;
        while(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - startEnable).count() <= stop);
        bcm2835_gpio_set(COM_DIRECTION);
    }
}
//

