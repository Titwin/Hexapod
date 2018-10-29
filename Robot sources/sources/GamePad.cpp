#include "GamePad.hpp"

#include <sys/ioctl.h>
#include <string.h>

/* test_bit  : Courtesy of Johan Deneux */
#define BITS_PER_LONG (sizeof(long) * 8)
#define OFF(x)  ((x)%BITS_PER_LONG)
#define BIT(x)  (1UL<<OFF(x))
#define LONG(x) ((x)/BITS_PER_LONG)
#define test_bit(bit, array)    ((array[LONG(bit)] >> OFF(bit)) & 1)

GamePad::GamePad() : rumbleSupport(0), rumbleMagnitude(0), rumbleTarget(0)
{
    //  Open Gamepad driver
    std::cout<<"connecting to gamepad dongle .. "<<std::flush;
    for(int i=0;i<600;i++)
    {
        fdjs = open ("/dev/input/js0", O_RDONLY|O_NONBLOCK);
        fdevent = open ("/dev/input/event0", O_RDWR|O_NONBLOCK);
        if(fdevent >= 0)
        {
            unsigned long features[4];
            if(ioctl(fdevent, EVIOCGBIT(EV_FF, sizeof(unsigned long) * 4), features) == -1)
            {
                close(fdevent);
                fdevent = -1;
                rumbleSupport = -1; //loading of js but rumble not supported
            }
            else
            {
                if (test_bit(FF_RUMBLE, features))
                    rumbleSupport = 1; // rumble supported
            }
        }
        if(fdjs>=0 && rumbleSupport!=0) break;
    }

    if(fdjs<0) std::cout<<"error"<<std::endl;
    else std::cout<<"ok"<<std::endl;

    //  Initialize event map & debug
    for(int i=0;i<12;i++)
    {
        buttonMap[i].value = false;
        buttonMap[i].pressed = false;
        buttonMap[i].released = false;
    }
    for(int i=0;i<6;i++)
        axisMap[i].value = 0;
    dbg = true;

    effect.type = FF_RUMBLE;
    effect.u.rumble.strong_magnitude = 0;
    effect.u.rumble.weak_magnitude = 0;
    effect.replay.length = 1000;
    effect.replay.delay = 0;
    effect.id = -1;

    stop.type = EV_FF;
    stop.code = effect.id;
    stop.value = 0;

    play.type = EV_FF;
    play.code = effect.id;
    play.value = 1;
};
GamePad::~GamePad()
{
    close(fdjs);
    close(fdevent);
}
bool GamePad::connected() { return fdjs >= 0; }

void GamePad::update()
{
    if(fdjs < 0)
    {
        fdjs = open("/dev/input/js0", O_RDONLY|O_NONBLOCK);
        if(fdjs >= 0)
        {
            std::cout<<"Gamepad connection !";
            if(rumbleSupport>=0)
            {
                fdevent = open ("/dev/input/event0", O_RDONLY|O_NONBLOCK);
                if(fdevent >= 0)
                {
                    unsigned long features[4];
                    if(ioctl(fdevent, EVIOCGBIT(EV_FF, sizeof(unsigned long) * 4), features) == -1)
                    {
                        close(fdevent);
                        fdevent = -1;
                        rumbleSupport = -1; //loading of js but rumble not supported
                    }
                    else
                    {
                        if (test_bit(FF_RUMBLE, features))
                        {
                            rumbleSupport = 1;
                        }
                    }
                }
            }
            if(rumbleSupport>0) std::cout<<"(with rumble)"<<std::endl;
            else std::cout<<"(rumble not supported)"<<std::endl;
        }
    }
    else
    {
        //  erase some value for rising edge and falling edge detection
        for(int i=0;i<12;i++)
        {
            buttonMap[i].pressed = false;
            buttonMap[i].released = false;
        }

        //  polling all event in queue
        JsEvent event;
        int errorCode;
        while(true)
        {
            errorCode = read(fdjs, &event, sizeof(event));
            if(errorCode <= 0) break;
            else processEvent(event);
        }
        switch(errno)
        {
            case EAGAIN: break;
            case ENODEV:
                if(fdjs >= 0)
                    std::cout<<"Gamepad deconnection !"<<std::endl;
                fdjs = -1;
                break;
            default:
                std::cout<<"unknown gamepad error : "<<errno<<std::endl;
                break;
        }

        //  rumble
        if(rumbleSupport>0 && fdevent>=0 && rumbleTarget!=rumbleMagnitude)
        {
            stop.code = effect.id;
            if (write(fdevent, (const void*) &stop, sizeof(stop)) == -1)
                std::cout<<"error stopping effect"<<std::endl;

            effect.u.rumble.strong_magnitude = rumbleTarget;
            effect.u.rumble.weak_magnitude = rumbleTarget;
            if (ioctl(fdevent, EVIOCSFF, &effect) == -1)
                std::cout<<"failed to upload effect: "<<strerror(errno)<<std::endl;

            play.code = effect.id;
            if (write(fdevent, (const void*) &play, sizeof(play)) == -1)
                std::cout<<"error playing effect"<<std::endl;

            rumbleMagnitude = rumbleTarget;
        }
    }
}

inline void GamePad::processEvent(JsEvent event)
{
    if(event.type&JS_EVENT_INIT) return;
    dbg = true;
    switch(event.type)
    {
        case JS_EVENT_BUTTON:
            if(event.value!=0 && !buttonMap[event.number].value)
                buttonMap[event.number].pressed = true;
            else if(event.value==0 && buttonMap[event.number].value)
                buttonMap[event.number].released = true;
            buttonMap[event.number].value = (event.value!=0);
            break;

        case JS_EVENT_AXIS:
            axisMap[event.number].value = event.value;
            break;

        default: break;
    }
};

void GamePad::debug()
{
    if(!dbg) return;

    std::cout<<"X : "<<(buttonMap[0].value?"true":"false")<<std::endl;
    std::cout<<"Y : "<<(buttonMap[3].value?"true":"false")<<std::endl;
    std::cout<<"A : "<<(buttonMap[1].value?"true":"false")<<std::endl;
    std::cout<<"B : "<<(buttonMap[2].value?"true":"false")<<std::endl;

    std::cout<<"LB : "<<(buttonMap[4].value?"true":"false")<<std::endl;
    std::cout<<"RB : "<<(buttonMap[5].value?"true":"false")<<std::endl;
    std::cout<<"LT : "<<(buttonMap[6].value?"true":"false")<<std::endl;
    std::cout<<"RT : "<<(buttonMap[7].value?"true":"false")<<std::endl;

    std::cout<<"BACK : " <<(buttonMap[8].value?"true":"false")<<std::endl;
    std::cout<<"START : "<<(buttonMap[9].value?"true":"false")<<std::endl;

    std::cout<<"J1 : "<<(buttonMap[10].value?"true":"false")<<std::endl;
    std::cout<<"J2 : "<<(buttonMap[11].value?"true":"false")<<std::endl;

    std::cout<<"AXIS 1 :\n\t"<<axisMap[0].value<<"\n\t"<<axisMap[1].value<<std::endl;
    std::cout<<"AXIS 2 :\n\t"<<axisMap[2].value<<"\n\t"<<axisMap[3].value<<std::endl;
    std::cout<<"AXIS 3 :\n\t"<<axisMap[4].value<<"\n\t"<<axisMap[5].value<<std::endl;

    std::cout<<std::endl;
    dbg = false;
};


void GamePad::rumble(uint16_t mag)
{
    rumbleTarget = mag;
}


float GamePad::getExpAxis(const ButtonAxisMap& id)
{
    int16_t value = getAxis(id);
    if(value >= 0) return std::exp( value/32768.f) - 1;
    else return -std::exp( -value/32768.f) + 1;
}
