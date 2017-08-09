#include "GamePad.hpp"

GamePad::GamePad()
{
    //  Open Gamepad driver
    std::cout<<"connecting to gamepad dongle .. "<<std::flush;
    for(int i=0;i<600;i++)
    {
        fd = open ("/dev/input/js0", O_RDONLY|O_NONBLOCK);
        if(fd>=0) break;
    }
    if(fd<0) std::cout<<"error"<<std::endl;
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
};

bool GamePad::connected() { return fd >= 0; }

void GamePad::update()
{
    if(fd < 0)
    {
        fd = open ("/dev/input/js0", O_RDONLY|O_NONBLOCK);
        if(fd >= 0) std::cout<<"Gamepad connection !"<<std::endl;
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
            errorCode = read(fd, &event, sizeof(event));
            if(errorCode <= 0) break;
            else processEvent(event);
        }
        switch(errno)
        {
            case EAGAIN: break;
            case ENODEV:
                fd = -1;
                if(fd >= 0) std::cout<<"Gamepad deconnection !"<<std::endl;
                break;
            default:
                std::cout<<"unknown gamepad error : "<<errno<<std::endl;
                break;
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

float GamePad::getExpAxis(const ButtonAxisMap& id)
{
    int16_t value = getAxis(id);
    if(value >= 0) return std::exp( value/32768.f) - 1;
    else return -std::exp( -value/32768.f) + 1;
}
