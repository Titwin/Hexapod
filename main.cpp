#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sys/time.h>

#include "sources/Hexapod.hpp"
#include "sources/GamePad.hpp"
#include "sources/bcm2835.h"

#define FRAME_TIME 16
#define TIRETTE 24//RPI_V2_GPIO_P1_15


uint32_t getMs()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return (float)(t.tv_usec)/1000.;
}
int main()
{
    std::cout<<"Hello friend!"<<std::endl;
    Hexapod damocles("Damocles");
    if(!damocles.isWellInitialized())
        return -1;
    GamePad pad1;
    if(!bcm2835_init())
    {
        std::cout<<"GPIO headers : wrong initialization"<<std::endl;
        return -1;
    }
    else
    {
        bcm2835_gpio_fsel(TIRETTE, BCM2835_GPIO_FSEL_INPT);
        bcm2835_gpio_set_pud(TIRETTE, BCM2835_GPIO_PUD_UP);
    }

    std::cout<<"Release the \"tirette\" please ..."<<std::flush;
    //while(!bcm2835_gpio_lev(TIRETTE));
    std::cout<<"thanks !!"<<std::endl;

    struct timeval loopTime;
    gettimeofday(&loopTime,NULL);
	long int start_time = loopTime.tv_sec;
    long int timer = 0;
    MyVector3f translation,rotation;
    int robotGait = Hexapod::TRIPOD;

    while(!pad1.isPressed(GamePad::J2))
    {
        //  catch user input
        pad1.update();

        translation = MyVector3f(-pad1.getAxis(GamePad::AXIS_1Y)/32768.f,-pad1.getAxis(GamePad::AXIS_1X)/32768.f, 0);
            if(pad1.getAxis(GamePad::AXIS_3Y)<0) translation.z = 0.1f;
            else if(pad1.getAxis(GamePad::AXIS_3Y)>0) translation.z = -0.1f;
        rotation = MyVector3f(-pad1.getAxis(GamePad::AXIS_2X)/32768.f,0,0);

        //  animate robot
        damocles.animate(FRAME_TIME,translation,rotation);

        if(pad1.instantPressed(GamePad::Y))
        {
            robotGait = (robotGait+1)%3;
            damocles.setGait((Hexapod::RobotGait)robotGait);
        }

        if(pad1.instantPressed(GamePad::LT))
            damocles.attack(0);
        if(pad1.instantPressed(GamePad::RT))
            damocles.attack(1);

        if(pad1.instantPressed(GamePad::LB))
            damocles.setPushPosition(true);
        if(pad1.instantPressed(GamePad::RB))
            damocles.setPushPosition(false);

        if(pad1.instantPressed(GamePad::START))
            damocles.setState(Hexapod::INIT);
        if(pad1.instantPressed(GamePad::BACK))
            damocles.toogleTorque();

        gettimeofday(&loopTime,NULL);
		timer = loopTime.tv_sec - start_time;
    }

    std::cout<<"End of match"<<std::endl;
    if(damocles.getTorque()) damocles.toogleTorque();

    sleep(1);
    damocles.openUmbrella();
    bcm2835_close();
    std::cout<<"End of program. Farewell my friend !"<<std::endl;
    return 0;
}

