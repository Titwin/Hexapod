#ifndef GAMEPAD_HPP_INCLUDED
#define GAMEPAD_HPP_INCLUDED

#include <stdint.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <map>

#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */
#define JS_EVENT_INIT   0x80    /* initial state of device */


class GamePad
{
    public:
        //  Special
        struct JsEvent {
            uint32_t time;     /* event timestamp in milliseconds */
            int16_t  value;    /* value */
            uint8_t  type;     /* event type */
            uint8_t  number;   /* axis/button number */
        };
        struct ButtonEvent {
            bool  value;
            bool  pressed;
            bool  released;
        };
        struct AxisEvent {
            int16_t  value;
        };
        enum ButtonAxisMap{
            X=0,A,B,Y, LB,RB,LT,RT, BACK,START, J1,J2,
            AXIS_1X=0,AXIS_1Y, AXIS_2X,AXIS_2Y, AXIS_3X,AXIS_3Y
        };
        //

        //  Default
        GamePad();
        //

        //  Public functions
        void update();
        void debug();

        bool isPressed(const ButtonAxisMap& id) {return buttonMap[id].value;};
        bool instantPressed(const ButtonAxisMap& id) {return buttonMap[id].pressed;};
        bool instantReleased(const ButtonAxisMap& id) {return buttonMap[id].released;};

        int16_t getAxis(const ButtonAxisMap& id) {return axisMap[id%6].value;};
        //

    private:
        //  Private functions
        void processEvent(JsEvent event);
        //

        //  Attributes
        int fd;
        std::map<uint8_t,ButtonEvent> buttonMap;
        std::map<uint8_t,AxisEvent>   axisMap;

        /*bool X,Y,A,B;
        bool LB,RB,LT,RT;
        bool BACK,START;
        bool J1,J2;

        int16_t axis1X,axis1Y;
        int16_t axis2X,axis2Y;
        int16_t axis3X,axis3Y;*/

        bool dbg;
        //
};


#endif // GAMEPAD_HPP_INCLUDED
