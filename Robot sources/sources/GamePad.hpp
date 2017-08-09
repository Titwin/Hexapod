#ifndef GAMEPAD_HPP_INCLUDED
#define GAMEPAD_HPP_INCLUDED

#include <stdint.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cmath>

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
        bool connected();

        bool isPressed(const ButtonAxisMap& id) {return buttonMap[id].value;};
        bool instantPressed(const ButtonAxisMap& id) {return buttonMap[id].pressed;};
        bool instantReleased(const ButtonAxisMap& id) {return buttonMap[id].released;};

        int16_t getAxis(const ButtonAxisMap& id) {return axisMap[id%6].value;};
        float getExpAxis(const ButtonAxisMap& id);
        //

    private:
        //  Private functions
        inline void processEvent(JsEvent event);
        //

        //  Attributes
        int fd;
        std::map<uint8_t,ButtonEvent> buttonMap;    //  button event map (store button state)
        std::map<uint8_t,AxisEvent>   axisMap;      //  axis event map (store axis value)
        bool dbg;                                   //  boolean used in debug function to avoid useless print in console
        //
};


#endif // GAMEPAD_HPP_INCLUDED
