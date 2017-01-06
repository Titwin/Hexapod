#ifndef HEXAPOD_HPP_INCLUDED
#define HEXAPOD_HPP_INCLUDED

#include <deque>


#include "ForwardKinematics.hpp"
#include "InverseKinematics.hpp"

#include "SerialProtocol.hpp"




class Hexapod
{
    public:
        //  Special
        enum RobotGait {
            TRIPOD = 0,
            RIPPLE,
            METACHRONAL_6,
            TETRAPOD,
            METACHRONAL_4
        };
        enum RobotState {
            STOP = 0,
            WALK = 1,
            TRANSITION = 2,
            INIT = 3
        };
        enum LegsState {
            ON_GROUND = 0,
            IN_AIR,
            AUTONOMOUS
        };
        //


        //  Default
        Hexapod(std::string s = "'unknown hexapod'");
        ~Hexapod();
        //


        //  Public functions
        void setState(RobotState s);
        RobotState getState(){return (RobotState)robotState;};
        void setGait(RobotGait g);
        void toogleTorque();
        bool getTorque(){return torque;};
        bool legsInPosition();
        bool obstructed(int sharpIndex){return frontDistance[sharpIndex]<20;};

        int debug(){return legTrajectory[0].size();};

        bool isWellInitialized(){return wellInitialized;};
        void animate(float elapseTime, MyVector3f translationSpeed, MyVector3f rotationSpeed);
        void attack(int legIndex);
        void setPushPosition(bool enable);
        void openUmbrella();
        //

    private:
        //  Private functions
        void animateWalk(float elapseTime, MyVector3f translationSpeed, MyVector3f rotationSpeed);
        void animateGotoStart(float elapseTime);
        void animateTransition(float elapseTime);
        void animateAutonomousLegs(float elapseTime);

        MyVector3f getLegDisplacement(int legIndex,const MyVector3f& translationSpeed,const MyVector3f& rotationSpeed);
        void updateLegPositionFromMotorsAngles();
        void updateMotorsAnglesFromLegPosition();
        bool tryConnectingToArduino();

        float getLegLimitFromDirection(int legIndex,MyVector3f direction);
        MyVector3f getAirTarget(int legIndex,MyVector3f takeoffPos,float t);
        void correctPositionError(int legIndex);
        void initiateLegDomainCenter(float radius,float angle,float zeroHeight);

        inline void constructTransitionTrajectory();
        inline float motorAngleFromBytes(const uint8_t& b1,const uint8_t& b2,const int8_t& s,const int16_t& offset);
        inline float limit(MyVector3f C,float R,MyVector3f P,MyVector3f u);
        //



        //  Attributes
        //  robot attributes
            std::string name;
            bool wellInitialized;
            ForwardKinematics* forwardKModule;
            InverseKinematics* inverseKModule;
            SerialProtocol* arduino;
            bool torque;

        //  limit speed and walk parameter
            float maxTranslationSpeedMagnitude;
            float maxRotationSpeedMagnitude;
            float speedGain;
            float stepLength,stepHeight,robotHeight;
            float gainP;

        //  important 3D position for algorithm
            MyVector3f* legPosition;
            MyVector3f* legTarget;
            MyVector3f* legSendPosition;
            MyVector3f* legDisplacement;
            MyVector3f* legDomainCenter;
            MyVector3f* takeoffdownPosition;
            MyVector3f* legErrorIntegral;

        //  trajectory parameters
            std::deque<std::pair<MyVector3f,float> > legTrajectory[6];
            bool blockAnimFlag[6];

            float dummy;

        //  leg machine states attributes
            int8_t legsState[6],legsGroup[6];
            float legsIncidence[6],legsIncidenceTarget[6];

        //  robot machine state attributes
            int8_t robotState,robotGait,robotTargetGait,transitState;
            uint8_t swingGroup,nbGroup;
            float limitForSwap,initialLimitForSwap;
            int frontDistance[3];

        //  angles parameters
            int16_t motorAnglesOffset[24];
            int8_t sens[24];

            LegAngleSet motorAnglesArray[6];
            LegAngleSet targetAnglesArray[6];

        //
};




#endif // HEXAPOD_HPP_INCLUDED
