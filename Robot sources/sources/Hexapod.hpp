#ifndef HEXAPOD_HPP_INCLUDED
#define HEXAPOD_HPP_INCLUDED

#include <deque>


#include "Maths/ForwardKinematics.hpp"
#include "Maths/InverseKinematics.hpp"

class Hexapod
{
    public:
        ///  Special
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


        ///  Default
        Hexapod(std::string s = "'unknown hexapod'");
        ~Hexapod();
        //


        ///  Public functions
        void setTorque(bool b);
        bool getTorque(){return torque;};

        void setMotorAngles(uint8_t* data);
        const uint8_t* getMotorsIds() const;
        uint16_t* getGoalMotorAngles();
        void getSensorsConfiguration(std::pair<MyVector3f, MyVector3f>* configuration) const;

        void animate(float elapseTime, MyVector3f translationSpeed, MyVector3f rotationSpeed);

        void setState(RobotState s);
        RobotState getState(){return (RobotState)robotState;};
        void setGait(RobotGait g, bool log = false);
        void toogleTorque();

        bool legsInPosition();
        MyVector3f getCorrectedTranslationSpeed() const { return correctedTranslationSpeed; };
        MyVector3f getCorrectedRotationSpeed() const { return correctedRotationSpeed; };

        int debug(){return legTrajectory[0].size();};

        bool isWellInitialized(){return wellInitialized;};
        //

    private:
        ///  Private functions
        void animateWalk(float elapseTime, MyVector3f translationSpeed, MyVector3f rotationSpeed);
        void animateGotoStart(float elapseTime);
        void animateTransition(float elapseTime);
        void animateAutonomousLegs(float elapseTime);

        MyVector3f getLegDisplacement(int legIndex,const MyVector3f& translationSpeed,const MyVector3f& rotationSpeed);

        float getLegLimitFromDirection(int legIndex,MyVector3f direction);
        MyVector3f getAirTarget(int legIndex,MyVector3f takeoffPos,float t);
        void initiateLegDomainCenter(float radius,float angle,float zeroHeight);

        inline void constructTransitionTrajectory();
        inline float motorAngleFromBytes(const uint8_t& b1,const uint8_t& b2,const int8_t& s,const int16_t& offset);
        inline float limit(MyVector3f C,float R,MyVector3f P,MyVector3f u);
        //


        ///  Attributes
        //  robot attributes
            std::string name;
            bool wellInitialized;
            ForwardKinematics* forwardKModule;
            InverseKinematics* inverseKModule;
            bool torque;
            float lastMoveTime;

        //  limit speed and walk parameter
            float maxTranslationSpeedMagnitude;
            float maxRotationSpeedMagnitude;
            MyVector3f correctedTranslationSpeed;
            MyVector3f correctedRotationSpeed;
            float speedGain;
            float stepLength,stepHeight,robotHeight;
            float gainP;

        //  important 3D position for algorithm
            MyVector3f* legPosition;
            MyVector3f* legTarget;
            MyVector3f* legDisplacement;
            MyVector3f* legDomainCenter;
            MyVector3f* takeoffdownPosition;

        //  trajectory parameters
            std::deque<std::pair<MyVector3f,float> > legTrajectory[6];
            bool blockAnimFlag[6];

        //  leg machine states attributes
            int8_t legsState[6],legsGroup[6];

        //  robot machine state attributes
            int8_t robotState,robotGait,robotTargetGait,transitState;
            uint8_t swingGroup,nbGroup;
            float limitForSwap,initialLimitForSwap;
            //int frontDistance[3];

        //  angles parameters
            int16_t motorAnglesOffset[6][3];
            uint16_t goalMotorAngle[18];
            static const uint8_t motorIDs[18];
            int8_t sens[6][3];

            LegAngleVector motorAnglesArray[6];
            LegAngleVector targetAnglesArray[6];
        //
};




#endif // HEXAPOD_HPP_INCLUDED
