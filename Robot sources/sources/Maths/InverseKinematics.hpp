#ifndef INVERSEKINEMATICS_HPP_INCLUDED
#define INVERSEKINEMATICS_HPP_INCLUDED

#include "ForwardKinematics.hpp"

class InverseKinematics
{
    public:
        //Default
        InverseKinematics();
        //

        //  Public functions
        LegAngleVector getAnglesFromExtremity(int legIndex, MyVector3f targetPosition, LegAngleVector initialAngle = defaultAngle);
        //

        //  Setter / getter
        void setForwardKinematicsModule(ForwardKinematics* module);
        void setStep(float s);
        void setGain(float g);
        void setAngleIterationBound(float bound);
        void setAngleErrorStopCondition(float error);
        void setPositionErrorStopCOndition(float error);
        void setDirectionErrorStopCOndition(float error);
        void setMaxIteration(int count);

        ForwardKinematics* getForwardKinematicsModule();
        float getStep();
        float getGain();
        float getAngleIterationBound();
        float getAngleErrorStopCondition();
        float getPositionErrorStopCOndition();
        float getDirectionErrorStopCOndition();
        int getMaxIteration();
        //

        //  Static attributes
        static LegAngleVector defaultAngle;
        //

    private:
        //  Attributes
        ForwardKinematics* fk;

        float step;                         //  displacement value for gradient calculus
        float gain;                         //  a gain to compute the value added to final gradient
        float angleIterationBound;          //  break condition on angle displacement (in deg)
        float angleErrorStopCondition;      //  a maximum displacement bound per step
        float positionErrorStopCondition;   //  break condition on target position
        float directionErrorStopCondition;  //  break condition on target direction
        int maxIteration;                   //  break condition on compute iteration
        //
};

#endif // INVERSEKINEMATICS_HPP_INCLUDED
