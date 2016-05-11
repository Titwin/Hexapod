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
        LegAngleSet getAnglesFromPosition(int legIndex,MyVector3f targetPosition,float thirdAngle = 0,LegAngleSet initialAngle = defaultAngle);

        void setForwardKinematicsModule(ForwardKinematics* module);
        ForwardKinematics* getForwardKinematicsModule();
        //

    private:
        //  Attributes
        ForwardKinematics* fk;

        float step;
        float gain;
        float angleIterationStepLimit;
        float angleError;
        float targetError;
        int maxIteration;

        static LegAngleSet defaultAngle;
        //
};

#endif // INVERSEKINEMATICS_HPP_INCLUDED
