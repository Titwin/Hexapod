#include "InverseKinematics.hpp"

LegAngleSet InverseKinematics::defaultAngle = LegAngleSet(0,45,0,-90);

//Default
InverseKinematics::InverseKinematics()
{
    fk = NULL;

    step = 0.2f;
    gain = 10;
    angleErrorStopCondition = 0.1;
    angleIterationBound = 10.f;
    positionErrorStopCondition = 0.1f;
    directionErrorStopCondition = 0.05f;
    maxIteration = 200;
}
//

//  Public functions
LegAngleSet InverseKinematics::getAnglesFromExtremityAndIncidence(int legIndex,MyVector3f targetPosition,float incidence,LegAngleSet initialAngle)
{
    if(!fk) return initialAngle;

    LegAngleSet finalAngle = initialAngle;
        finalAngle.angle3 = incidence;

    MyVector3f gradient;                            //  gradient vector
    MyVector3f positionError;                       //  error vector from iteration position to target
    MyVector3f tmpLegPosition;                      //  legPosition after iteration process

    float angleContrib1,angleContrib2,angleContrib4;//  iteration angle contribution
    float a1 = initialAngle.angle1;                 //  temporary angle
    float a2 = initialAngle.angle2;
    float a4 = initialAngle.angle4;

    tmpLegPosition = fk->getLegExtremityPosition(legIndex,finalAngle);
    positionError = targetPosition - tmpLegPosition;

    for(int i=0;i<maxIteration;i++)
    {
        //gradient for angle 1
        gradient = fk->getLegExtremityPosition(legIndex,finalAngle + LegAngleSet(step,0,0,0)) - tmpLegPosition;
        angleContrib1 = gain*(positionError*gradient);
        if(angleContrib1 >  angleIterationBound)
             a1 = finalAngle.angle1 + angleIterationBound ;
        else if(angleContrib1 < -angleIterationBound)
             a1 = finalAngle.angle1 - angleIterationBound ;
        else a1 = finalAngle.angle1 + angleContrib1;

        //gradient for angle 2
        gradient = fk->getLegExtremityPosition(legIndex,finalAngle + LegAngleSet(0,step,0,0)) - tmpLegPosition;
        angleContrib2 = gain*(positionError*gradient);
        if(angleContrib2 >  angleIterationBound)
             a2 = finalAngle.angle2 + angleIterationBound ;
        else if(angleContrib2 < -angleIterationBound)
             a2 = finalAngle.angle2 - angleIterationBound ;
        else a2 = finalAngle.angle2 + angleContrib2;

        //gradient for angle 4
        gradient = fk->getLegExtremityPosition(legIndex,finalAngle + LegAngleSet(0,0,0,step)) - tmpLegPosition;
        angleContrib4 = gain*(positionError*gradient);
        if(angleContrib4 >  angleIterationBound)
             a4 = finalAngle.angle4 + angleIterationBound ;
        else if(angleContrib4 < -angleIterationBound)
             a4 = finalAngle.angle4 - angleIterationBound ;
        else a4 = finalAngle.angle4 + angleContrib4;

        //  first break condition
        if(std::abs(angleContrib1) < angleErrorStopCondition && std::abs(angleContrib2) < angleErrorStopCondition && std::abs(angleContrib4) < angleErrorStopCondition)
            break;

        //  move
        finalAngle.angle1 = a1;
        finalAngle.angle2 = a2;
        finalAngle.angle4 = a4;

        //  recalculate leg position
        tmpLegPosition = fk->getLegExtremityPosition(legIndex,finalAngle);
        positionError = targetPosition - tmpLegPosition;

        //  second break condition
        if(positionError.length() < positionErrorStopCondition)
            break;
    }
    return finalAngle;
}
LegAngleSet InverseKinematics::getAnglesFromJointAndDirection(int legIndex,MyVector3f jointTargetPosition,MyVector3f legTargetDirection,LegAngleSet initialAngle)
{
    if(!fk) return initialAngle;
    legTargetDirection.normalize();

    LegAngleSet finalAngle = initialAngle;
    MyVector3f gradient;
    MyVector3f jointPositionError,directionError;
    MyVector3f tmpJointPosition,tmpDirection;

    float angleContrib1,angleContrib2,angleContrib3,angleContrib4;
    float a1 = initialAngle.angle1;
    float a2 = initialAngle.angle2;
    float a3 = initialAngle.angle3;
    float a4 = initialAngle.angle4;

    tmpJointPosition = fk->getLegInterJointPosition(legIndex,finalAngle);
    tmpDirection = fk->getLegPointingDirection(legIndex,finalAngle);
    jointPositionError = jointTargetPosition - tmpJointPosition;
    directionError = legTargetDirection - tmpDirection;

    for(int i=0;i<maxIteration;i++)
    {
        //gradient for angle 1
        gradient = fk->getLegInterJointPosition(legIndex,finalAngle + LegAngleSet(step,0,0,0)) - tmpJointPosition;
        angleContrib1 = gain*(jointPositionError*gradient);
        if(angleContrib1 >  angleIterationBound)
             a1 = finalAngle.angle1 + angleIterationBound ;
        else if(angleContrib1 < -angleIterationBound)
             a1 = finalAngle.angle1 - angleIterationBound ;
        else a1 = finalAngle.angle1 + angleContrib1;

        //gradient for angle 2
        gradient = fk->getLegInterJointPosition(legIndex,finalAngle + LegAngleSet(0,step,0,0)) - tmpJointPosition;
        angleContrib2 = gain*(jointPositionError*gradient);
        if(angleContrib2 >  angleIterationBound)
             a2 = finalAngle.angle2 + angleIterationBound ;
        else if(angleContrib2 < -angleIterationBound)
             a2 = finalAngle.angle2 - angleIterationBound ;
        else a2 = finalAngle.angle2 + angleContrib2;

        //gradient for angle 3
        gradient = fk->getLegPointingDirection(legIndex,finalAngle + LegAngleSet(0,0,step,0)) - tmpDirection;
        angleContrib3 = gain*(directionError*gradient);
        if(angleContrib3 >  angleIterationBound)
             a3 = finalAngle.angle3 + angleIterationBound ;
        else if(angleContrib3 < -angleIterationBound)
             a3 = finalAngle.angle3 - angleIterationBound ;
        else a3 = finalAngle.angle3 + angleContrib3;

        //gradient for angle 4
        gradient = fk->getLegPointingDirection(legIndex,finalAngle + LegAngleSet(0,0,0,step)) - tmpDirection;
        angleContrib4 = gain*(directionError*gradient);
        if(angleContrib4 >  angleIterationBound)
             a4 = finalAngle.angle4 + angleIterationBound ;
        else if(angleContrib4 < -angleIterationBound)
             a4 = finalAngle.angle4 - angleIterationBound ;
        else a4 = finalAngle.angle4 + angleContrib4;

        //  first break condition
        if(std::abs(angleContrib1) < angleErrorStopCondition && std::abs(angleContrib2) < angleErrorStopCondition && std::abs(angleContrib4) < angleErrorStopCondition)
            break;

        //  move
        finalAngle.angle1 = a1;
        finalAngle.angle2 = a2;
        finalAngle.angle3 = a3;
        finalAngle.angle4 = a4;

        //  recalculate joint position and leg direction
        tmpJointPosition = fk->getLegInterJointPosition(legIndex,finalAngle);
        tmpDirection = fk->getLegPointingDirection(legIndex,finalAngle);
        jointPositionError = jointTargetPosition - tmpJointPosition;
        directionError = legTargetDirection - tmpDirection;

        //  second break condition
        if(jointPositionError.length() < positionErrorStopCondition && directionError.length() < directionErrorStopCondition)
            break;
    }
    return finalAngle;
}
//

//  Setter / getter
void InverseKinematics::setForwardKinematicsModule(ForwardKinematics* module){ fk = module; }
void InverseKinematics::setStep(float s){ step = s; }
void InverseKinematics::setGain(float g){ gain = g; }
void InverseKinematics::setAngleIterationBound(float bound){ angleIterationBound = bound; }
void InverseKinematics::setAngleErrorStopCondition(float error){ angleErrorStopCondition = error; }
void InverseKinematics::setPositionErrorStopCOndition(float error){ positionErrorStopCondition = error; }
void InverseKinematics::setDirectionErrorStopCOndition(float error){ directionErrorStopCondition = error; }
void InverseKinematics::setMaxIteration(int count){ maxIteration = count; }

ForwardKinematics* InverseKinematics::getForwardKinematicsModule(){ return fk; }
float InverseKinematics::getStep(){ return step; }
float InverseKinematics::getGain(){ return gain; }
float InverseKinematics::getAngleIterationBound(){ return angleIterationBound; }
float InverseKinematics::getAngleErrorStopCondition(){ return angleErrorStopCondition; }
float InverseKinematics::getPositionErrorStopCOndition(){ return positionErrorStopCondition; }
float InverseKinematics::getDirectionErrorStopCOndition(){ return directionErrorStopCondition; }
int InverseKinematics::getMaxIteration(){ return maxIteration; }
//


