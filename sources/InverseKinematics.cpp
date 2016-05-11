#include "InverseKinematics.hpp"

LegAngleSet InverseKinematics::defaultAngle = LegAngleSet(0,45,0,-90);

//Default
InverseKinematics::InverseKinematics()
{
    fk = NULL;

    step = 0.2f;                    //  displacement value for gradient calculus
    gain = 10;                      //  a gain added to final gradient
    angleError = 0.1;               //  break condition on angle displacement (0.1 deg)
    angleIterationStepLimit = 10.f; //  a maximum displacement bound per step
    targetError = 0.1f;             //  break condition on leg position (1mm)
    maxIteration = 200;             //  maximum iteration before break calculus (like a timeout)
}
//

//  Public functions
LegAngleSet InverseKinematics::getAnglesFromPosition(int legIndex,MyVector3f targetPosition,float thirdAngle,LegAngleSet initialAngle)
{
    if(!fk) return initialAngle;

    LegAngleSet finalAngle = initialAngle;
        finalAngle.angle3 = thirdAngle;

    MyVector3d gradient;        //  gradient vector
    MyVector3d positionError;   //  error vector from iteration position to target
    MyVector3d tmpLegPosition;  //  legPosition after iteration process
    float angleContrib1,angleContrib2,angleContrib4;
    float a1 = initialAngle.angle1;
    float a2 = initialAngle.angle2;
    float a4 = initialAngle.angle4;

    tmpLegPosition = fk->getLegExtremityPosition(legIndex,finalAngle); //std::cout<<"p0 "<<tmpLegPosition;
    positionError = targetPosition - tmpLegPosition;  //std::cout<<"p0 "<<positionError;
    for(int i=0;i<maxIteration;i++)
    {
        //gradient for angle 1
        gradient = (fk->getLegExtremityPosition(legIndex,finalAngle + LegAngleSet(step,0,0,0)) - tmpLegPosition);
        angleContrib1 = gain*(positionError*gradient);
        if(angleContrib1 >  angleIterationStepLimit)
             a1 = finalAngle.angle1 + angleIterationStepLimit ;
        else if(angleContrib1 < -angleIterationStepLimit)
             a1 = finalAngle.angle1 - angleIterationStepLimit ;
        else a1 = finalAngle.angle1 + angleContrib1;



        //gradient for angle 2
        gradient = (fk->getLegExtremityPosition(legIndex,finalAngle + LegAngleSet(0,step,0,0)) - tmpLegPosition);
        angleContrib2 = gain*(positionError*gradient);
        if(angleContrib2 >  angleIterationStepLimit)
             a2 = finalAngle.angle2 + angleIterationStepLimit ;
        else if(angleContrib2 < -angleIterationStepLimit)
             a2 = finalAngle.angle2 - angleIterationStepLimit ;
        else a2 = finalAngle.angle2 + angleContrib2;



        //gradient for angle 4
        gradient = (fk->getLegExtremityPosition(legIndex,finalAngle + LegAngleSet(0,0,0,step)) - tmpLegPosition);
        angleContrib4 = gain*(positionError*gradient);
        if(angleContrib4 >  angleIterationStepLimit)
             a4 = finalAngle.angle4 + angleIterationStepLimit ;
        else if(angleContrib4 < -angleIterationStepLimit)
             a4 = finalAngle.angle4 - angleIterationStepLimit ;
        else a4 = finalAngle.angle4 + angleContrib4;



        //  first break condition
        if(std::abs(angleContrib1) < angleError && std::abs(angleContrib2) < angleError && std::abs(angleContrib4) < angleError)
            break;

        //  move
        finalAngle.angle1 = a1;
        finalAngle.angle2 = a2;
        finalAngle.angle4 = a4;

        //  recalculate leg position
        tmpLegPosition = fk->getLegExtremityPosition(legIndex,finalAngle);
        positionError = targetPosition - tmpLegPosition;//  debug
        //std::cout<<i<<'\t'<<positionError.length()<<std::endl;
        //std::cout<<i<<'\t'<<angleContrib1<<'\t'<<angleContrib2<<'\t'<<angleContrib4;

        if(positionError.length() < targetError)
            break;
    }
    return finalAngle;
}

void InverseKinematics::setForwardKinematicsModule(ForwardKinematics* module){fk = module;}
ForwardKinematics* InverseKinematics::getForwardKinematicsModule(){return fk;}
//


