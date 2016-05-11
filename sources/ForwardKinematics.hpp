#ifndef FORWARDKINEMATICS_HPP_INCLUDED
#define FORWARDKINEMATICS_HPP_INCLUDED


#include "MyVector.hpp"
#include "MyMatrix.hpp"

struct LegAngleSet
{
    LegAngleSet() : angle1(0),angle2(0),angle3(0),angle4(0) {};
    LegAngleSet(float a1,float a2,float a3,float a4) : angle1(a1),angle2(a2),angle3(a3),angle4(a4) {};
    LegAngleSet operator+(const LegAngleSet& a){
        return LegAngleSet(angle1+a.angle1, angle2+a.angle2, angle3+a.angle3, angle4+a.angle4); };

    float angle1,angle2,angle3,angle4;
};

class ForwardKinematics
{
    public:
        //Default
        ForwardKinematics();
        //

        //  Public functions
        MyVector3f getLegExtremityPosition(int legIndex,const float& angle1,const float& angle2,const float& angle3,const float& angle4);
        MyVector3f getLegExtremityPosition(int legIndex,LegAngleSet angles);

        MyVector3f getLegInterJointPosition(int legIndex,const float& angle1,const float& angle2);
        MyVector3f getLegInterJointPosition(int legIndex,LegAngleSet angles);

        MyVector3f getLegPointingDirection(int legIndex,const float& angle1,const float& angle2,const float& angle3,const float& angle4);
        MyVector3f getLegPointingDirection(int legIndex,LegAngleSet angles);

        void setLegOrigin(int legIndex,MyVector3f position);
        void setLegOrientation(int legIndex,MyMatrix3f orientation);
        void setLegLength1(MyVector3f length);
        void setLegLength2(MyVector3f length);

        MyVector3f getLegOrigin(int legIndex);
        //

    private:
        //  Attributes
        MyVector3f legLength1,legLength2;
        MyVector3f legOrigin[6];
        MyMatrix3f legOrientation[6];
        float anglesOffset[4];
        //
};

#endif // FORWARDKINEMATICS_HPP_INCLUDED
