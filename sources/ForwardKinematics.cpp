#include "ForwardKinematics.hpp"



//Default
ForwardKinematics::ForwardKinematics()
{
    legLength1 = MyVector3f(0,12,0);
    legLength2 = MyVector3f(0,19,0);

    anglesOffset[0] = 0;
    anglesOffset[1] = 35;
    anglesOffset[2] = 0;
    anglesOffset[3] = -133;

    legOrigin[0] = MyVector3f( 7.3, 4.0, 8.5);
    legOrigin[1] = MyVector3f( 7.3,-4.0, 8.5);
    legOrigin[2] = MyVector3f( 0.0,-8.6, 8.5);
    legOrigin[3] = MyVector3f(-7.3,-4.0, 8.5);
    legOrigin[4] = MyVector3f(-7.3, 4.0, 8.5);
    legOrigin[5] = MyVector3f( 0.0, 8.6, 8.5);

    legOrientation[0] = MyMatrix3f::rotation(-60,  MyVector3f(0,0,1));
    legOrientation[1] = MyMatrix3f::rotation(-120, MyVector3f(0,0,1));
    legOrientation[2] = MyMatrix3f::rotation( 180, MyVector3f(0,0,1));
    legOrientation[3] = MyMatrix3f::rotation( 120, MyVector3f(0,0,1));
    legOrientation[4] = MyMatrix3f::rotation( 60,  MyVector3f(0,0,1));
    legOrientation[5] = MyMatrix3f::rotation( 0,   MyVector3f(0,0,1));
}
//

//  Public functions
MyVector3f ForwardKinematics::getLegExtremityPosition(int legIndex,const float& angle1,const float& angle2,const float& angle3,const float& angle4)
{
    MyMatrix3f R1,R2,R3,R4;
        R1 = MyMatrix3f::rotation(angle1 + anglesOffset[0],MyVector3f(0,0,1));
        R2 = MyMatrix3f::rotation(angle2 + anglesOffset[1],MyVector3f(1,0,0));

        R3 = MyMatrix3f::rotation(angle3 + anglesOffset[2],MyVector3f(0,1,0));
        R4 = MyMatrix3f::rotation(angle4 + anglesOffset[3],MyVector3f(1,0,0));

    return legOrigin[legIndex] + legOrientation[legIndex]*R1*R2*legLength1 + legOrientation[legIndex]*R1*R2*R3*R4*legLength2;
}
MyVector3f ForwardKinematics::getLegExtremityPosition(int legIndex,LegAngleSet angles) {
    return getLegExtremityPosition(legIndex,angles.angle1,angles.angle2,angles.angle3,angles.angle4); }


MyVector3f ForwardKinematics::getLegInterJointPosition(int legIndex,const float& angle1,const float& angle2)
{
    MyMatrix3f R1,R2;
        R1 = MyMatrix3f::rotation(angle1 + anglesOffset[0],MyVector3f(0,0,1));
        R2 = MyMatrix3f::rotation(angle2 + anglesOffset[1],MyVector3f(1,0,0));

    return legOrigin[legIndex] + legOrientation[legIndex]*R1*R2*legLength1;
}
MyVector3f ForwardKinematics::getLegInterJointPosition(int legIndex,LegAngleSet angles) {
    return getLegInterJointPosition(legIndex,angles.angle1,angles.angle2); }

MyVector3f ForwardKinematics::getLegPointingDirection(int legIndex,const float& angle1,const float& angle2,const float& angle3,const float& angle4)
{
    MyMatrix3f R1,R2,R3,R4;
        R1 = MyMatrix3f::rotation(angle1,MyVector3f(0,0,1));
        R2 = MyMatrix3f::rotation(angle2,MyVector3f(1,0,0));

        R3 = MyMatrix3f::rotation(angle3,MyVector3f(0,1,0));
        R4 = MyMatrix3f::rotation(angle4,MyVector3f(1,0,0));

    return legOrientation[legIndex]*R1*R2*R3*R4*legLength2;
}
MyVector3f ForwardKinematics::getLegPointingDirection(int legIndex,LegAngleSet angles) {
    return getLegPointingDirection(legIndex,angles.angle1,angles.angle2,angles.angle3,angles.angle4); }


void ForwardKinematics::setLegOrigin(int legIndex,MyVector3f position){ legOrigin[legIndex] = position; }
void ForwardKinematics::setLegOrientation(int legIndex,MyMatrix3f orientation){ legOrientation[legIndex] = orientation; }
void ForwardKinematics::setLegLength1(MyVector3f length){ legLength1 = length; }
void ForwardKinematics::setLegLength2(MyVector3f length){ legLength2 = length; }

MyVector3f ForwardKinematics::getLegOrigin(int legIndex){ return legOrigin[legIndex]; }
//


