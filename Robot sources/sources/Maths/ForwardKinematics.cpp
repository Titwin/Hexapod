#include "ForwardKinematics.hpp"



//Default
ForwardKinematics::ForwardKinematics()
{
    legLength1 = MyVector3f(0, 12.6, 0);
    legLength2 = MyVector3f(0, 22,   0);

    anglesOffset[0] = 0;
    anglesOffset[1] = 45;
    anglesOffset[2] = 0;
    anglesOffset[3] = -58;

    legOrigin[0] = MyVector3f( 6.0, 6.0, 6.4);
    legOrigin[1] = MyVector3f( 8.5, 0.0, 6.4);
    legOrigin[2] = MyVector3f( 6.0,-6.0, 6.4);
    legOrigin[3] = MyVector3f(-6.0, 6.0, 6.4);
    legOrigin[4] = MyVector3f(-8.5, 0.0, 6.4);
    legOrigin[5] = MyVector3f(-6.6,-6.0, 6.4);

    legOrientation[0] = MyMatrix3f::rotation( -45, MyVector3f(0,0,1));
    legOrientation[1] = MyMatrix3f::rotation( -90, MyVector3f(0,0,1));
    legOrientation[2] = MyMatrix3f::rotation(-135, MyVector3f(0,0,1));
    legOrientation[3] = MyMatrix3f::rotation(  45, MyVector3f(0,0,1));
    legOrientation[4] = MyMatrix3f::rotation(  90, MyVector3f(0,0,1));
    legOrientation[5] = MyMatrix3f::rotation( 135, MyVector3f(0,0,1));
}
//

//  Public functions
MyVector3f ForwardKinematics::getLegExtremityPosition(int legIndex,const float& angle1,const float& angle2,const float& angle3,const float& angle4)
{
    MyMatrix3f R1, R2, R3, R4;
        R1 = MyMatrix3f::rotation(angle1 + anglesOffset[0],MyVector3f(0,0,1));
        R2 = MyMatrix3f::rotation(angle2 + anglesOffset[1],MyVector3f(1,0,0));

        R3 = MyMatrix3f::rotation(angle3 + anglesOffset[2],MyVector3f(0,1,0));
        R4 = MyMatrix3f::rotation(angle4 + anglesOffset[3],MyVector3f(1,0,0));

    return legOrigin[legIndex] + legOrientation[legIndex] * R1 * R2 * legLength1 + legOrientation[legIndex] * R1 * R2 * R3 * R4 * legLength2;
}
MyVector3f ForwardKinematics::getLegExtremityPosition(int legIndex, LegAngleVector angles)
{
    return getLegExtremityPosition(legIndex, angles.angle1, angles.angle2, angles.angle3, angles.angle4);
}


MyVector3f ForwardKinematics::getLegInterJointPosition(int legIndex,const float& angle1,const float& angle2)
{
    MyMatrix3f R1,R2;
        R1 = MyMatrix3f::rotation(angle1 + anglesOffset[0],MyVector3f(0,0,1));
        R2 = MyMatrix3f::rotation(angle2 + anglesOffset[1],MyVector3f(1,0,0));

    return legOrigin[legIndex] + legOrientation[legIndex]*R1*R2*legLength1;
}
MyVector3f ForwardKinematics::getLegInterJointPosition(int legIndex, LegAngleVector angles)
{
    return getLegInterJointPosition(legIndex, angles.angle1, angles.angle2);
}


MyVector3f ForwardKinematics::getLegPointingDirection(int legIndex, const float& angle1, const float& angle2, const float& angle3, const float& angle4)
{
    MyMatrix3f R1, R2, R3, R4;
        R1 = MyMatrix3f::rotation(angle1,MyVector3f(0,0,1));
        R2 = MyMatrix3f::rotation(angle2,MyVector3f(1,0,0));

        R3 = MyMatrix3f::rotation(angle3,MyVector3f(0,1,0));
        R4 = MyMatrix3f::rotation(angle4,MyVector3f(1,0,0));

    return (legOrientation[legIndex] * R1 * R2 * R3 * R4 * legLength2).normalize();
}
MyVector3f ForwardKinematics::getLegPointingDirection(int legIndex, LegAngleVector angles)
{
    return getLegPointingDirection(legIndex,angles.angle1,angles.angle2,angles.angle3,angles.angle4);
}
//

//  Setter / getter
void ForwardKinematics::setLegOrigin(int legIndex, MyVector3f position){ legOrigin[legIndex] = position; }
void ForwardKinematics::setLegOrientation(int legIndex, MyMatrix3f orientation){ legOrientation[legIndex] = orientation; }
void ForwardKinematics::setLegLength1(MyVector3f length){ legLength1 = length; }
void ForwardKinematics::setLegLength2(MyVector3f length){ legLength2 = length; }
void ForwardKinematics::setAngleOffset(int jointIndex, float offset){ anglesOffset[jointIndex] = offset; }


MyVector3f ForwardKinematics::getLegOrigin(int legIndex){ return legOrigin[legIndex]; }
MyMatrix3f ForwardKinematics::getLegOrientation(int legIndex){ return legOrientation[legIndex]; }
float ForwardKinematics::getLegLength1(){ return legLength1; }
float ForwardKinematics::getLegLength2(){ return legLength2; }
float ForwardKinematics::getAngleOffset(int jointIndex){ return anglesOffset[jointIndex]; }
//


