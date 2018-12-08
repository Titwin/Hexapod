#include "ForwardKinematics.hpp"



//Default
ForwardKinematics::ForwardKinematics()
{
    legLength1 = MyVector3f(0, 4.4, 2);
    legLength2 = MyVector3f(0, 5.4, 0);
    legLength3 = MyVector3f(0, 9.9, 0);

    sensorPosition = MyVector3f(0, 0.7,  2.8);

    anglesOffset[0] = 0;
    anglesOffset[1] = 0;
    anglesOffset[2] = -90;

    legOrigin[0] = MyVector3f( 4.2,  4.7, 1.0);
    legOrigin[1] = MyVector3f( 6.52, 0.0, 1.0);
    legOrigin[2] = MyVector3f( 4.2, -4.7, 1.0);
    legOrigin[3] = MyVector3f(-4.2,  4.7, 1.0);
    legOrigin[4] = MyVector3f(-6.52, 0.0, 1.0);
    legOrigin[5] = MyVector3f(-4.2, -4.7, 1.0);

    legOrientation[0] = MyMatrix3f::rotation( -45, MyVector3f(0,0,1));
    legOrientation[1] = MyMatrix3f::rotation( -90, MyVector3f(0,0,1));
    legOrientation[2] = MyMatrix3f::rotation(-135, MyVector3f(0,0,1));
    legOrientation[3] = MyMatrix3f::rotation(  45, MyVector3f(0,0,1));
    legOrientation[4] = MyMatrix3f::rotation(  90, MyVector3f(0,0,1));
    legOrientation[5] = MyMatrix3f::rotation( 135, MyVector3f(0,0,1));
}
//

//  Public functions
MyVector3f ForwardKinematics::getLegExtremityPosition(int legIndex,const float& angle1,const float& angle2,const float& angle3)
{
    MyMatrix3f R1, R2, R3;
        R1 = MyMatrix3f::rotation(angle1 + anglesOffset[0],MyVector3f(0,0,1));
        R2 = MyMatrix3f::rotation(angle2 + anglesOffset[1],MyVector3f(1,0,0));
        R3 = MyMatrix3f::rotation(angle3 + anglesOffset[2],MyVector3f(1,0,0));

    return  legOrigin[legIndex] +
            legOrientation[legIndex] * R1 * legLength1 +
            legOrientation[legIndex] * R1 * R2 * legLength2 +
            legOrientation[legIndex] * R1 * R2 * R3 * legLength3;
}
MyVector3f ForwardKinematics::getLegExtremityPosition(int legIndex, const LegAngleVector& angles)
{
    return getLegExtremityPosition(legIndex, angles.angle1, angles.angle2, angles.angle3);
}

std::pair<MyVector3f, MyVector3f> ForwardKinematics::getDistanceSensorPositonDirection(int legIndex, const float& angle1, const float& angle2, const float& angle3)
{
    MyMatrix3f R1, R2, R3;
        R1 = MyMatrix3f::rotation(angle1 + anglesOffset[0],MyVector3f(0,0,1));
        R2 = MyMatrix3f::rotation(angle2 + anglesOffset[1],MyVector3f(1,0,0));
        R3 = MyMatrix3f::rotation(angle3 + anglesOffset[2],MyVector3f(1,0,0));

    return std::pair<MyVector3f, MyVector3f>(
        legOrigin[legIndex] +
        legOrientation[legIndex] * R1 * legLength1 +
        legOrientation[legIndex] * R1 * R2 * legLength2 +
        legOrientation[legIndex] * R1 * R2 * R3 * sensorPosition ,
        R1 * R2 * R3 * MyVector3f(0, 0, 1)   );
}
std::pair<MyVector3f, MyVector3f> ForwardKinematics::getDistanceSensorPositonDirection(int legIndex, const LegAngleVector& angles)
{
    return getDistanceSensorPositonDirection(legIndex, angles.angle1, angles.angle2, angles.angle3);
}
MyVector3f ForwardKinematics::getDistanceSensorDirection(int legIndex, const float& angle1, const float& angle2, const float& angle3)
{
    return getDistanceSensorPositonDirection(legIndex, angle1, angle2, angle3).second;
}
MyVector3f ForwardKinematics::getDistanceSensorDirection(int legIndex, const LegAngleVector& angles)
{
    return getDistanceSensorPositonDirection(legIndex, angles.angle1, angles.angle2, angles.angle3).second;
}
MyVector3f ForwardKinematics::getDistanceSensorPositon(int legIndex, const float& angle1, const float& angle2, const float& angle3)
{
    return getDistanceSensorPositonDirection(legIndex, angle1, angle2, angle3).first;
}
MyVector3f ForwardKinematics::getDistanceSensorPositon(int legIndex, const LegAngleVector& angles)
{
    return getDistanceSensorPositonDirection(legIndex, angles.angle1, angles.angle2, angles.angle3).first;
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


