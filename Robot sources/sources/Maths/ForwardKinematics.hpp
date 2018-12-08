#ifndef FORWARDKINEMATICS_HPP_INCLUDED
#define FORWARDKINEMATICS_HPP_INCLUDED

#include <utility>

#include "MyVector.hpp"
#include "MyMatrix.hpp"

struct LegAngleVector
{
    LegAngleVector() : angle1(0), angle2(0), angle3(0) {};
    LegAngleVector(float a1, float a2, float a3) : angle1(a1), angle2(a2), angle3(a3) {};
    LegAngleVector operator+(const LegAngleVector& a){
        return LegAngleVector(angle1 + a.angle1, angle2 + a.angle2, angle3 + a.angle3); };

    float angle1,angle2,angle3;
};

class ForwardKinematics
{
    public:
        //Default
        ForwardKinematics();
        //

        //  Public functions
        MyVector3f getLegExtremityPosition(int legIndex, const float& angle1, const float& angle2, const float& angle3);
        MyVector3f getLegExtremityPosition(int legIndex, const LegAngleVector& angles);

        std::pair<MyVector3f, MyVector3f> getDistanceSensorPositonDirection(int legIndex, const float& angle1, const float& angle2, const float& angle3);
        std::pair<MyVector3f, MyVector3f> getDistanceSensorPositonDirection(int legIndex, const LegAngleVector& angles);
        MyVector3f getDistanceSensorDirection(int legIndex, const float& angle1, const float& angle2, const float& angle3);
        MyVector3f getDistanceSensorDirection(int legIndex, const LegAngleVector& angles);
        MyVector3f getDistanceSensorPositon(int legIndex, const float& angle1, const float& angle2, const float& angle3);
        MyVector3f getDistanceSensorPositon(int legIndex, const LegAngleVector& angles);
        //

        //  Setter / getter
        void setLegOrigin(int legIndex, MyVector3f position);
        void setLegOrientation(int legIndex, MyMatrix3f orientation);
        void setLegLength1(MyVector3f length);
        void setLegLength2(MyVector3f length);
        void setAngleOffset(int jointIndex, float offset);

        MyVector3f getLegOrigin(int legIndex);
        MyMatrix3f getLegOrientation(int legIndex);
        float getLegLength1();
        float getLegLength2();
        float getAngleOffset(int jointIndex);
        //

    private:
        //  Attributes
        MyVector3f legLength1, legLength2, legLength3;
        MyVector3f sensorPosition;
        MyVector3f legOrigin[6];
        MyMatrix3f legOrientation[6];
        float anglesOffset[3];
        //
};

#endif // FORWARDKINEMATICS_HPP_INCLUDED
