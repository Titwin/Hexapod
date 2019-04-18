#ifndef MATHCONVERSION_HPP_INCLUDED
#define MATHCONVERSION_HPP_INCLUDED

#include "Maths/MyVector.hpp"
#include "Maths/MyQuaternion.hpp"
#include "Maths/MyMatrix.hpp"


class MathConversion
{
    public:
        template<typename T>
        static MyMatrix3<T> toMat3(const MyQuaternion<T>& q)
        {
            return MyMatrix3<T>(1-2*q.y*q.y-2*q.z*q.z, 2*q.x*q.y-2*q.z*q.w,   2*q.x*q.z+2*q.y*q.w,
                                2*q.x*q.y+2*q.z*q.w,   1-2*q.x*q.x-2*q.z*q.z, 2*q.y*q.z-2*q.x*q.w,
                                2*q.x*q.z-2*q.y*q.w,   2*q.y*q.z+2*q.x*q.w,   1-2*q.x*q.x-2*q.y*q.y);
        };
        template<typename T>
        static MyMatrix4<T> toMat4(const MyQuaternion<T>& q)
        {
            return MyMatrix4<T>(1-2*q.y*q.y-2*q.z*q.z, 2*q.x*q.y-2*q.z*q.w,   2*q.x*q.z+2*q.y*q.w,      0,
                                2*q.x*q.y+2*q.z*q.w,   1-2*q.x*q.x-2*q.z*q.z, 2*q.y*q.z-2*q.x*q.w,      0,
                                2*q.x*q.z-2*q.y*q.w,   2*q.y*q.z+2*q.x*q.w,   1-2*q.x*q.x-2*q.y*q.y,    0,
                                0,                     0,                     0,                        1);
        };
        template<typename T>
        static MyQuaternion<T> toQuat(const MyMatrix3<T>& m)
        {
            T trace = m.a[0][0] + m.a[1][1] + m.a[2][2];
            if(trace > 0)
            {
                T qw = std::sqrt(1 + trace)/2;
                return MyQuaternion<T>((m.a[2][1] - m.a[1][2])/(4*qw), (m.a[0][2] - m.a[2][0])/(4*qw), (m.a[1][0] - m.a[0][1])/(4*qw), qw);
            }
            else if(m.a[0][0] > m.a[1][1] && m.a[0][0] > m.a[2][2])
            {
                T s = std::sqrt(1 + m.a[0][0] - m.a[1][1] - m.a[2][2])*2;
                return MyQuaternion<T>((m.a[2][1] - m.a[1][2])/s, s/4, (m.a[0][1] + m.a[1][0])/s, (m.a[0][2] + m.a[2][0])/s);
            }
            else if(m.a[1][1] > m.a[2][2])
            {
                T s = std::sqrt(1 + m.a[1][1] - m.a[0][0] - m.a[2][2])*2;
                return MyQuaternion<T>((m.a[0][2] - m.a[2][0])/s, (m.a[0][1] + m.a[1][0])/s, s/4, (m.a[1][2] + m.a[2][1])/s);
            }
            else
            {
                T s = std::sqrt(1 + m.a[2][2] - m.a[0][0] - m.a[1][1])*2;
                return MyQuaternion<T>((m.a[1][0] - m.a[0][1])/s, (m.a[0][2] + m.a[2][0])/s, s/4, (m.a[1][2] + m.a[2][1])/s);
            }
        };
        template<typename T>
        static MyQuaternion<T> toQuat(const MyMatrix4<T>& m)
        {
            T trace = m.a[0][0] + m.a[1][1] + m.a[2][2];
            if(trace > 0)
            {
                T qw = std::sqrt(1 + trace)/2;
                return MyQuaternion<T>((m.a[2][1] - m.a[1][2])/(4*qw), (m.a[0][2] - m.a[2][0])/(4*qw), (m.a[1][0] - m.a[0][1])/(4*qw), qw);
            }
            else if(m.a[0][0] > m.a[1][1] && m.a[0][0] > m.a[2][2])
            {
                T s = std::sqrt(1 + m.a[0][0] - m.a[1][1] - m.a[2][2])*2;
                return MyQuaternion<T>((m.a[2][1] - m.a[1][2])/s, s/4, (m.a[0][1] + m.a[1][0])/s, (m.a[0][2] + m.a[2][0])/s);
            }
            else if(m.a[1][1] > m.a[2][2])
            {
                T s = std::sqrt(1 + m.a[1][1] - m.a[0][0] - m.a[2][2])*2;
                return MyQuaternion<T>((m.a[0][2] - m.a[2][0])/s, (m.a[0][1] + m.a[1][0])/s, s/4, (m.a[1][2] + m.a[2][1])/s);
            }
            else
            {
                T s = std::sqrt(1 + m.a[2][2] - m.a[0][0] - m.a[1][1])*2;
                return MyQuaternion<T>((m.a[1][0] - m.a[0][1])/s, (m.a[0][2] + m.a[2][0])/s, s/4, (m.a[1][2] + m.a[2][1])/s);
            }
        };


        template<typename T>
        static T remap(const T& inValue, const T& in1, const T& in2, const T& out1, const T& out2)
        {
            return (inValue - in1) / (in2 - in1) * (out2 - out1) + out1;
        }

};


#endif // MATHCONVERSION_HPP_INCLUDED
