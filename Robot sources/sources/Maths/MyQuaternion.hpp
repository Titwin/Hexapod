#ifndef MYQUATERNION_HPP_INCLUDED
#define MYQUATERNION_HPP_INCLUDED

#include <stdint.h>
#include <cmath>
#include <stdexcept>

#include "MyVector.hpp"
#include "MyMatrix3.hpp"

template<typename T>
class MyQuaternion : public MyVector4<T>
{
    public:
        //Public functions
            MyQuaternion(T X,T Y,T Z,T W) : MyVector4<T>(X,Y,Z,W){};

        //{ Special functions
            MyQuaternion inverse() const
            {
                return MyQuaternion(-MyVector4<T>::x, -MyVector4<T>::y, -MyVector4<T>::z, MyVector4<T>::w);
            };
            static MyQuaternion identity()
            {
                return MyQuaternion(0,0,0,1);
            }
        //}
};

typedef MyQuaternion<float>  MyQuaternionf;
typedef MyQuaternion<double> MyQuaterniond;
#endif // MYQUATERNION_HPP_INCLUDED
