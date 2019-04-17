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
        //{ Special functions
            MyMatrix3<T> toMatrix3() const
            {
                return MyMatrix3<T>(1-2*MyVector4<T>::y*MyVector4<T>::y-2*MyVector4<T>::z*MyVector4<T>::z, 2*MyVector4<T>::x*MyVector4<T>::y-2*MyVector4<T>::z*MyVector4<T>::w,   2*MyVector4<T>::x*MyVector4<T>::z+2*MyVector4<T>::y*MyVector4<T>::y,
                                    2*MyVector4<T>::x*MyVector4<T>::y+2*MyVector4<T>::z*MyVector4<T>::w,   1-2*MyVector4<T>::x*MyVector4<T>::x-2*MyVector4<T>::z*MyVector4<T>::z, 2*MyVector4<T>::y*MyVector4<T>::z-2*MyVector4<T>::x*MyVector4<T>::w,
                                    2*MyVector4<T>::x*MyVector4<T>::z-2*MyVector4<T>::y*MyVector4<T>::w,   2*MyVector4<T>::y*MyVector4<T>::z+2*MyVector4<T>::x*MyVector4<T>::w,   1-2*MyVector4<T>::x*MyVector4<T>::x-2*MyVector4<T>::y*MyVector4<T>::y);
            };
            MyMatrix4<T> toMatrix4() const
            {
                MyMatrix4<T> r;
                MyMatrix3<T> R = toMatrix3();
                for(uint8_t i=0;i<3;i++)
                    for(uint8_t j=0;j<3;j++)
                        r.a[i][j] = R.a[i][j];
                return r;
            }
        //}
};

typedef MyQuaternion<float>  MyQuaternionf;
typedef MyQuaternion<double> MyQuaterniond;
#endif // MYQUATERNION_HPP_INCLUDED
