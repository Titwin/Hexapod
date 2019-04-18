#ifndef MYMATRIX4_HPP_INCLUDED
#define MYMATRIX4_HPP_INCLUDED

#include <stdint.h>
#include <cmath>
#include <stdexcept>

#include "MyVector.hpp"
#include "MyMatrix3.hpp"


template<typename T>
class MyMatrix4
{
    public:
        //Default
            MyMatrix4()
            {   // default matrix is identity
                a[0][0] = 1; a[0][1] = 0; a[0][2] = 0; a[0][3] = 0;
                a[1][0] = 0; a[1][1] = 1; a[1][2] = 0; a[1][3] = 0;
                a[2][0] = 0; a[2][1] = 0; a[2][2] = 1; a[2][3] = 0;
                a[3][0] = 0; a[3][1] = 0; a[3][2] = 0; a[3][3] = 1;
            };
            template<typename T2> MyMatrix4(const MyMatrix4<T2>& m)
            {
                for(uint8_t i=0;i<4;i++)
                    for(uint8_t j=0;j<4;j++)
                        a[i][j] = m.a[i][j];
            }
            template<typename T2> MyMatrix4(const MyMatrix3<T2>& m) : MyMatrix4()
            {
                for(uint8_t i=0;i<3;i++)
                    for(uint8_t j=0;j<3;j++)
                        a[i][j] = m.a[i][j];
            }
            template<typename T2> MyMatrix4(const MyVector4<T2>& v1,const MyVector4<T2>& v2,const MyVector4<T2>& v3,const MyVector4<T2>& v4)
            {
                a[0][0] = v1.x; a[0][1] = v2.x; a[0][2] = v3.x; a[0][3] = v4.x;
                a[1][0] = v1.y; a[1][1] = v2.y; a[1][2] = v3.y; a[1][3] = v4.y;
                a[2][0] = v1.z; a[2][1] = v2.z; a[2][2] = v3.z; a[2][3] = v4.z;
                a[3][0] = v1.w; a[3][1] = v2.w; a[3][2] = v3.w; a[3][3] = v4.w;
            };
            MyMatrix4(const T& a00, const T& a01, const T& a02, const T& a03,
                      const T& a10, const T& a11, const T& a12, const T& a13,
                      const T& a20, const T& a21, const T& a22, const T& a23,
                      const T& a30, const T& a31, const T& a32, const T& a33)
            {
                a[0][0] = a00; a[0][1] = a01; a[0][2] = a02; a[0][3] = a03;
                a[1][0] = a10; a[1][1] = a11; a[1][2] = a12; a[1][3] = a13;
                a[2][0] = a20; a[2][1] = a21; a[2][2] = a22; a[2][3] = a23;
                a[3][0] = a30; a[3][1] = a31; a[3][2] = a32; a[3][3] = a33;
            };
        //

        //Public functions
            //{ Math operator ( += -= *= /= )
                MyMatrix4& operator+=(const MyMatrix4& m)
                {
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            a[i][j] += m.a[i][j];
                    return *this;
                };
                MyMatrix4& operator-=(const MyMatrix4& m)
                {
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            a[i][j] -= m.a[i][j];
                    return *this;
                };
                template<typename T2> MyMatrix4& operator*=(const T2& b)
                {
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            a[i][j] *= b;
                    return *this;
                };
                template<typename T2> MyMatrix4& operator/=(const T2& b)
                {
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            a[i][j] /= b;
                    return *this;
                };
            //}

            //{ Math operator ( + - * / )
                MyMatrix4 operator+(const MyMatrix4& m)
                {
                    MyMatrix4 r;
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            r.a[i][j] = a[i][j] + m.a[i][j];
                    return r;
                };
                MyMatrix4 operator-(const MyMatrix4& m)
                {
                    MyMatrix4 r;
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            r.a[i][j] = a[i][j] - m.a[i][j];
                    return r;
                };
                template<typename T2> friend MyMatrix4 operator*(const MyMatrix4& m,const T2& b)
                {
                    MyMatrix4 r;
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            r.a[i][j] = m.a[i][j]*b;
                    return r;
                };
                template<typename T2> friend MyMatrix4 operator*(const T2& b,const MyMatrix4& m)
                {
                    MyMatrix4 r;
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            r.a[i][j] = m.a[i][j]*b;
                    return r;
                };
                template<typename T2> MyMatrix4 operator/(const T2& b)
                {
                    MyMatrix4 r;
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            r.a[i][j] = a[i][j]/b;
                    return r;
                };

                MyMatrix4 operator*(const MyMatrix4& m)
                {
                    MyMatrix4 r;
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            r.a[i][j] = a[i][0]*m.a[0][j] + a[i][1]*m.a[1][j] + a[i][2]*m.a[2][j] + a[i][3]*m.a[3][j];
                    return r;
                }
                template<typename T2> MyVector4<T2> operator*(const MyVector4<T2>& v)
                {
                    MyVector4<T2> r;
                        r.x = a[0][0]*v.x + a[0][1]*v.y + a[0][2]*v.z + a[0][3]*v.w;
                        r.y = a[1][0]*v.x + a[1][1]*v.y + a[1][2]*v.z + a[1][3]*v.w;
                        r.z = a[2][0]*v.x + a[2][1]*v.y + a[2][2]*v.z + a[2][3]*v.w;
                        r.w = a[3][0]*v.x + a[3][1]*v.y + a[3][2]*v.z + a[3][3]*v.w;
                    return r;
                }
            //}

            //{ Special functions
                T det() const
                {
                    //  determinant formula expansion using line 1
                    T c0 = a[0][0] * getCofactor(0,0);
                    T c1 = a[0][1] * getCofactor(0,1);
                    T c2 = a[0][2] * getCofactor(0,2);
                    T c3 = a[0][3] * getCofactor(0,3);
                    return c0 - c1 + c2 - c3;
                };
                MyMatrix4& transpose() const
                {
                    MyMatrix4 r;
                    for(uint8_t i=0;i<4;i++)
                        for(uint8_t j=0;j<4;j++)
                            r.a[i][j] = a[j][i];
                    return r;
                };
                friend std::ostream& operator<<(std::ostream& os, const MyMatrix4& m)
                {
                    os<<std::fixed<<std::setprecision(3);
                    os<<"[  "<<m.a[0][0]<<" "<<std::setw(10)<<m.a[0][1]<<" "<<std::setw(10)<<m.a[0][2]<<std::setw(10)<<m.a[0][3]<<" ]\n";
                    os<<"[  "<<m.a[1][0]<<" "<<std::setw(10)<<m.a[1][1]<<" "<<std::setw(10)<<m.a[1][2]<<std::setw(10)<<m.a[1][3]<<" ]\n";
                    os<<"[  "<<m.a[2][0]<<" "<<std::setw(10)<<m.a[2][1]<<" "<<std::setw(10)<<m.a[2][2]<<std::setw(10)<<m.a[2][3]<<" ]\n";
                    os<<"[  "<<m.a[3][0]<<" "<<std::setw(10)<<m.a[3][1]<<" "<<std::setw(10)<<m.a[3][2]<<std::setw(10)<<m.a[3][3]<<" ]\n";
                    os<<std::resetiosflags(std::ios_base::fixed|std::ios_base::floatfield);
                    return os;
                };
                T getCofactor(const uint8_t& raw, const uint8_t& column) const
                {
                    MyMatrix3<T> coMatrix;
                    uint8_t k = 0;
                    uint8_t l = 0;

                    for(uint8_t i=0;i<4;i++)
                    {
                        if(i==raw) continue;
                        l=0;
                        for(uint8_t j=0;j<4;j++)
                        {
                            if(j==column) continue;
                            coMatrix.a[k][l] = a[i][j];
                            l++;
                        }
                        k++;
                    }
                    return coMatrix.det();
                };
                static MyMatrix4 rotation(const T& a, const MyVector3<T>& v)
                {
                    MyMatrix4 r = MyMatrix4();
                    MyMatrix3<T> R = MyMatrix3<T>::rotation(a, v);

                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            r.a[i][j] = R.a[i][j];
                    return r;
                };
                static MyMatrix4 translation(const MyVector3<T>& v)
                {
                    MyMatrix4 t = MyMatrix4();
                    t.a[0][3] = v.x;
                    t.a[1][3] = v.y;
                    t.a[2][3] = v.z;
                    return t;
                };
                static MyMatrix4 scale(const MyVector3<T>& v)
                {
                    MyMatrix4 s = MyMatrix4();
                    s.a[0][0] = v.x;
                    s.a[1][1] = v.y;
                    s.a[2][2] = v.z;
                    return s;
                };
                MyMatrix4 inverse() const
                {
                    T d = det();
                    if(d != 0)
                    {
                        MyMatrix4 I;
                        for(uint8_t i=0;i<4;i++)
                            for(uint8_t j=0;j<4;j++)
                                I.a[i][j] = std::pow(-1, i+j) * getCofactor(j, i);
                        return I/d;
                    }
                    else
                    {
                        throw std::invalid_argument("Matrix4 not invertible");
                        return zero();
                    }
                };
                MyVector3<T> getOrigin() const { return MyVector3<T>(a[0][3], a[1][3], a[2][3]); };
                MyVector3<T> getX() const { return MyVector3<T>(a[0][0], a[1][0], a[2][0]); };
                MyVector3<T> getY() const { return MyVector3<T>(a[0][1], a[1][1], a[2][1]); };
                MyVector3<T> getZ() const { return MyVector3<T>(a[0][2], a[1][2], a[2][2]); };

                static MyMatrix4 zero() { return MyMatrix4(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0); };
            //}

        //Attributes
        T a[4][4];
};

typedef MyMatrix4<float>  MyMatrix4f;
typedef MyMatrix4<double> MyMatrix4d;

#endif // MYMATRIX4_HPP_INCLUDED
