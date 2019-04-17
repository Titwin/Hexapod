#ifndef MYMATRIX3_HPP_INCLUDED
#define MYMATRIX3_HPP_INCLUDED

#include <stdint.h>

#include "MyVector.hpp"

template<typename T>
class MyMatrix3
{
    public:
        //Default
            MyMatrix3()
            {   // default matrix is identity
                a[0][0] = 1; a[0][1] = 0; a[0][2] = 0;
                a[1][0] = 0; a[1][1] = 1; a[1][2] = 0;
                a[2][0] = 0; a[2][1] = 0; a[2][2] = 1;
            };
            template<typename T2> MyMatrix3(const MyMatrix3<T2>& m)
            {
                for(uint8_t i=0;i<3;i++)
                    for(uint8_t j=0;j<3;j++)
                        a[i][j] = m.a[i][j];
            }
            template<typename T2> MyMatrix3(const MyVector3<T2>& v1,const MyVector3<T2>& v2,const MyVector3<T2>& v3)
            {
                a[0][0] = v1.x; a[0][1] = v2.x; a[0][2] = v3.x;
                a[1][0] = v1.y; a[1][1] = v2.y; a[1][2] = v3.y;
                a[2][0] = v1.z; a[2][1] = v2.z; a[2][2] = v3.z;
            };
            MyMatrix3(const T& a00,const T& a01,const T& a02,
                      const T& a10,const T& a11,const T& a12,
                      const T& a20,const T& a21,const T& a22)
            {
                a[0][0] = a00; a[0][1] = a01; a[0][2] = a02;
                a[1][0] = a10; a[1][1] = a11; a[1][2] = a12;
                a[2][0] = a20; a[2][1] = a21; a[2][2] = a22;
            };
        //

        //Public functions
            //{ Math operator ( += -= *= /= )
                MyMatrix3& operator+=(const MyMatrix3& m)
                {
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            a[i][j] += m.a[i][j];
                    return *this;
                };
                MyMatrix3& operator-=(const MyMatrix3& m)
                {
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            a[i][j] -= m.a[i][j];
                    return *this;
                };
                template<typename T2> MyMatrix3& operator*=(const T2& b)
                {
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            a[i][j] *= b;
                    return *this;
                };
                template<typename T2> MyMatrix3& operator/=(const T2& b)
                {
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            a[i][j] /= b;
                    return *this;
                };
            //}

            //{ Math operator ( + - * / )
                MyMatrix3 operator+(const MyMatrix3& m)
                {
                    MyMatrix3 r;
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            r.a[i][j] = a[i][j] + m.a[i][j];
                    return r;
                };
                MyMatrix3 operator-(const MyMatrix3& m)
                {
                    MyMatrix3 r;
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            r.a[i][j] = a[i][j] - m.a[i][j];
                    return r;
                };
                template<typename T2> friend MyMatrix3 operator*(const MyMatrix3& m,const T2& b)
                {
                    MyMatrix3 r;
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            r.a[i][j] = m.a[i][j]*b;
                    return r;
                };
                template<typename T2> friend MyMatrix3 operator*(const T2& b,const MyMatrix3& m)
                {
                    MyMatrix3 r;
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            r.a[i][j] = m.a[i][j]*b;
                    return r;
                };
                template<typename T2> MyMatrix3 operator/(const T2& b)
                {
                    MyMatrix3 r;
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            r.a[i][j] = a[i][j]/b;
                    return r;
                };

                MyMatrix3 operator*(const MyMatrix3& m)
                {
                    MyMatrix3 r;
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            r.a[i][j] = a[i][0]*m.a[0][j] + a[i][1]*m.a[1][j] + a[i][2]*m.a[2][j];
                    return r;
                }
                template<typename T2> MyVector3f operator*(const MyVector3<T2>& v)
                {
                    MyVector3f r;
                        r.x = a[0][0]*v.x + a[0][1]*v.y + a[0][2]*v.z;
                        r.y = a[1][0]*v.x + a[1][1]*v.y + a[1][2]*v.z;
                        r.z = a[2][0]*v.x + a[2][1]*v.y + a[2][2]*v.z;
                    return r;
                }
            //}

            //{ Special functions
                T det() const
                {
                    return  a[0][0]*a[1][1]*a[2][2] + a[0][2]*a[1][0]*a[2][1] + a[0][1]*a[1][2]*a[2][0]
                           -a[0][0]*a[1][2]*a[2][1] - a[0][2]*a[1][1]*a[2][0] - a[0][1]*a[1][0]*a[2][2];
                };
                MyMatrix3& transpose() const
                {
                    MyMatrix3 r;
                    for(uint8_t i=0;i<3;i++)
                        for(uint8_t j=0;j<3;j++)
                            r.a[i][j] = a[j][i];
                    return r;
                };
                friend std::ostream& operator<<(std::ostream& os, const MyMatrix3& m)
                {
                    os<<std::fixed<<std::setprecision(3);
                    os<<"\n[  "<<m.a[0][0]<<" "<<std::setw(10)<<m.a[0][1]<<" "<<std::setw(10)<<m.a[0][2]<<" ]\n";
                    os<<  "[  "<<m.a[1][0]<<" "<<std::setw(10)<<m.a[1][1]<<" "<<std::setw(10)<<m.a[1][2]<<" ]\n";
                    os<<  "[  "<<m.a[2][0]<<" "<<std::setw(10)<<m.a[2][1]<<" "<<std::setw(10)<<m.a[2][2]<<" ]\n";
                    os<<std::resetiosflags(std::ios_base::fixed|std::ios_base::floatfield);
                    return os;
                };

                static MyMatrix3 rotation(T a, MyVector3<T> v)
                {
                    v.normalize();
                    a *= 3.14159265/180.;

                    T qw = std::cos(a/2);
                    T qx = v.x*std::sin(a/2);
                    T qy = v.y*std::sin(a/2);
                    T qz = v.z*std::sin(a/2);

                    MyMatrix3 r;
                        r.a[0][0] = 1 - 2*qy*qy - 2*qz*qz;
                        r.a[0][1] =     2*qx*qy - 2*qz*qw;
                        r.a[0][2] =     2*qx*qz + 2*qy*qw;

                        r.a[1][0] =     2*qx*qy + 2*qz*qw;
                        r.a[1][1] = 1 - 2*qx*qx - 2*qz*qz;
                        r.a[1][2] =     2*qy*qz - 2*qx*qw;

                        r.a[2][0] =     2*qx*qz - 2*qy*qw;
                        r.a[2][1] =     2*qy*qz + 2*qx*qw;
                        r.a[2][2] = 1 - 2*qx*qx - 2*qy*qy;
                    return r;
                };
            //}

        //Attributes
        T a[3][3];
};

typedef MyMatrix3<float>  MyMatrix3f;
typedef MyMatrix3<double> MyMatrix3d;

#endif // MYMATRIX3_HPP_INCLUDED
