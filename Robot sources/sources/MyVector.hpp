#ifndef MYVECTOR_HPP_INCLUDED
#define MYVECTOR_HPP_INCLUDED

#include <cmath>
#include <iostream>
#include <iomanip>

template<typename T>
class MyVector3
{
    public:
        //Default
            MyVector3(){ x = 0; y = 0; z = 0; };
            template<typename T2> MyVector3(const MyVector3<T2>& other) : x((T)other.x) , y((T)other.y) , z((T)other.z){};
            MyVector3(T X,T Y,T Z) : x(X),y(Y),z(Z){};
        //

        //Public functions
            //{ Access operator []
                const T& operator[](int i) const { return (&x)[i]; }
                T& operator[](int i) { return (&x)[i]; }
            //}

            //{ Math operator ( += -= *= /= )
                MyVector3& operator+=(const MyVector3& v){x += v.x; y += v.y; z += v.z; return *this;};
                MyVector3& operator-=(const MyVector3& v){x -= v.x; y -= v.y; z -= v.z; return *this;};
                template<typename T2> MyVector3& operator*=(const T2& a) {x *= a; y *= a; z *= a;  return *this;};
                template<typename T2> MyVector3& operator/=(const T2& a) {x /= a; y /= a; z /= a;  return *this;};
            //}

            //{ Math operator ( + - * / )
                MyVector3 operator-() {return MyVector3<T>(-x,-y,-z);};
                MyVector3 operator+(const MyVector3& v) {return MyVector3<T>(v.x+x , v.y+y , v.z+z);};
                MyVector3 operator-(const MyVector3& v) {return MyVector3<T>(x-v.x , y-v.y , z-v.z);};
                template<typename T2> friend MyVector3 operator*(const MyVector3& v,const T2& a) {return MyVector3<T>(v.x*a , v.y*a , v.z*a);};
                template<typename T2> friend MyVector3 operator*(const T2& a,const MyVector3& v) {return MyVector3<T>(v.x*a , v.y*a , v.z*a);};
                template<typename T2> MyVector3 operator/(const T2& a) {return MyVector3<T>(x/a , y/a , z/a);};
            //}

            //{ Comparator operator ( == < > <= >= )
                bool operator==(const MyVector3<T>& b) const
                {
                    if(x!=b.x) return false;
                    else if(y!=b.y) return false;
                    else if(z!=b.z) return false;
                    else return true;
                }
                bool operator!=(const MyVector3<T>& b) const {return !(*this==b);}
                bool operator<(const MyVector3<T>& b) const
                {
                    if(x<b.x) return true;
                    else if(x>b.x) return false;
                    if(y<b.y) return true;
                    else if(y>b.y) return false;
                    if(z<b.x) return true;
                    else return false;
                };
            //}

            //{ Special functions ( * ^ bool() << = length() normalize())
                T length()const{return sqrt(x*x+y*y+z*z);};
                const MyVector3& normalize()
                {
                    T l = sqrt(x*x+y*y+z*z);
                    if(l){x /= l; y /= l; z /= l;}
                    else{x = 0; y = 0; z = 0;}
                    return *this;
                };

                template<typename T2> MyVector3& operator=(const MyVector3<T2>& v){x = (T)v.x; y = (T)v.y; z = (T)v.z; return *this;};
                operator bool()const {if(x||y||z) return true; return false;};
                friend std::ostream& operator<<(std::ostream& os, const MyVector3& v)
                {
                    os<<std::fixed<<std::setprecision(3);
                    os<<"[  "<<v.x<<" "<<std::setw(10)<<v.y<<" "<<std::setw(10) << v.z << " ]\n";
                    os<<std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);
                    return os;
                };

                T operator*(const MyVector3& v){return v.x*x + v.y*y + v.z*z;};
                MyVector3 operator^(const MyVector3& v)
                {
                    MyVector3<T> result;
                        result.x = y*v.z - z*v.y;
                        result.y = z*v.x - x*v.z;
                        result.z = x*v.y - y*v.x;
                    return result;
                };
            //}


        //Attributes
        T x; //!< The x component value.
        T y; //!< The y component value.
        T z; //!< The z component value.
};

typedef MyVector3<float>  MyVector3f;
typedef MyVector3<double> MyVector3d;

//**************************

template<typename T>
class MyVector2
{
    public:
        //Default
            MyVector2(){ x = 0; y = 0; };
            template<typename T2> MyVector2(const MyVector2<T2>& other) : x((T)other.x) , y((T)other.y){};
            MyVector2(T X,T Y):x(X),y(Y){};
        //

        //Public functions
            //{ Access operator []
                const T& operator[](int i) const { return (&x)[i]; }
                T& operator[](int i) { return (&x)[i]; }
            //}

            //{ Math operator ( += -= *= /= )
                MyVector2& operator+=(const MyVector2& v){x += v.x; y += v.y; return *this;};
                MyVector2& operator-=(const MyVector2& v){x -= v.x; y -= v.y; return *this;};
                template<typename T2> MyVector2& operator*=(const T2& a) {x *= a; y *= a;  return *this;};
                template<typename T2> MyVector2& operator/=(const T2& a) {x /= a; y /= a;  return *this;};
            //}

            //{ Math operator ( + - * / )
                MyVector2 operator-() {return MyVector2<T>(-x,-y);};
                MyVector2 operator+(const MyVector2& v) {return MyVector2<T>(v.x+x , v.y+y);};
                MyVector2 operator-(const MyVector2& v) {return MyVector2<T>(x-v.x , y-v.y);};
                template<typename T2> friend MyVector2 operator*(const MyVector2& v,const T2& a) {return MyVector2<T>(v.x*a , v.y*a);};
                template<typename T2> MyVector2 operator/(const T2& a) {return MyVector2<T>(x/a , y/a);};
                template<typename T2> friend MyVector2 operator*(const T2& a,const MyVector2& v) {return MyVector2<T>(v.x*a , v.y*a);};
            //}

            //{ Comparator operator ( == < > <= >= )
                bool operator==(const MyVector2<T>& b) const
                {
                    if(x!=b.x) return false;
                    else if(y!=b.y) return false;
                    else return true;
                }
                bool operator!=(const MyVector3<T>& b) const {return !(*this==b);}
                bool operator<(const MyVector3<T>& b) const
                {
                    if(x<b.x) return true;
                    else if(x>b.x) return false;
                    if(y<b.y) return true;
                    else return false;
                };
            //}

            //{ Special functions ( * bool() << = length() normalize())
                T length()const{return sqrt(x*x+y*y);};
                const MyVector2& normalise()
                {
                    T l = sqrt(x*x+y*y);
                    if(l){x /= l; y /= l;}
                    else{x = 0; y = 0;}
                    return *this;
                };

                template<typename T2> MyVector2& operator=(const MyVector2<T2>& v){x = (T)v.x; y = (T)v.y; return *this;};
                operator bool()const {if(x||y) return true; return false;};
                friend std::ostream& operator<<(std::ostream& os, const MyVector2& v)
                {
                    os<<std::fixed<<std::setprecision(3);
                    os<<"[  "<<v.x<<" "<<std::setw(10)<<v.y<<" ]";
                    os<<std::resetiosflags(std::ios_base::fixed | std::ios_base::floatfield);
                    return os;
                };
                 T operator*(const MyVector2& v){return v.x*x + v.y*y;};
            //}


        //Attributes
        T x; //!< The x component value.
        T y; //!< The y component value.
};

typedef MyVector2<float>  MyVector2f;
typedef MyVector2<double> MyVector2d;

#endif // MYVECTOR_HPP_INCLUDED
