#pragma once
#include <math.h>
#include <geometry_msgs/msg/point.hpp>

namespace Utils{
    class Vector2;
    inline Vector2 operator/(const Vector2 vec, const float& other);

    class Vector2{
        public:
            float x,y;

            Vector2(){
                x=0; y=0;
            }
            Vector2(const float& x, const float& y){
                this->x = x;
                this->y = y;
            }

            explicit Vector2(const geometry_msgs::msg::Point& point): x(point.x), y(point.y)
            {}

            inline Vector2 operator+(const Vector2& other) const{
                return Vector2(x+other.x, y+other.y);
            }
            inline Vector2 operator-(const Vector2& other) const{
                return Vector2(x-other.x, y-other.y);
            }
            inline void operator+=(const Vector2& other){
                x+=other.x;
                y+=other.y;
            }
            inline void operator-=(const Vector2& other){
                x-=other.x;
                y-=other.y;
            }
            inline Vector2 operator-(){
                return Vector2(-x, -y);
            }

            //other stuff
            inline float dot(const Vector2& other) const {
                return x*other.x + y*other.y;
            }

            inline float norm() const{
                return std::sqrt(x*x+y*y);
            }

            inline Vector2 normalized() const{
                float n = norm();
                if ( n == 0)
                    return{0,0};

                return *this/n;
            }

            inline void normalize(){
                float n = norm();
                if ( n == 0)
                {
                    x/=n;
                    y/=n;
                }
            }      

            inline Vector2 rotate(float signedAngleRadians) const{
                float c = std::cos(signedAngleRadians);
                float s = std::sin(signedAngleRadians);
                return {x * c - y*s,
                        x * s + y *c};
            }      
    };


    //scalar multiplication
    inline Vector2 operator*(const Vector2 vec, const float& other){
        return Vector2(vec.x*other, vec.y*other);
    }
    inline Vector2 operator*(const float& other, const Vector2 vec){
        return vec*other;
    }

    inline Vector2 operator/(const Vector2 vec, const float& other){
        return Vector2(vec.x/other, vec.y/other);
    }
}

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h" // must be included

inline std::ostream& operator<<(std::ostream& os, const Utils::Vector2& v)
{ 
    return os << "(" <<v.x <<", "<<v.y<<")"; 
}


