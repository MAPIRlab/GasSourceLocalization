#pragma once
#include <math.h>

namespace Utils
{
    class Vector3;
    inline Vector3 operator/(const Vector3 vec, const float& other);

    class Vector3
    {
    public:
        float x, y, z;

        Vector3()
        {
            x = 0;
            y = 0;
            z = 0;
        }
        Vector3(const float& x, const float& y, const float& z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
        }

        inline Vector3 operator+(const Vector3& other) const
        {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }
        inline Vector3 operator-(const Vector3& other) const
        {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }
        inline void operator+=(const Vector3& other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
        }
        inline void operator-=(const Vector3& other)
        {
            x -= other.x;
            y -= other.y;
            z -= other.z;
        }
        inline Vector3 operator-()
        {
            return Vector3(-x, -y, -z);
        }

        // other stuff
        inline float dot(const Vector3& other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }

        inline Vector3 cross(const Vector3& other) const
        {
            return Vector3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
        }

        inline float norm() const
        {
            return std::sqrt(x * x + y * y + z * z);
        }

        inline Vector3 normalized() const
        {
            float n = norm();
            if (n == 0)
                return {0, 0, 0};
            return *this / n;
        }

        void normalize()
        {
            Vector3 n = normalized();
            x = n.x;
            y = n.y;
        }
    };

    // scalar multiplication
    inline Vector3 operator*(const Vector3 vec, const float& other)
    {
        return Vector3(vec.x * other, vec.y * other, vec.z * other);
    }
    inline Vector3 operator*(const int& other, const Vector3 vec)
    {
        return vec * other;
    }
    inline Vector3 operator*(const float& other, const Vector3 vec)
    {
        return vec * other;
    }

    inline Vector3 operator/(const Vector3 vec, const float& other)
    {
        return Vector3(vec.x / other, vec.y / other, vec.z / other);
    }
} // namespace Utils

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h" // must be included

inline std::ostream& operator<<(std::ostream& os, const Utils::Vector3& v)
{
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}