#pragma once
#include <opencv2/core.hpp>
#include <Utils/Vector2.h>

namespace Utils
{
    struct Vector2Int
    {
        int x, y;
        Vector2Int()
        {
            x = y = 0;
        }
        Vector2Int(int i, int j)
        {
            x = i;
            y = j;
        }
        Vector2Int(const Vector2Int& other)
        {
            x = other.x;
            y = other.y;
        }

        explicit Vector2Int(const Vector2& point) : x(point.x), y(point.y)
        {
        }

        bool operator==(const Vector2Int& other) const
        {
            return other.x == x && other.y == y;
        }

        bool operator!=(const Vector2Int& other) const
        {
            return other.x != x || other.y != y;
        }

        inline Vector2Int operator+(const Vector2Int& other) const
        {
            return Vector2Int(x + other.x, y + other.y);
        }

        inline Vector2Int operator-(const Vector2Int& other) const
        {
            return Vector2Int(x - other.x, y - other.y);
        }

        inline Vector2Int operator/(const float& other) const
        {
            return Vector2Int(x / other, y / other);
        }

        inline float norm() const
        {
            return std::sqrt(x * x + y * y);
        }

        struct Vec2IntCompare
        {
            bool operator()(const Vector2Int& one, const Vector2Int& other) const
            {
                return one == other;
            }
        };
        struct Vec2IntHash
        {
            size_t operator()(const Vector2Int& one) const
            {
                return one.x * 7 + one.y * 4397;
            }
        };

        operator cv::Point() const
        {
            return cv::Point(x, y);
        }

        operator cv::Point2f() const
        {
            return cv::Point2f(x, y);
        }

        operator Vector2() const
        {
            return Vector2(x, y);
        }
    };

    inline Vector2Int operator*(const Vector2Int& p, const float& f)
    {
        return Vector2Int(p.x * f, p.y * f);
    }
    inline Vector2Int operator*(const float& f, const Vector2Int& p)
    {
        return p * f;
    }
}

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h" // must be included

inline std::ostream& operator<<(std::ostream& os, const Utils::Vector2Int& v)
{
    return os << "(" << v.x << ", " << v.y << ")";
}