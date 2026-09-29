#pragma once

#include <DDA/Vectors.h>
#include <tf2/LinearMath/Vector3.hpp>
#include "../../third_party/fast_atan2/fast_atan.h"

namespace GSL
{
    using Vector2 = DDA::Vector2;
    using Vector3 = DDA::Vector3;
    using Vector2Int = DDA::Vector2Int;
    using Vector3Int = DDA::Vector3Int;
} // namespace GSL

namespace GSL::vmath
{
    template <typename Vec>
    inline float length(const Vec& vec)
    {
        return vec.norm();
    }

    template <typename Vec>
    inline Vec normalized(const Vec& vec)
    {
        return vec.normalized();
    }

    template <typename Vec>
    inline float angle(const Vec& vec)
    {
        return std::atan2(vec.y, vec.x);
    }

    template <typename Vec>
    inline float angle_fast(const Vec& vec)
    {
        return atan2approx(vec.y, vec.x);
    }

    template <typename Vec>
    inline Vec rotate(const Vec& vec, float signedAngleRadians)
    {
        return vec.rotate(signedAngleRadians);
    }

    inline Vector2 transpose(const Vector2& vec)
    {
        return {vec.y, vec.x};
    }

    inline Vector3 WithZ(const Vector2& vec, float z)
    {
        return Vector3(vec.x, vec.y, z);
    }

    inline Vector2Int ceil(const Vector2& vec)
    {
        return Vector2Int(std::ceil(vec.x), std::ceil(vec.y));
    }

    inline Vector3Int ceil(const Vector3& vec)
    {
        return Vector3Int(std::ceil(vec.x), std::ceil(vec.y), std::ceil(vec.z));
    }

    inline Vector3 fromTF2(const tf2::Vector3& v)
    {
        return Vector3(v.x(), v.y(), v.z());
    }

    inline tf2::Vector3 toTF2(const Vector3& v)
    {
        return tf2::Vector3(v.x, v.y, v.z);
    }
} // namespace GSL::vmath
