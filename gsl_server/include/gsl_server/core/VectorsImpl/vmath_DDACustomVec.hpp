#pragma once


#include <DDA/Vectors.h>

namespace GSL
{
    using Vector2 = DDA::Vector2;
    using Vector3 = DDA::Vector3;
    using Vector2Int = DDA::Vector2Int;
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
    inline Vec rotate(const Vec& vec, float signedAngleRadians)
    {
        return vec.rotate(signedAngleRadians);
    }

    inline Vector2 transpose(const Vector2& vec)
    {
        return {vec.y, vec.x};
    }

} // namespace GSL::vmath