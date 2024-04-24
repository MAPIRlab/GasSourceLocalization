#pragma once

#define USE_GLM 0
#if USE_GLM

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace GSL
{
    using Vector2 = glm::vec2;
    using Vector3 = glm::vec3;
    using Vector2Int = glm::ivec2;
} // namespace GSL

namespace GSL::vmath
{
    template <typename Vec> inline float length(const Vec& vec)
    {
        return glm::length(vec);
    }

    template <typename Vec> inline Vec normalize(const Vec& vec)
    {
        return glm::normalize(vec);
    }

    template <typename Vec> inline Vec rotate(const Vec& vec, float signedAngleRadians)
    {
        return glm::rotate(vec, signedAngleRadians);
    }
} // namespace GSL::VecMath

#else

#include <DDA/Vectors.h>

namespace GSL
{
    using Vector2 = DDA::Vector2;
    using Vector3 = DDA::Vector3;
    using Vector2Int = DDA::Vector2Int;
} // namespace GSL

namespace GSL::vmath
{
    template <typename Vec> inline float length(const Vec& vec)
    {
        return vec.norm();
    }

    template <typename Vec> inline Vec normalize(const Vec& vec)
    {
        return vec.normalized();
    }

    template <typename Vec> inline Vec rotate(const Vec& vec, float signedAngleRadians)
    {
        return vec.rotate(signedAngleRadians);
    }
} // namespace GSL::VecMath
#endif

#ifndef OMIT_FMT
#include <fmt/ostream.h>
inline std::ostream& operator<<(std::ostream& os, const GSL::Vector2Int& v)
{
    return os << "(" << v.x << ", " << v.y << ")";
}

inline std::ostream& operator<<(std::ostream& os, const GSL::Vector2& v)
{
    return os << "(" << v.x << ", " << v.y << ")";
}

inline std::ostream& operator<<(std::ostream& os, const GSL::Vector3& v)
{
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}
#endif