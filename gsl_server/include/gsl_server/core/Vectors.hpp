#pragma once

//--------------------------
// GSL::vmath is a front-end for multiple possible math libraries
// currently, you can choose between glm and a custom vector implementation that comes with a specific branch of the DDA library

// You should not use the functions of those vector types directly, but instead through vmath functions.
// that way, swapping between libraries is entirely trivial. 
// For example, to find the magnitude of a vector, instead of doing glm::length(v) or v.norm(), which would break when you switch to the other library
// you do vmath::length(v), and the implementation of vmath::length in this file is the only bit of code that needs to change 

// If you need to use any functionality of, say, GLM, that is not exposed through vmath, create a new function in vmath to expose it (and, ideally, make sure a suitable implementation exists for the other libraries)
//--------------------------


#define USE_GLM 0
#if USE_GLM

#define GLM_FORCE_INLINE
#define GLM_FORCE_XYZW_ONLY
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

    template <typename Vec> inline Vec normalized(const Vec& vec)
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

    template <typename Vec> inline Vec normalized(const Vec& vec)
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