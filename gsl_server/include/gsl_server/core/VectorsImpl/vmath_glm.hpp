#pragma once
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
    template <typename Vec>
    inline float length(const Vec& vec)
    {
        return glm::length(vec);
    }

    template <typename Vec>
    inline Vec normalized(const Vec& vec)
    {
        return glm::normalize(vec);
    }

    template <typename Vec>
    inline Vec rotate(const Vec& vec, float signedAngleRadians)
    {
        return glm::rotate(vec, signedAngleRadians);
    }
} // namespace GSL::vmath
