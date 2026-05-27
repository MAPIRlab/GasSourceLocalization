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

#include <tf2/LinearMath/Vector3.hpp>
namespace GSL
{
    using Vector2 = glm::vec2;
    using Vector3 = glm::vec3;
    using Vector3Int = glm::ivec3;
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

inline GSL::Vector2 operator*(const GSL::Vector2& vec, float scalar)
{
    return GSL::Vector2(vec.x * scalar, vec.y * scalar);
}

inline GSL::Vector2 operator/(const GSL::Vector2& vec, float scalar)
{
    return GSL::Vector2(vec.x / scalar, vec.y / scalar);
}