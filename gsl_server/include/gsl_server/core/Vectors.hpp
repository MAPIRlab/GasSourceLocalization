#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/hash.hpp"

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