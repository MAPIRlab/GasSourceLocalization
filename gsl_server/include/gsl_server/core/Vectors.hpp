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

#include <fmt/format.h>
template <> struct fmt::formatter<GSL::Vector2Int> : formatter<std::string>
{
    auto format(GSL::Vector2Int const& v, format_context& ctx)
    {
        return fmt::format_to(ctx.out(), "({},{})", v.x, v.y);
    }
};

template <> struct fmt::formatter<GSL::Vector2> : formatter<std::string>
{
    auto format(GSL::Vector2 const& v, format_context& ctx)
    {
        return fmt::format_to(ctx.out(), "({},{})", v.x, v.y);
    }
};

template <> struct fmt::formatter<GSL::Vector3> : formatter<std::string>
{
    auto format(GSL::Vector3 const& v, format_context& ctx)
    {
        return fmt::format_to(ctx.out(), "({},{},{})", v.x, v.y, v.z);
    }
};