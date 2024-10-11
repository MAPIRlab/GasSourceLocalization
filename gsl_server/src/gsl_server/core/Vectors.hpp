#pragma once

//--------------------------
// GSL::vmath is a front-end for multiple possible math libraries
// currently, you can choose between glm and a custom vector implementation that comes with a specific branch of the DDA library

// You should not use the functions of those vector types directly, but instead through vmath functions.
// that way, swapping between libraries is entirely trivial.
// For example, to find the magnitude of a vector, instead of doing glm::length(v) or v.norm(), which would break when you switch to the other library
// you do vmath::length(v), and the implementation of vmath::length in this file is the only bit of code that needs to change

// If you need to use any functionality of, say, GLM, that is not exposed through vmath, create a new function in vmath to expose it (and, ideally,
// make sure a suitable implementation exists for the other libraries)
//--------------------------

#define USE_GLM 0
#if USE_GLM
#include "VectorsImpl/vmath_glm.hpp"
#else
#include "VectorsImpl/vmath_DDACustomVec.hpp"
#endif

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