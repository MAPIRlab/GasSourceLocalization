#pragma once
#include "gsl_server/core/Macros.hpp"
#include <magic_enum.hpp>

namespace GSL
{

    enum class SemanticsType
    {
        ClassMap2D,
        ClassMapVoxeland
    };

    inline SemanticsType ParseSemanticsType(const std::string& name)
    {
        std::optional<SemanticsType> semanticsType = magic_enum::enum_cast<SemanticsType>(name, magic_enum::case_insensitive);
        if (!semanticsType.has_value())
        {
            constexpr auto names = magic_enum::enum_names<SemanticsType>();
            GSL_ERROR("{} is not a valid semantics type. Valid types are {}", name, fmt::join(names, ", "));
            GSL_ERROR("Closing");
            CLOSE_PROGRAM;
        }
        return *semanticsType;
    }

} // namespace GSL